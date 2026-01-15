#include "analog_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <cfloat>
#include <memory>
#include <algorithm>
#include <esp_jpeg_dec.h> // Required for local decoding

namespace esphome {
namespace analog_reader {

static const char *const TAG = "analog_reader";

// Deleter for JPEG decoder handle
struct JpegDecoderDeleter {
    void operator()(jpeg_dec_handle_t handle) const {
        if (handle) jpeg_dec_close(handle);
    }
};

// Local helper to decode JPEG directly into a buffer
static bool local_decode_jpeg(
    const uint8_t* data, size_t len, 
    uint8_t* output_buffer, size_t output_size, 
    int* width, int* height) {
    
    jpeg_dec_io_t io = {0};
    jpeg_dec_header_info_t header_info;
    jpeg_dec_handle_t decoder_handle = nullptr;
    jpeg_dec_config_t decode_config = {
        .output_type = JPEG_PIXEL_FORMAT_RGB888, // Hardcoded to RGB888
        .rotate = JPEG_ROTATE_0D,
    };

    if (jpeg_dec_open(&decode_config, &decoder_handle) != JPEG_ERR_OK) {
        return false;
    }
    // RAII for decoder
    std::unique_ptr<void, JpegDecoderDeleter> decoder(decoder_handle);

    io.inbuf = const_cast<uint8_t*>(data);
    io.inbuf_len = len;
    io.inbuf_remain = len;
    
    // Parse header to check dimensions
    if (jpeg_dec_parse_header(decoder_handle, &io, &header_info) != JPEG_ERR_OK) {
        return false;
    }
    
    *width = header_info.width;
    *height = header_info.height;
    
    size_t required_size = header_info.width * header_info.height * 3;
    if (output_size < required_size) {
        ESP_LOGE(TAG, "Buffer too small for decoding: %u < %u", output_size, required_size);
        return false;
    }
    
    io.outbuf = output_buffer;
    io.out_size = output_size;

    if (jpeg_dec_process(decoder_handle, &io) != JPEG_ERR_OK) {
        return false;
    }

    return true;
}

// Wrapper for Persistent RGB Buffer to implement CameraImage interface
class PersistentDecodedImage : public esphome::camera::CameraImage {
 public:
  PersistentDecodedImage(uint8_t* data, int width, int height, pixformat_t format = PIXFORMAT_RGB888)
      : data_(data), width_(width), height_(height), format_(format) {}

  uint8_t *get_data_buffer() override { return data_; }
  size_t get_data_length() override { 
      return width_ * height_ * (format_ == PIXFORMAT_GRAYSCALE ? 1 : 3); 
  }
  int get_width() const { return width_; }
  int get_height() const { return height_; }
  pixformat_t get_format() const { return format_; }
  
  bool was_requested_by(esphome::camera::CameraRequester requester) const override { return true; }

 protected:
  uint8_t* data_;
  int width_;
  int height_;
  pixformat_t format_;
};

// Configuration constants for Enhanced Radial Profile Analysis (exported for multi_algorithm.cpp)
const float kScanStartRadius = 0.3f;   // Start scan at 30% radius (skip hub/tail)
const float kScanEndRadius = 0.9f;     // End scan at 90% radius (before edge markings)
const float kIntensityWeight = 0.7f;   // Weight for average intensity (70%)
const float kEdgeWeight = 0.3f;        // Weight for edge gradient (30%)
static const float kDecimalPrecision = 10000.0f;  // 4 decimal places (10^4)

// Define static storage
float AnalogReader::sin_lut_[360];
float AnalogReader::cos_lut_[360];
static bool s_luts_initialized = false;

AnalogReader::~AnalogReader() {
  if (rgb_buffer_) {
      free(rgb_buffer_);
      rgb_buffer_ = nullptr;
  }
}

void AnalogReader::set_update_interval(uint32_t interval) {
  PollingComponent::set_update_interval(interval);
  ESP_LOGI(TAG, "Update interval set to %u ms", interval);
}

void AnalogReader::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Analog Reader...");

  // Setup Camera Coordinator
  if (this->camera_) {
      camera_coord_.set_camera((esp32_camera::ESP32Camera*)this->camera_);
  }
  
  if (img_width_ > 0 && img_height_ > 0) {
       camera_coord_.set_config(img_width_, img_height_, pixel_format_str_);
  }

  // Register Listener
  if (this->camera_) {
      this->camera_->add_listener(this);
  }

   // Pre-allocate buffers to prevent heap fragmentation later
   // We prefer PSRAM for this large buffer (up to ~1MB for SVGA) to save internal RAM
   if (pixel_format_str_ == "JPEG") {
        // Force RGB888 for now as JPEG decoder doesn't support direct Grayscale downsampling
        size_t bpp = 3;
        size_t rgb_size = img_width_ * img_height_ * bpp;
        
        ESP_LOGI(TAG, "Allocating persistent RGB buffer: %u bytes (%dx%d)", rgb_size, img_width_, img_height_);
        
        // Try PSRAM first
        rgb_buffer_ = (uint8_t*)heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        
        if (rgb_buffer_) {
            ESP_LOGI(TAG, "Success: Buffer allocated in PSRAM");
        } else {
            ESP_LOGW(TAG, "Failed to allocate in PSRAM, falling back to internal RAM/Default");
            rgb_buffer_ = (uint8_t*)malloc(rgb_size);
        }

        if (rgb_buffer_) {
            rgb_buffer_size_ = rgb_size;
            // Touch memory to enforce allocation
            memset(rgb_buffer_, 0, rgb_size);
        } else {
             ESP_LOGE(TAG, "CRITICAL: Failed to allocate persistent buffer!");
        }
   }

  // Initialize LUTs once
  if (!s_luts_initialized) {
      for (int i = 0; i < 360; i++) {
          float rad = i * M_PI / 180.0f;
          sin_lut_[i] = std::sin(rad);
          cos_lut_[i] = std::cos(rad);
      }
      s_luts_initialized = true;
      ESP_LOGD(TAG, "Trigonometric LUTs initialized");
  }
  
  // Setup Flashlight (Default: None)
  flashlight_coord_.setup(this, nullptr, nullptr);
}

void AnalogReader::dump_config() {
  ESP_LOGCONFIG(TAG, "Analog Reader:");
  ESP_LOGCONFIG(TAG, "  Debug: %s", this->debug_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->get_update_interval());
  for (const auto &dial : dials_) {
      ESP_LOGCONFIG(TAG, "  Dial '%s': Scale=%.3f, Crop=[%d,%d,%d,%d], AutoContrast=%s, Contrast=%.2f", 
          dial.id.c_str(), dial.scale, dial.crop_x, dial.crop_y, dial.crop_w, dial.crop_h,
          dial.auto_contrast ? "ON" : "OFF", dial.contrast);
  }
}

void AnalogReader::update() {
  if (debug_) ESP_LOGD(TAG, "Update triggered (Interval cycle)");

  // Flash scheduling
  if (flashlight_coord_.update_scheduling()) {
      return;
  }

  if (paused_) {
      return;
  }

  // Request frame
  if (!frame_requested_ && !processing_frame_) {
      frame_requested_ = true;
      last_request_time_ = millis();
      ESP_LOGD(TAG, "Requesting frame");
  }
}

void AnalogReader::on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) {
    if (paused_) return;
    if (frame_requested_ && !processing_frame_) {
        // QUICKLY store frame and return to unblock camera thread
        pending_frame_ = image;
    }
}

void AnalogReader::loop() {
// Process pending frame if available
    if (pending_frame_) {
        // Move to local standard pointer
        std::shared_ptr<esphome::camera::CameraImage> frame = pending_frame_;
        pending_frame_ = nullptr;
        
        // OPTIMIZATION: Copy compressed data to local buffer and release frame IMMEDIATELY
        if (frame->get_data_length() > 0) {
             working_buffer_.assign(frame->get_data_buffer(), frame->get_data_buffer() + frame->get_data_length());
             frame.reset(); 
             if (!working_buffer_.empty()) {
                  process_image_from_buffer(working_buffer_.data(), working_buffer_.size());
             }
        }
        
        frame_requested_ = false;
    }

    // Timeout check
    if (frame_requested_ && millis() - last_request_time_ > 5000) {
        ESP_LOGW(TAG, "Frame timeout");
        frame_requested_ = false;
        processing_frame_ = false;
        pending_frame_ = nullptr;
    }
    PollingComponent::loop();
}

// Helpers for image enhancement
static void apply_auto_contrast(uint8_t* data, int size) {
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    for (int i = 0; i < size; i++) {
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }
    
    if (max_val > min_val) {
        float scale = 255.0f / (max_val - min_val);
        for (int i = 0; i < size; i++) {
            data[i] = (uint8_t)((data[i] - min_val) * scale);
        }
    }
}

static void apply_contrast(uint8_t* data, int size, float contrast) {
    if (std::abs(contrast - 1.0f) < 0.01f) return;
    for (int i = 0; i < size; i++) {
        float val = (float)data[i];
        val = (val - 128.0f) * contrast + 128.0f;
        if (val < 0) val = 0; 
        if (val > 255) val = 255;
        data[i] = (uint8_t)val;
    }
}

// Helper class to wrap decoded JPEG buffer
class DecodedImage : public esphome::camera::CameraImage {
 public:
  DecodedImage(esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr &&data, 
               size_t width, size_t height)
      : data_(std::move(data)), width_(width), height_(height),
        size_(width * height * 3) {} // RGB888 = 3 bytes per pixel

  uint8_t *get_data_buffer() override { return data_.get(); }
  size_t get_data_length() override { return size_; }
  bool was_requested_by(camera::CameraRequester requester) const override { return true; }
  
  size_t get_width() const { return width_; }
  size_t get_height() const { return height_; }

  ~DecodedImage() {
      ESP_LOGV("analog_reader", "Destroying DecodedImage (Buffer released)");
  }

 private:
  esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr data_;
  size_t width_;
  size_t height_;
  size_t size_;
};

void AnalogReader::process_image(std::shared_ptr<esphome::camera::CameraImage> image) {
    // Backward compatibility wrapper
    if (image) {
        process_image_from_buffer(image->get_data_buffer(), image->get_data_length());
    }
}

void AnalogReader::process_image_from_buffer(const uint8_t* data, size_t len) {
  if (this->camera_ == nullptr || dials_.empty()) return;

  if (processing_frame_) {
      ESP_LOGW(TAG, "Already processing, skipping");
      return; 
  }
  processing_frame_ = true;

  // Track time
  uint32_t start_time = micros();

  // Basic validation
  ESP_LOGD(TAG, "Process Image From Buffer Start. Len: %u, Free Heap: %u", 
           len, (uint32_t)esp_get_free_heap_size());
           
  // Basic validation
  if (data == nullptr || len == 0) {
       processing_frame_ = false;
       return;
  }
  
  std::shared_ptr<esphome::camera::CameraImage> processing_image = nullptr;
  bool is_decoded = false;
  int processing_w = 0;
  int processing_h = 0;

  if (pixel_format_str_ == "JPEG") {
      // Use persistent buffer if available
      size_t bpp = 3;
      size_t needed_size = img_width_ * img_height_ * bpp;
      
      if (rgb_buffer_ && rgb_buffer_size_ >= needed_size) {
          int out_w = 0, out_h = 0;
          
          // Use LOCAL decoder instead of ImageProcessor
          bool ok = local_decode_jpeg(
              data, len, 
              rgb_buffer_, rgb_buffer_size_, 
              &out_w, &out_h);
          
          if (ok) {
              if (out_w != img_width_ || out_h != img_height_) {
                  ESP_LOGW(TAG, "Decoded dimensions mismatch config: %dx%d vs %dx%d", out_w, out_h, img_width_, img_height_);
              }
              // Wrap the raw pointer
              processing_image = std::make_shared<PersistentDecodedImage>(rgb_buffer_, out_w, out_h, PIXFORMAT_RGB888);
              is_decoded = true;
              processing_w = out_w;
              processing_h = out_h;
              ESP_LOGD(TAG, "Decoded JPEG to RGB888 (Persistent Buffer) (%dx%d)", out_w, out_h);
          } else {
              ESP_LOGE(TAG, "Failed to decode JPEG to persistent buffer.");
          }
      }
      
      // Fallback: If persistent buffer failed or wasn't available, but we still have data
      if (!is_decoded) {
            ESP_LOGW(TAG, "Persistent buffer not used, falling back to dynamic (likely slower/fragmented)");
             // Fallback to dynamic allocation (should not happen if setup ran correctly)
          int w, h;
          auto decoded_buf = esphome::esp32_camera_utils::ImageProcessor::decode_jpeg(
              data, len, &w, &h);
          
          if (decoded_buf) {
              processing_image = std::make_shared<DecodedImage>(std::move(decoded_buf), (size_t)w, (size_t)h);
              is_decoded = true;
              processing_w = w;
              processing_h = h;
              ESP_LOGD(TAG, "Decoded JPEG to RGB888 (Dynamic) (%dx%d)", w, h);
          } else {
               ESP_LOGE(TAG, "Failed to decode JPEG image via fallback.");
          }
      }
  }

  if (!processing_image) {
      ESP_LOGE(TAG, "No valid image available for processing.");
      processing_frame_ = false;
      return;
  }
  
  float total_value = 0.0f;
  std::string debug_str = "";
  
  for (const auto& dial : dials_) {
      // Determine effective image dimensions for checking
      int check_w = (processing_w > 0) ? processing_w : img_width_;
      int check_h = (processing_h > 0) ? processing_h : img_height_;

      // Bounds Check
      if (dial.crop_x + dial.crop_w > check_w || dial.crop_y + dial.crop_h > check_h) {
          ESP_LOGE(TAG, "Dial %s crop is out of bounds! Crop: [x=%d, w=%d] > Img: %d OR [y=%d, h=%d] > Img: %d", 
                   dial.id.c_str(), dial.crop_x, dial.crop_w, check_w, dial.crop_y, dial.crop_h, check_h);
          continue;
      }
      
      // Configure ImageProcessor explicitly
      esphome::esp32_camera_utils::ImageProcessorConfig config;
      if (is_decoded) {
           config.camera_width = processing_w;
           config.camera_height = processing_h;
      } else {
            // Use processing dimensions if available, or config fallback
            config.camera_width = (processing_w > 0) ? processing_w : img_width_;
            config.camera_height = (processing_h > 0) ? processing_h : img_height_;
       }
      
      // Use RGB if Color Detection is enabled
      config.pixel_format = (is_decoded || dial.use_color) ? "RGB888" : pixel_format_str_;
      config.model_width = dial.crop_w;
      config.model_height = dial.crop_h;
      config.model_channels = dial.use_color ? 3 : 1; // 3 for Color, 1 for Grayscale
      config.input_type = esphome::esp32_camera_utils::kInputTypeUInt8;
      
      auto processor = std::make_unique<esphome::esp32_camera_utils::ImageProcessor>(config);

      std::vector<esphome::esp32_camera_utils::CropZone> zones;
      zones.push_back({dial.crop_x, dial.crop_y, dial.crop_x + dial.crop_w, dial.crop_y + dial.crop_h});
      
      // Process
      auto results = processor->split_image_in_zone(processing_image, zones);
      
      if (results.empty() || !results[0].data) {
           ESP_LOGE(TAG, "Failed to process dial %s", dial.id.c_str());
           continue;
      }
      
      uint8_t* raw = results[0].data->get();
      size_t len = results[0].size;
      int crop_w = dial.crop_w;
      int crop_h = dial.crop_h;
      
      const uint8_t* input_for_algo = raw;
      
      if (debug_) ESP_LOGD(TAG, "Processing Dial: %s (Algorithm: %s)", dial.id.c_str(), dial.algorithm.c_str());

      
      // If Color Mode: Convert RGB to Distance Map (Grayscale)
      if (dial.use_color) {
          if (scratch_buffer_.size() != crop_w * crop_h) {
              scratch_buffer_.resize(crop_w * crop_h);
          }
          
          uint8_t tr = (dial.target_color >> 16) & 0xFF;
          uint8_t tg = (dial.target_color >> 8) & 0xFF;
          uint8_t tb = dial.target_color & 0xFF;
          
          for (int i = 0; i < crop_w * crop_h; i++) {
              uint8_t r = raw[i*3 + 0];
              uint8_t g = raw[i*3 + 1];
              uint8_t b = raw[i*3 + 2];
              
              float dist = sqrtf(powf(r - tr, 2) + powf(g - tg, 2) + powf(b - tb, 2));
              // Map 0-442 to 0-255 directly (Close = Dark/Low Value)
              // This ensures compatibility with default needle_type (DARK)
              float val = (dist * 255.0f / 442.0f);
              if (val > 255) val = 255;
              scratch_buffer_[i] = (uint8_t)val;
          }
          input_for_algo = scratch_buffer_.data();
      } else {
          // Standard Grayscale
          // Apply enhancements
          if (dial.auto_contrast) {
              apply_auto_contrast(raw, len);
          }
           if (std::abs(dial.contrast - 1.0f) > 0.01f) {
              apply_contrast(raw, len, dial.contrast);
          }
      }
      
      // Use actual crop dimensions
      uint32_t start_algo = micros();
      DetectionResult result = find_needle_angle(input_for_algo, crop_w, crop_h, dial);
      uint32_t dur_algo = micros() - start_algo;
      
      float angle = result.angle;
      float confidence = result.confidence;

      // Debug angle calculation (detailed logging)
      if (this->debug_) {
          debug_angle_calculation(angle, dial);
      }
      
      float val = angle_to_value(angle, dial);
      
      // Convert to North-based angle for logging clarity 
      float display_angle = angle + 90.0f; 
      if (display_angle >= 360.0f) display_angle -= 360.0f;
      
      ESP_LOGD(TAG, "Dial %s: Angle=%.1f (North), Val=%.2f, Conf=%.2f, Time=%u us", dial.id.c_str(), display_angle, val, confidence, dur_algo);
      
      // Publish to per-dial sensors
      if (dial.angle_sensor) dial.angle_sensor->publish_state(display_angle);
      if (dial.confidence_sensor) dial.confidence_sensor->publish_state(confidence);
      if (dial.value_sensor) {
         dial.value_sensor->publish_state(val);
      }
      
      // Aggregate
      total_value += val * dial.scale;
      
      char val_buf[16];
      snprintf(val_buf, sizeof(val_buf), "%.1f", val);
      debug_str += (debug_str.empty() ? "" : ", ") + std::string(val_buf);
  }
  
  // Truncate to configured decimal precision
  total_value = truncf(total_value * kDecimalPrecision) / kDecimalPrecision;

  ESP_LOGI(TAG, "Result: (Raw: %.4f) [%s]", total_value, debug_str.c_str());
  if (value_sensor_) {
      value_sensor_->publish_state(total_value);
  }

  processing_frame_ = false;
}

AnalogReader::DetectionResult AnalogReader::find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    DetectionResult selected_result;
    
    // Flag to run all algorithms for comparison
    bool run_comparison = (dial.algorithm == "auto") || debug_;

    // Initialize variables to store results of each algorithm
    DetectionResult result_legacy, result_radial, result_hough, result_template;
    bool has_legacy = false, has_radial = false, has_hough = false, has_template = false;

    // Phase 1: Run Algorithms
    
    // 1. Legacy (runs on raw image)
    if (run_comparison || dial.algorithm == "legacy") {
         result_legacy = detect_legacy(img, w, h, dial);
         has_legacy = true;
    }

    // 2. Preprocessing & Modern Algorithms
    if (run_comparison || dial.algorithm != "legacy") {
        // Run preprocessing for all modern algorithms
        preprocess_image(img, w, h, cx, cy, radius, dial.needle_type, working_buffer_);
        uint8_t* processed_data = working_buffer_.data();

        if (run_comparison || dial.algorithm == "radial_profile") {
            result_radial = detect_radial_profile(processed_data, w, h, dial);
            has_radial = true;
        }
        if (run_comparison || dial.algorithm == "hough_transform") {
             result_hough = detect_hough_transform(processed_data, w, h, dial);
             has_hough = true;
        }
        if (run_comparison || dial.algorithm == "template_match") {
             result_template = detect_template_match(processed_data, w, h, dial);
             has_template = true;
        }
    }

    // Phase 2: Select Result
    if (dial.algorithm == "auto") {
        selected_result = result_legacy;
        if (result_radial.confidence > selected_result.confidence) selected_result = result_radial;
        if (result_hough.confidence > selected_result.confidence) selected_result = result_hough;
        if (result_template.confidence > selected_result.confidence) selected_result = result_template;
    } else {
        // Enforce specific choice
        if (dial.algorithm == "legacy") selected_result = result_legacy;
        else if (dial.algorithm == "radial_profile") selected_result = result_radial;
        else if (dial.algorithm == "hough_transform") selected_result = result_hough;
        else if (dial.algorithm == "template_match") selected_result = result_template;
        else selected_result = result_radial; // Default to radial if unknown (or if default was changed in init)
    }

    // Phase 3: Log Comparison (if requested)
    if (run_comparison && debug_) {
        ESP_LOGD(TAG, "%s Algorithm Comparison:", dial.id.c_str());
        if (has_legacy) ESP_LOGD(TAG, "  Legacy: angle=%.1f°, conf=%.2f", result_legacy.angle, result_legacy.confidence);
        if (has_radial) ESP_LOGD(TAG, "  Radial Profile: angle=%.1f°, conf=%.2f", result_radial.angle, result_radial.confidence);
        if (has_hough)  ESP_LOGD(TAG, "  Hough Transform: angle=%.1f°, conf=%.2f", result_hough.angle, result_hough.confidence);
        if (has_template) ESP_LOGD(TAG, "  Template Match: angle=%.1f°, conf=%.2f", result_template.angle, result_template.confidence);
        ESP_LOGD(TAG, "  Selected: %s", selected_result.algorithm.c_str());
    }
    
    if (debug_) {
        ESP_LOGD(TAG, "%s using %s: angle=%.1f°, confidence=%.2f", 
                 dial.id.c_str(), selected_result.algorithm.c_str(), selected_result.angle, selected_result.confidence);
        
        // ASCII visualization - use raw image for legacy, processed for others
        if (dial.algorithm == "legacy") {
            debug_dial_image(img, w, h, selected_result.angle);
        } else {
            // Reuse working_buffer_ which should contain the processed image from above
            // (Assuming find_needle_angle flow is sequential and single-threaded per instance)
            debug_dial_image(working_buffer_.data(), w, h, selected_result.angle);
        }
    }
    
    return selected_result;
}


// Simplified Angle Conversion 
float AnalogReader::angle_to_value(float image_angle, const DialConfig& dial) {
    // SIMPLIFIED VERSION 
    
    // Image: 0° = East (3 o'clock), clockwise
    // Dial: 0° = North (12 o'clock), clockwise
    
    // Conversion: Image to North-based
    // Image 0° (East) = North 90° (East)
    // Image 90° (South) = North 180° (South)  
    // Image 180° (West) = North 270° (West)
    // Image 270° (North) = North 0° (North)
    // Formula: North = (Image + 90) % 360
    
    float dial_angle = fmodf(image_angle + 90.0f, 360.0f);
    
    // Apply user's angle offset
    dial_angle = fmodf(dial_angle - dial.angle_offset + 360.0f, 360.0f);
    
    // Now handle the dial range
    float effective_dial_angle = dial_angle;
    
    // If min_angle > max_angle (e.g., 300 to 60), handle wrap-around
    if (dial.min_angle > dial.max_angle) {
        // Wrap-around case (like 300° to 60°)
        if (effective_dial_angle < dial.min_angle) {
            effective_dial_angle += 360.0f;
        }
        float range = (dial.max_angle + 360.0f) - dial.min_angle;
        float rel_angle = effective_dial_angle - dial.min_angle;
        float fraction = rel_angle / range;
        return dial.min_value + fraction * (dial.max_value - dial.min_value);
    }
    
    // Calibration Map Override (non-linear mapping)
    if (!dial.calibration_mapping.empty()) {
        // Let's use `dial_angle` (North-based, 0-360) as input to map.
        
        float input_x = dial_angle;
        
        // Find interval
        if (input_x <= dial.calibration_mapping.front().first) return dial.calibration_mapping.front().second;
        if (input_x >= dial.calibration_mapping.back().first) return dial.calibration_mapping.back().second;
        
        for (size_t i = 0; i < dial.calibration_mapping.size() - 1; i++) {
            float x1 = dial.calibration_mapping[i].first;
            float y1 = dial.calibration_mapping[i].second;
            float x2 = dial.calibration_mapping[i+1].first;
            float y2 = dial.calibration_mapping[i+1].second;
            
            if (input_x >= x1 && input_x <= x2) {
                // Lerp
                if (std::abs(x2 - x1) < 0.001f) return y1;
                float t = (input_x - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }
        return dial.calibration_mapping.back().second; // Should not reach
    }

    // Normal case: linear mapping
    float fraction = (effective_dial_angle - dial.min_angle) / (dial.max_angle - dial.min_angle);
    fraction = std::max(0.0f, std::min(1.0f, fraction));
    return dial.min_value + fraction * (dial.max_value - dial.min_value);
}

// Debug function for detailed angle conversion logging 
void AnalogReader::debug_angle_calculation(float image_angle, const DialConfig& dial) {
    ESP_LOGD(TAG, "=== DEBUG ANGLE CONVERSION for %s ===", dial.id.c_str());
    ESP_LOGD(TAG, "Input image angle: %.1f° (0°=East, clockwise)", image_angle);
    
    // Current conversion
    float dial_angle = fmodf(image_angle + 90.0f, 360.0f);
    ESP_LOGD(TAG, "Dial angle (after +90 to North): %.1f°", dial_angle);
    
    // Apply offset
    dial_angle = fmodf(dial_angle - dial.angle_offset + 360.0f, 360.0f);
    ESP_LOGD(TAG, "After angle_offset (%.1f): %.1f°", dial.angle_offset, dial_angle);
    
    // Range calculation
    float range_angle = dial.max_angle - dial.min_angle;
    if (dial.min_angle > dial.max_angle) {
        range_angle = (dial.max_angle + 360.0f) - dial.min_angle;
    }
    
    float effective_dial_angle = dial_angle;
    if (dial.min_angle > dial.max_angle && effective_dial_angle < dial.min_angle) {
        effective_dial_angle += 360.0f;
    }
    
    float rel_angle = effective_dial_angle - dial.min_angle;
    
    float fraction = 0.0f;
    if (range_angle > 0) {
        fraction = rel_angle / range_angle;
        fraction = std::max(0.0f, std::min(1.0f, fraction));
    }
    
    float value = dial.min_value + fraction * (dial.max_value - dial.min_value);
    
    ESP_LOGD(TAG, "Range: %.1f° to %.1f° (span=%.1f°)", 
             dial.min_angle, dial.max_angle, range_angle);
    ESP_LOGD(TAG, "Rel angle: %.1f°, Fraction: %.3f", rel_angle, fraction);
    ESP_LOGD(TAG, "Value: %.3f (%.1f to %.1f)", 
             value, dial.min_value, dial.max_value);
    ESP_LOGD(TAG, "=====================================");
}

// Debug visualization
void AnalogReader::debug_dial_image(const uint8_t* img, int w, int h, float detected_angle) {
    // Create ASCII visualization of the dial with needle
    const int grid_w = 40;
    const int grid_h = 40;
    
    std::vector<std::string> lines(grid_h, std::string(grid_w, ' '));
    
    // Initialize grid
    for (int y = 0; y < grid_h; y++) {
        for (int x = 0; x < grid_w; x++) {
            int px = x * w / grid_w;
            int py = y * h / grid_h;
            if (px < w && py < h) {
                uint8_t val = img[py * w + px];
                lines[y][x] = val > 128 ? '.' : '#';
            }
        }
    }
    
    // Draw needle
    float rad = detected_angle * M_PI / 180.0f;
    // We draw in "Grid Space"
    // Grid Space Center
    int cx = grid_w / 2;
    int cy = grid_h / 2;
    
    int max_r = std::min(cx, cy);
    
    for (int r = max_r/5; r < max_r; r++) { // Start 20% out
        // Map the needle angle to the grid coordinates.
        // We use a normalized coordinate system (0.0-1.0) and map it to the grid dimensions.
        float scale = (float)r / max_r; // Scale factor for line drawing
        
        int draw_x = (int)( (0.5f + 0.5f * cos(rad) * scale) * grid_w );
        int draw_y = (int)( (0.5f + 0.5f * sin(rad) * scale) * grid_h );
        
        if (draw_x >= 0 && draw_x < grid_w && draw_y >= 0 && draw_y < grid_h) {
            lines[draw_y][draw_x] = 'X';
        }
    }
    
    // Calculate North Angle for display
    float north_angle = fmodf(90.0f - detected_angle + 360.0f, 360.0f);

    // ANSI color codes for better visualization
    const char* COLOR_RESET = "\033[0m";
    const char* COLOR_RED = "\033[91m";      // Bright red for needle
    const char* COLOR_GREEN = "\033[92m";    // Green for center marker
    
    // Print header with legend
    ESP_LOGD(TAG, "Dial visualization (Raw %.1f / North %.1f):", detected_angle, north_angle);
    ESP_LOGD(TAG, "Legend: '.' = Light pixels | '#' = Dark pixels | %sX%s = Detected needle", 
             COLOR_RED, COLOR_RESET);
    
    // Print grid with colors
    for (int y = 0; y < grid_h; y++) {
        std::string colored_line = "|";
        for (int x = 0; x < grid_w; x++) {
            char c = lines[y][x];
            if (c == 'X') {
                // Needle in RED
                colored_line += COLOR_RED;
                colored_line += c;
                colored_line += COLOR_RESET;
            } else if (x == cx && y == cy) {
                // Center marker in GREEN
                colored_line += COLOR_GREEN;
                colored_line += '+';
                colored_line += COLOR_RESET;
            } else {
                colored_line += c;
            }
        }
        colored_line += "|";
        ESP_LOGD(TAG, "%s", colored_line.c_str());
    }
}

void AnalogReader::set_dial_range(std::string dial_id, float min_val, float max_val) {
    for (auto &dial : dials_) {
        if (dial.id == dial_id) {
            dial.min_value = min_val;
            dial.max_value = max_val;
            ESP_LOGI(TAG, "Updated dial '%s' range to %.2f - %.2f", dial_id.c_str(), min_val, max_val);
            return;
        }
    }
    ESP_LOGW(TAG, "Dial '%s' not found for set_dial_range", dial_id.c_str());
}

void AnalogReader::set_dial_angle(std::string dial_id, float min_deg, float max_deg) {
    for (auto &dial : dials_) {
        if (dial.id == dial_id) {
            dial.min_angle = min_deg;
            dial.max_angle = max_deg;
            ESP_LOGI(TAG, "Updated dial '%s' angle limits to %.1f - %.1f", dial_id.c_str(), min_deg, max_deg);
            return;
        }
    }
    ESP_LOGW(TAG, "Dial '%s' not found for set_dial_angle", dial_id.c_str());
}

}  // namespace analog_reader
}  // namespace esphome
