#include "analog_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <cfloat>
#include <memory>
#include <algorithm>
#include "esphome/components/esp32_camera_utils/image_processor.h"
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
    jpeg_pixel_format_t output_format,
    uint8_t* output_buffer, size_t output_size, 
    int* width, int* height) {
    
    jpeg_dec_io_t io = {0};
    jpeg_dec_header_info_t header_info;
    jpeg_dec_handle_t decoder_handle = nullptr;
    jpeg_dec_config_t decode_config = {
        .output_type = output_format,
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
    
    size_t bpp = (output_format == JPEG_PIXEL_FORMAT_GRAY) ? 1 : 3;
    size_t required_size = header_info.width * header_info.height * bpp;
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
extern const float kScanStartRadius = 0.3f;   // Start scan at 30% radius (skip hub/tail)
extern const float kScanEndRadius = 0.9f;     // End scan at 90% radius (before edge markings)
extern const float kIntensityWeight = 0.7f;   // Weight for average intensity (70%)
extern const float kEdgeWeight = 0.3f;        // Weight for edge gradient (30%)
static const float kDecimalPrecision = 10000.0f;  // 4 decimal places (10^4)

// Define static storage
float AnalogReader::sin_lut_[360];
float AnalogReader::cos_lut_[360];
static bool s_luts_initialized = false;

AnalogReader::~AnalogReader() {
  // persistent_buffer_ is a unique_ptr, so it auto-releases.
  // Explicit reset is fine but not strictly needed unless we want to force earlier release.
  this->persistent_buffer_.reset();
}

void AnalogReader::set_update_interval(uint32_t interval) {
  PollingComponent::set_update_interval(interval);
  ESP_LOGI(TAG, "Update interval set to %u ms", interval);
}

void AnalogReader::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Analog Reader...");

  // Register Listener
  if (this->camera_) {
      this->camera_->add_listener(this);
  }

   // Sort calibration mappings for correct interpolation
   for (auto &dial : this->dials_) {
       if (!dial.calibration_mapping.empty()) {
           std::sort(dial.calibration_mapping.begin(), dial.calibration_mapping.end(), 
               [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                   return a.first < b.first;
               });
           ESP_LOGD(TAG, "Sorted calibration mapping for dial %s (%d points)", dial.id.c_str(), static_cast<int>(dial.calibration_mapping.size()));
       }
   }

   // Pre-allocate buffers to prevent heap fragmentation later
   // We prefer PSRAM for this large buffer (up to ~1MB for SVGA)
   if (this->pixel_format_str_ == "JPEG") {
        size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "Available PSRAM: %u bytes", static_cast<uint32_t>(free_psram));

        // 1. Determine requirements
        this->requires_color_ = false;
        for (const auto &dial : this->dials_) {
            if (dial.use_color || dial.process_channel != PROCESS_CHANNEL_GRAYSCALE) {
                this->requires_color_ = true;
                break;
            }
        }
        
        // 2. Decide Format based on PSRAM
        // Force RGB888 if color is required
        // Prefer RGB888 if sufficient RAM (faster/simpler than on-the-fly grayscale conversion logic elsewhere?)
        // Actually, converting directly to Grayscale saves memory which is the main constraint.
        // If we have TONS of RAM, we use RGB to be safe/future proof. 
        // If we are tight, we drop to Gray.

        size_t rgb_size = this->img_width_ * this->img_height_ * 3;
        size_t gray_size = this->img_width_ * this->img_height_ * 1;
        
        // Threshold: Buffer + Safety Margin
        // TFLite arena is small (<200KB), BUT ImageProcessor needs a full RGB framebuffer (~920KB)
        // to decode images for cropping. We must reserve enough RAM for that concurrent allocation.
        // Safety Margin: ~1.5MB (1MB for Framebuffer + 500KB for Arena/Stack/Overhead)
        bool sufficient_psram_for_rgb = free_psram > (rgb_size + 1536 * 1024);

        if (this->requires_color_ || sufficient_psram_for_rgb) {
            this->buffer_format_ = PIXFORMAT_RGB888;
            this->persistent_buffer_size_ = rgb_size;
            ESP_LOGI(TAG, "Selected RGB888 format for persistent buffer (ReqColor=%s, SuffPSRAM=%s)", 
                this->requires_color_ ? "YES" : "NO", sufficient_psram_for_rgb ? "YES" : "NO");
        } else {
            this->buffer_format_ = PIXFORMAT_GRAYSCALE;
            this->persistent_buffer_size_ = gray_size;
            ESP_LOGI(TAG, "Selected GRAYSCALE format for persistent buffer (Low PSRAM Mode)");
        }
        
        ESP_LOGI(TAG, "Allocating persistent buffer: %u bytes (%dx%d)", static_cast<uint32_t>(this->persistent_buffer_size_), this->img_width_, this->img_height_);
        
        // Try PSRAM first
        this->persistent_buffer_.reset(static_cast<uint8_t *>(heap_caps_malloc(this->persistent_buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
        
        if (this->persistent_buffer_) {
            ESP_LOGI(TAG, "Success: Buffer allocated in PSRAM");
        } else {
            ESP_LOGW(TAG, "Failed to allocate in PSRAM, falling back to internal RAM");
            this->persistent_buffer_.reset(static_cast<uint8_t *>(malloc(this->persistent_buffer_size_)));
        }

        if (this->persistent_buffer_) {
            // Touch memory to enforce allocation
            memset(this->persistent_buffer_.get(), 0, this->persistent_buffer_size_);
        } else {
             ESP_LOGE(TAG, "CRITICAL: Failed to allocate persistent buffer!");
        }
   }

  // Initialize LUTs once
  if (!s_luts_initialized) {
      for (int i = 0; i < 360; i++) {
          float rad = i * static_cast<float>(M_PI) / 180.0f;
          AnalogReader::sin_lut_[i] = std::sin(rad);
          AnalogReader::cos_lut_[i] = std::cos(rad);
      }
      s_luts_initialized = true;
      ESP_LOGD(TAG, "Trigonometric LUTs initialized");
  }
  
  // Setup Flashlight (Default: None)
  this->flashlight_coord_.setup(this, nullptr, nullptr);
}

void AnalogReader::dump_config() {
  ESP_LOGCONFIG(TAG, "Analog Reader:");
  ESP_LOGCONFIG(TAG, "  Debug: %s", this->debug_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->get_update_interval());
  for (const auto &dial : this->dials_) {
      ESP_LOGCONFIG(TAG, "  Dial '%s': Scale=%.3f, Crop=[%d,%d,%d,%d], AutoContrast=%s, Contrast=%.2f", 
          dial.id.c_str(), dial.scale, dial.crop_x, dial.crop_y, dial.crop_w, dial.crop_h,
          dial.auto_contrast ? "ON" : "OFF", dial.contrast);
  }
}

void AnalogReader::update() {
  if (this->debug_) ESP_LOGD(TAG, "Update triggered (Interval cycle)");

  // The instruction requested to remove usages of `camera_coord_` in `set_debug()`.
  // As `set_debug()` is not present in the provided code, and the snippet provided
  // in the instruction was malformed and referred to `flashlight_coord_`,
  // no changes are made here.
  // The line `flashlight_coord_.set_debug(debug);` was not part of the original document.
  // The original document's `update()` function is preserved as is.

  if (this->flashlight_coord_.update_scheduling()) {
      return;
  }

  if (this->paused_) {
      return;
  }

  // Request frame
  if (!this->frame_requested_ && !this->processing_frame_) {
      this->frame_requested_ = true;
      this->last_request_time_ = millis();
      ESP_LOGD(TAG, "Requesting frame");
  }
}

void AnalogReader::on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) {
    std::lock_guard<std::mutex> lock(this->frame_mutex_);
    if (this->paused_) return;
    if (this->frame_requested_ && !this->processing_frame_) {
        // QUICKLY store frame and return to unblock camera thread
        this->pending_frame_ = image;
    }
}

void AnalogReader::loop() {
// Process pending frame if available
// Process pending frame if available
    std::shared_ptr<esphome::camera::CameraImage> frame;
    {
        std::lock_guard<std::mutex> lock(this->frame_mutex_);
        if (this->pending_frame_) {
            frame = this->pending_frame_;
            this->pending_frame_ = nullptr;
        }
    }

    if (frame) {
        // OPTIMIZATION: Pass frame buffer directly to processing instead of copying to working_buffer_.
        // We hold the frame during processing, which is fine since we don't request a new one until we are done.
        if (frame->get_data_length() > 0) {
             this->process_image_from_buffer(frame->get_data_buffer(), frame->get_data_length());
        }
        frame.reset(); // Release frame after processing
        
        this->frame_requested_ = false;
    }

    // Timeout check
    if (this->frame_requested_ && millis() - this->last_request_time_ > 5000) {
        ESP_LOGW(TAG, "Frame timeout");
        this->frame_requested_ = false;
        this->processing_frame_ = false;
        this->pending_frame_ = nullptr;
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
        float scale = 255.0f / static_cast<float>(max_val - min_val);
        for (int i = 0; i < size; i++) {
            data[i] = static_cast<uint8_t>(static_cast<float>(data[i] - min_val) * scale);
        }
    }
}

static void apply_contrast(uint8_t* data, int size, float contrast) {
    if (std::abs(contrast - 1.0f) < 0.01f) return;
    for (int i = 0; i < size; i++) {
        float val = static_cast<float>(data[i]);
        val = (val - 128.0f) * contrast + 128.0f;
        if (val < 0) val = 0; 
        if (val > 255) val = 255;
        data[i] = static_cast<uint8_t>(val);
    }
}

// Helper class to wrap decoded JPEG buffer
class DecodedImage : public esphome::camera::CameraImage {
 public:
  DecodedImage(esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr &&data, 
               size_t width, size_t height)
      : data_(std::move(data)), width_(width), height_(height),
        size_(width * height * 3) {} // RGB888 = 3 bytes per pixel

  uint8_t *get_data_buffer() override { return this->data_.get(); }
  size_t get_data_length() override { return this->size_; }
  bool was_requested_by(camera::CameraRequester requester) const override { return true; }
  
  size_t get_width() const { return this->width_; }
  size_t get_height() const { return this->height_; }

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
        this->process_image_from_buffer(image->get_data_buffer(), image->get_data_length());
    }
}

void AnalogReader::process_image_from_buffer(const uint8_t* data, size_t len) {
  if (this->camera_ == nullptr || this->dials_.empty()) return;

  if (this->processing_frame_) {
      ESP_LOGW(TAG, "Already processing, skipping");
      return; 
  }
  this->processing_frame_ = true;

  // Track time
  uint32_t start_time = micros();

  // Basic validation
  ESP_LOGD(TAG, "Process Image From Buffer Start. Len: %u, Free Heap: %u", 
           len, static_cast<uint32_t>(esp_get_free_heap_size()));
           
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
      // Check if buffer size matches current requirements (simple validity check)
      
      if (persistent_buffer_) {
          int out_w = 0, out_h = 0;
          
          jpeg_pixel_format_t out_fmt = (this->buffer_format_ == PIXFORMAT_GRAYSCALE) 
                                        ? JPEG_PIXEL_FORMAT_GRAY 
                                        : JPEG_PIXEL_FORMAT_RGB888;

          // Use LOCAL decoder instead of ImageProcessor
          bool ok = local_decode_jpeg(
              data, len, 
              out_fmt,
              this->persistent_buffer_.get(), this->persistent_buffer_size_, 
              &out_w, &out_h);
          
          if (ok) {
              if (out_w != this->img_width_ || out_h != this->img_height_) {
                  ESP_LOGW(TAG, "Decoded dimensions mismatch config: %dx%d vs %dx%d. Skipping analog processing to avoid conflict.", out_w, out_h, this->img_width_, this->img_height_);
                  this->processing_frame_ = false;
                  return;
              }
              // Wrap the raw pointer
              // Note: PersistentDecodedImage expects a raw pointer. We pass the managed pointer's content.
              // The PersistentDecodedImage does NOT own the memory, so it must not free it.
              // We need to check PersistentDecodedImage implementation to ensure it doesn't free.
              // Checking previous code: PersistentDecodedImage just holds uint8_t* data_; and has no destructor freeing it. Correct.
              processing_image = std::make_shared<PersistentDecodedImage>(this->persistent_buffer_.get(), out_w, out_h, this->buffer_format_);
              is_decoded = true;
              processing_w = out_w;
              processing_h = out_h;
              ESP_LOGD(TAG, "Decoded JPEG to %s (Persistent Buffer) (%dx%d)", 
                  (this->buffer_format_ == PIXFORMAT_GRAYSCALE ? "GRAY" : "RGB888"), out_w, out_h);
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
              processing_image = std::make_shared<DecodedImage>(std::move(decoded_buf), static_cast<size_t>(w), static_cast<size_t>(h));
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
      this->processing_frame_ = false;
      return;
  }
  
  float total_value = 0.0f;
  std::string debug_str = "";
  
  for (const auto& dial : this->dials_) {
      // Determine effective image dimensions for checking
      int check_w = (processing_w > 0) ? processing_w : this->img_width_;
      int check_h = (processing_h > 0) ? processing_h : this->img_height_;

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
            config.camera_width = (processing_w > 0) ? processing_w : this->img_width_;
            config.camera_height = (processing_h > 0) ? processing_h : this->img_height_;
       }
      
      // Use RGB if Color Detection is enabled OR specific Channel processing is requested
      // Note: is_decoded implies we have the full image, but check format!
      bool need_rgb = dial.use_color || (dial.process_channel != PROCESS_CHANNEL_GRAYSCALE);
      
      // If we only have grayscale buffer, but need RGB... we have a problem.
      // Setup logic should prevent this (requires_color_ would force RGB if need_rgb is true).
      // But double check buffer format:
      if (need_rgb && this->buffer_format_ == PIXFORMAT_GRAYSCALE) {
          ESP_LOGE(TAG, "Dial %s needs color but buffer is Grayscale! Check setup logic.", dial.id.c_str());
          continue; 
      }

      config.pixel_format = need_rgb ? "RGB888" : "GRAYSCALE"; 
      config.model_width = dial.crop_w;
      config.model_height = dial.crop_h;
      config.model_channels = need_rgb ? 3 : 1; // 3 for Color, 1 for Grayscale
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
      
      if (this->debug_) ESP_LOGD(TAG, "Processing Dial: %s (Algorithm: %s)", dial.id.c_str(), dial.algorithm.c_str());

      // Channel Filtering (Red/Green/Blue)
      if (dial.process_channel != PROCESS_CHANNEL_GRAYSCALE && !dial.use_color) {
          if (this->scratch_buffer_.size() != static_cast<size_t>(crop_w * crop_h)) {
              this->scratch_buffer_.resize(crop_w * crop_h);
          }
          
          int offset = 0;
          if (dial.process_channel == PROCESS_CHANNEL_RED) offset = 0;
          else if (dial.process_channel == PROCESS_CHANNEL_GREEN) offset = 1;
          else if (dial.process_channel == PROCESS_CHANNEL_BLUE) offset = 2;
          
          // Determine if input is RGB or Grayscale
          // If we requested RGB because of process_channel, input should be RGB (3 bytes)
          bool input_is_rgb = (config.model_channels == 3);
          
          if (input_is_rgb) {
              for (int i = 0; i < crop_w * crop_h; i++) {
                  // Safety check for raw buffer access if needed, but crop_w*crop_h*3 should match results size
                  this->scratch_buffer_[i] = raw[i*3 + offset];
              }
              input_for_algo = this->scratch_buffer_.data();
              
              // Apply contrast/auto-contrast to the single channel if needed
              if (dial.auto_contrast) {
                  apply_auto_contrast(this->scratch_buffer_.data(), crop_w * crop_h);
              }
              if (std::abs(dial.contrast - 1.0f) > 0.01f) {
                  apply_contrast(this->scratch_buffer_.data(), crop_w * crop_h, dial.contrast);
              }
          } else {
             // Fallback if somehow we got grayscale (shouldn't happen if logic is correct below)
             // input_for_algo already points to raw (grayscale)
             ESP_LOGW(TAG, "Dial %s configured for channel %d but input is Grayscale", dial.id.c_str(), static_cast<int>(dial.process_channel));
          }
      } 
      // Existing Color Mode: Convert RGB to Distance Map (Grayscale)
      else if (dial.use_color) {
          if (this->scratch_buffer_.size() != static_cast<size_t>(crop_w * crop_h)) {
              this->scratch_buffer_.resize(crop_w * crop_h);
          }
          
          uint8_t tr = (dial.target_color >> 16) & 0xFF;
          uint8_t tg = (dial.target_color >> 8) & 0xFF;
          uint8_t tb = dial.target_color & 0xFF;
          
          for (int i = 0; i < crop_w * crop_h; i++) {
              uint8_t r = raw[i*3 + 0];
              uint8_t g = raw[i*3 + 1];
              uint8_t b = raw[i*3 + 2];
              
              float dist = sqrtf(powf(static_cast<float>(r - tr), 2) + powf(static_cast<float>(g - tg), 2) + powf(static_cast<float>(b - tb), 2));
              // Map 0-442 to 0-255 directly (Close = Dark/Low Value)
              // This ensures compatibility with default needle_type (DARK)
              float val = (dist * 255.0f / 442.0f);
              if (val > 255) val = 255;
              this->scratch_buffer_[i] = static_cast<uint8_t>(val);
          }
          input_for_algo = this->scratch_buffer_.data();
      } else if (dial.process_channel == PROCESS_CHANNEL_GRAYSCALE) {
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
      DetectionResult result = this->find_needle_angle(input_for_algo, crop_w, crop_h, dial);
      uint32_t dur_algo = micros() - start_algo;
      
      float angle = result.angle;
      float confidence = result.confidence;

      // Debug angle calculation (detailed logging)
      if (this->debug_) {
          this->debug_angle_calculation(angle, dial);
      }
      
      float val = this->angle_to_value(angle, dial);
      
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
  if (this->value_sensor_) {
      this->value_sensor_->publish_state(total_value);
  }

  this->processing_frame_ = false;
}

AnalogReader::DetectionResult AnalogReader::find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    DetectionResult selected_result;
    
    // Flag to run all algorithms for comparison
    bool run_comparison = (dial.algorithm == "auto") || this->debug_;

    // Initialize variables to store results of each algorithm
    DetectionResult result_legacy, result_radial, result_hough, result_template;
    bool has_legacy = false, has_radial = false, has_hough = false, has_template = false;

    // Phase 1: Run Algorithms
    
    // 1. Legacy (runs on raw image)
    if (run_comparison || dial.algorithm == "legacy") {
         result_legacy = this->detect_legacy(img, w, h, dial);
         has_legacy = true;
    }

    // 2. Preprocessing & Modern Algorithms
    if (run_comparison || dial.algorithm != "legacy") {
        // Run preprocessing for all modern algorithms
        this->preprocess_image(img, w, h, cx, cy, radius, dial.needle_type, this->working_buffer_);
        uint8_t* processed_data = this->working_buffer_.data();

        if (run_comparison || dial.algorithm == "radial_profile") {
            result_radial = this->detect_radial_profile(processed_data, w, h, dial);
            has_radial = true;
        }
        if (run_comparison || dial.algorithm == "hough_transform") {
             result_hough = this->detect_hough_transform(processed_data, w, h, dial);
             has_hough = true;
        }
        if (run_comparison || dial.algorithm == "template_match") {
             result_template = this->detect_template_match(processed_data, w, h, dial);
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
    if (run_comparison && this->debug_) {
        ESP_LOGD(TAG, "%s Algorithm Comparison:", dial.id.c_str());
        if (has_legacy) ESP_LOGD(TAG, "  Legacy: angle=%.1f°, val=%.2f, conf=%.2f", result_legacy.angle, this->angle_to_value(result_legacy.angle, dial), result_legacy.confidence);
        if (has_radial) ESP_LOGD(TAG, "  Radial Profile: angle=%.1f°, val=%.2f, conf=%.2f", result_radial.angle, this->angle_to_value(result_radial.angle, dial), result_radial.confidence);
        if (has_hough)  ESP_LOGD(TAG, "  Hough Transform: angle=%.1f°, val=%.2f, conf=%.2f", result_hough.angle, this->angle_to_value(result_hough.angle, dial), result_hough.confidence);
        if (has_template) ESP_LOGD(TAG, "  Template Match: angle=%.1f°, val=%.2f, conf=%.2f", result_template.angle, this->angle_to_value(result_template.angle, dial), result_template.confidence);
        ESP_LOGD(TAG, "  Selected: %s", selected_result.algorithm.c_str());
    }
    
    if (this->debug_) {
        ESP_LOGD(TAG, "%s using %s: angle=%.1f°, confidence=%.2f", 
                 dial.id.c_str(), selected_result.algorithm.c_str(), selected_result.angle, selected_result.confidence);
        
        // ASCII visualization - use raw image for legacy, processed for others
        if (dial.algorithm == "legacy") {
            this->debug_dial_image(img, w, h, selected_result.angle);
        } else {
            // Reuse working_buffer_ which should contain the processed image from above
            // (Assuming find_needle_angle flow is sequential and single-threaded per instance)
            this->debug_dial_image(this->working_buffer_.data(), w, h, selected_result.angle);
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

    // 0-85: # in Dark Gray
    // 86-170: + in Light Gray
    // 171-255: . in Bright White
    
    std::vector<std::string> lines(grid_h, std::string(grid_w, ' '));
    std::vector<uint8_t> grid_vals(grid_w * grid_h, 0);
    
    // Initialize grid
    for (int y = 0; y < grid_h; y++) {
        for (int x = 0; x < grid_w; x++) {
            int px = x * w / grid_w;
            int py = y * h / grid_h;
            if (px < w && py < h) {
                uint8_t val = img[py * w + px];
                grid_vals[y * grid_w + x] = val;
                
                if (val <= 85) lines[y][x] = '#';
                else if (val <= 170) lines[y][x] = '+';
                else lines[y][x] = '.';
            }
        }
    }
    
    // Draw needle
    float rad = detected_angle * static_cast<float>(M_PI) / 180.0f;
    // We draw in "Grid Space"
    // Grid Space Center
    int cx = grid_w / 2;
    int cy = grid_h / 2;
    
    int max_r = std::min(cx, cy);
    
    for (int r = max_r/5; r < max_r; r++) { // Start 20% out
        int draw_x = cx + static_cast<int>(cos(rad) * static_cast<float>(r) * static_cast<float>(grid_w) / static_cast<float>(max_r * 2));
        int draw_y = cy + static_cast<int>(sin(rad) * static_cast<float>(r) * static_cast<float>(grid_h) / static_cast<float>(max_r * 2));
        
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
    const char* COLOR_LOW = "\033[90m";      // Dark Gray for low values
    const char* COLOR_MID = "\033[37m";      // Light Gray for mid values
    const char* COLOR_HIGH = "\033[97m";     // Bright white for high values
    
    // Print header with legend
    ESP_LOGD(TAG, "Dial visualization (Raw %.1f / North %.1f):", detected_angle, north_angle);
    ESP_LOGD(TAG, "Legend: '%s#'%s=0-85 | '%s+'%s=86-170 | '%s.'%s=171-255 | %sX%s = Detected needle", 
             COLOR_LOW, COLOR_RESET, COLOR_MID, COLOR_RESET, COLOR_HIGH, COLOR_RESET, COLOR_RED, COLOR_RESET);
    
    // Print grid with colors
    for (int y = 0; y < grid_h; y++) {
        std::string colored_line = "|";
        for (int x = 0; x < grid_w; x++) {
            char c = lines[y][x];
            uint8_t val = grid_vals[y * grid_w + x];
            
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
            } else if (c != ' ') {
                // Color based on value
                if (val <= 85) colored_line += COLOR_LOW;
                else if (val <= 170) colored_line += COLOR_MID;
                else colored_line += COLOR_HIGH;
                
                colored_line += c;
                colored_line += COLOR_RESET;
            } else {
                colored_line += c;
            }
            colored_line += ' '; // Adjust with spaces
        }
        colored_line += "|";
        ESP_LOGD(TAG, "%s", colored_line.c_str());
    }
}

void AnalogReader::set_dial_range(const std::string &dial_id, float min_val, float max_val) {
    for (auto &dial : this->dials_) {
        if (dial.id == dial_id) {
            dial.min_value = min_val;
            dial.max_value = max_val;
            ESP_LOGI(TAG, "Updated dial '%s' range to %.2f - %.2f", dial_id.c_str(), min_val, max_val);
            return;
        }
    }
    ESP_LOGW(TAG, "Dial '%s' not found for set_dial_range", dial_id.c_str());
}

void AnalogReader::set_dial_angle(const std::string &dial_id, float min_deg, float max_deg) {
    for (auto &dial : this->dials_) {
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
