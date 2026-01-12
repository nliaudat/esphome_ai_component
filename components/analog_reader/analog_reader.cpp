#include "analog_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <cfloat>
#include <memory>
#include <algorithm>

namespace esphome {
namespace analog_reader {

static const char *const TAG = "analog_reader";

// Configuration constants for Enhanced Radial Profile Analysis (exported for multi_algorithm.cpp)
const float kScanStartRadius = 0.3f;   // Start scan at 30% radius (skip hub/tail)
const float kScanEndRadius = 0.9f;     // End scan at 90% radius (before edge markings)
const float kIntensityWeight = 0.7f;   // Weight for average intensity (70%)
const float kEdgeWeight = 0.3f;        // Weight for edge gradient (30%)
static const float kDecimalPrecision = 10000.0f;  // 4 decimal places (10^4)

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
  
  // Setup Validation
  // External component handles this


  // Register Listener
  if (this->camera_) {
      this->camera_->add_listener(this);
  }
  
  // Configure ImageProcessor - Now handled per-dial in process_image
  // camera_coord_.update_image_processor_config(...)
  
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
        // Simple lock logic
        process_image(image);
        frame_requested_ = false;
    }
}

void AnalogReader::loop() {
    // Timeout check
    if (frame_requested_ && millis() - last_request_time_ > 5000) {
        ESP_LOGW(TAG, "Frame timeout");
        frame_requested_ = false;
        processing_frame_ = false;
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
      // Trace log for memory debugging
      ESP_LOGV("analog_reader", "Destroying DecodedImage (Buffer released)");
  }

 private:
  esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr data_;
  size_t width_;
  size_t height_;
  size_t size_;
};

void AnalogReader::process_image(std::shared_ptr<esphome::camera::CameraImage> image) {
  if (this->camera_ == nullptr || dials_.empty()) return;
  
  processing_frame_ = true;
  
  ESP_LOGD(TAG, "Process Image Start. Valid: %s, Free Heap: %u", 
           image ? "YES" : "NO", (uint32_t)esp_get_free_heap_size());
  
  // OPTIMIZATION: Decode JPEG once if needed
  std::shared_ptr<esphome::camera::CameraImage> processing_image = image;
  bool is_decoded = false;
  int processing_w = img_width_;
  int processing_h = img_height_;

  if (pixel_format_str_ == "JPEG") {
      int w, h;
      auto decoded_buf = esphome::esp32_camera_utils::ImageProcessor::decode_jpeg(
          image->get_data_buffer(), image->get_data_length(), &w, &h);
      
      if (decoded_buf) {
          // Wrap in DecodedImage (RGB888)
          // Use our wrapper which takes ownership
          // Width/Height are int from decode_jpeg, cast to size_t
          processing_image = std::make_shared<DecodedImage>(std::move(decoded_buf), (size_t)w, (size_t)h);
          is_decoded = true;
          processing_w = w;
          processing_h = h;
          ESP_LOGD(TAG, "Decoded JPEG to RGB888 (%dx%d) for batch processing", w, h);
      } else {
           ESP_LOGE(TAG, "Failed to decode JPEG image. Free Heap: %u", (uint32_t)esp_get_free_heap_size());
           processing_frame_ = false;
           return;
      }
  }

  float total_value = 0.0f;
  std::string debug_str = "";
  
  for (const auto& dial : dials_) {
      // Bounds Check
      if (dial.crop_x + dial.crop_w > img_width_ || dial.crop_y + dial.crop_h > img_height_) {
          ESP_LOGE(TAG, "Dial %s crop is out of bounds!", dial.id.c_str());
          continue;
      }
      
      // Configure ImageProcessor explicitly
      esphome::esp32_camera_utils::ImageProcessorConfig config;
      if (is_decoded) {
           config.camera_width = processing_w;
           config.camera_height = processing_h;
      } else {
           config.camera_width = img_width_;
           config.camera_height = img_height_;
      }
      config.pixel_format = is_decoded ? "RGB888" : pixel_format_str_;
      config.model_width = dial.crop_w;
      config.model_height = dial.crop_h;
      config.model_channels = 1; // Grayscale output
      config.input_type = esphome::esp32_camera_utils::kInputTypeUInt8;
      
      // Handle rotation manually if needed, but assuming crop coordinates are pre-rotated or image is upright
      // If image is already valid, we just crop.
      // NOTE: Original CameraCoordinator handled rotation config. 
      // AnalogReader assumes crops are relative to the *delivered* image.
      
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

      // Apply enhancements
      if (dial.auto_contrast) {
          apply_auto_contrast(raw, len);
      }
      if (std::abs(dial.contrast - 1.0f) > 0.01f) {
          apply_contrast(raw, len, dial.contrast);
      }
      
      // Use actual crop dimensions
      float angle = find_needle_angle(raw, dial.crop_w, dial.crop_h, dial);
      
      // Debug angle calculation (detailed logging)
      if (this->debug_) {
          debug_angle_calculation(angle, dial);
      }
      
      float val = angle_to_value(angle, dial);
      
      // Convert to North-based angle for logging clarity 
      float display_angle = angle + 90.0f; 
      if (display_angle >= 360.0f) display_angle -= 360.0f;
      
      ESP_LOGD(TAG, "Dial %s: Angle=%.1f (North), Val=%.2f", dial.id.c_str(), display_angle, val);
      
      // Aggregate
      total_value += val * dial.scale;
      
      char val_buf[16];
      snprintf(val_buf, sizeof(val_buf), "%.1f", val);
      debug_str += (debug_str.empty() ? "" : ", ") + std::string(val_buf);
  }
  
  // Validation (Bypassed for Analog Reader float support)
  // int validated_int_val = (int)total_value;
  // bool valid = validation_coord_.validate_reading((int)total_value, 1.0f, validated_int_val);

  // Truncate to configured decimal precision
  total_value = truncf(total_value * kDecimalPrecision) / kDecimalPrecision;

  ESP_LOGI(TAG, "Result: (Raw: %.4f) [%s]", total_value, debug_str.c_str());
  if (value_sensor_) {
      value_sensor_->publish_state(total_value);
  }


  processing_frame_ = false;
}

float AnalogReader::find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    DetectionResult selected_result;
    
    // Legacy algorithm: Original radial edge detection (NO preprocessing)
    if (dial.algorithm == "legacy" || dial.algorithm == "radial_profile") {
        selected_result = detect_legacy(img, w, h, dial);  // Use RAW image, no preprocessing
    } 
    else {
        // New algorithms: Use preprocessing
        auto processed = preprocess_image(img, w, h, cx, cy, radius);
        
        if (dial.algorithm == "hough_transform") {
            selected_result = detect_hough_transform(processed.data(), w, h, dial);
        } else if (dial.algorithm == "template_match") {
            selected_result = detect_template_match(processed.data(), w, h, dial);
        } else if (dial.algorithm == "auto") {
            // Auto: run all 4, pick highest confidence
            auto result_legacy = detect_legacy(img, w, h, dial);  // No preprocessing for legacy
            auto result_radial = detect_radial_profile(processed.data(), w, h, dial);
            auto result_hough = detect_hough_transform(processed.data(), w, h, dial);
            auto result_template = detect_template_match(processed.data(), w, h, dial);
            
            selected_result = result_legacy;
            if (result_radial.confidence > selected_result.confidence) selected_result = result_radial;
            if (result_hough.confidence > selected_result.confidence) selected_result = result_hough;
            if (result_template.confidence > selected_result.confidence) selected_result = result_template;
            
            if (debug_) {
                ESP_LOGD(TAG, "%s AUTO mode comparison:", dial.id.c_str());
                ESP_LOGD(TAG, "  Legacy: angle=%.1f°, conf=%.2f", result_legacy.angle, result_legacy.confidence);
                ESP_LOGD(TAG, "  Radial Profile: angle=%.1f°, conf=%.2f", result_radial.angle, result_radial.confidence);
                ESP_LOGD(TAG, "  Hough Transform: angle=%.1f°, conf=%.2f", result_hough.angle, result_hough.confidence);
                ESP_LOGD(TAG, "  Template Match: angle=%.1f°, conf=%.2f", result_template.angle, result_template.confidence);
                ESP_LOGD(TAG, "  Selected: %s", selected_result.algorithm.c_str());
            }
        } else {
            // Unknown algorithm, default to legacy
            selected_result = detect_legacy(img, w, h, dial);
        }
    }
    
    if (debug_) {
        ESP_LOGD(TAG, "%s using %s: angle=%.1f°, confidence=%.2f", 
                 dial.id.c_str(), selected_result.algorithm.c_str(), selected_result.angle, selected_result.confidence);
        
        // ASCII visualization - use raw image for legacy, processed for others
        if (dial.algorithm == "legacy" || dial.algorithm == "radial_profile") {
            debug_dial_image(img, w, h, selected_result.angle);
        } else {
            auto processed = preprocess_image(img, w, h, cx, cy, radius);
            debug_dial_image(processed.data(), w, h, selected_result.angle);
        }
    }
    
    return selected_result.angle;
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
    } else {
        // Normal case
        float fraction = (effective_dial_angle - dial.min_angle) / (dial.max_angle - dial.min_angle);
        fraction = std::max(0.0f, std::min(1.0f, fraction));
        return dial.min_value + fraction * (dial.max_value - dial.min_value);
    }
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
// Debug visualization
void AnalogReader::debug_dial_image(const uint8_t* img, int w, int h, float detected_angle) {
    // Create ASCII visualization of the dial with needle
    // Create ASCII visualization of the dial with needle
    const int grid_w = 40;
    const int grid_h = 40;
    
    // We can't stack allocate variable size 2D arrays easily in C++98/standard, use flat vector or fixed max size
    // Using simple vector of strings to buffer line
    
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
        // The 0.9f factor keeps the needle slightly inside the grid boundaries.
        
        int draw_x = (int)( (0.5f + 0.5f * cos(rad) * 0.9f) * grid_w );
        int draw_y = (int)( (0.5f + 0.5f * sin(rad) * 0.9f) * grid_h );
        
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
    const char* COLOR_YELLOW = "\033[93m";   // Yellow for North indicator
    
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

}  // namespace analog_reader
}  // namespace esphome
