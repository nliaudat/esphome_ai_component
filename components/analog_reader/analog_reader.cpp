#include "analog_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <memory>
#include <algorithm>

namespace esphome {
namespace analog_reader {

static const char *const TAG = "analog_reader";

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
  DecodedImage(esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr &&data, size_t len, int width, int height)
      : data_(std::move(data)), len_(len), width_(width), height_(height) {}

  uint8_t *get_data_buffer() override { return data_.get(); }
  size_t get_data_length() override { return len_; }
  bool was_requested_by(camera::CameraRequester requester) const override { return true; }

  ~DecodedImage() {
      // Trace log for memory debugging
      ESP_LOGV("analog_reader", "Destroying DecodedImage (Buffer released)");
  }

 private:
  esphome::esp32_camera_utils::ImageProcessor::JpegBufferPtr data_;
  size_t len_;
  int width_;
  int height_;
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
          // 3 bytes per pixel
          processing_image = std::make_shared<DecodedImage>(std::move(decoded_buf), w * h * 3, w, h);
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
      float val = angle_to_value(angle, dial);
      
      // Convert to North-based angle for logging clarity (User expectation)
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

  // Round to 4 decimal places as requested
  total_value = roundf(total_value * 10000.0f) / 10000.0f;

  ESP_LOGI(TAG, "Result: (Raw: %.4f) [%s]", total_value, debug_str.c_str());
  if (value_sensor_) {
      value_sensor_->publish_state(total_value);
  }


  processing_frame_ = false;
}

float AnalogReader::find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial) {
    // Basic implementation: Radial Sum (Dark Needle)
    // Center
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    float best_angle = 0;
    float min_sum = 1e9; // We look for DARK needle (min sum)
    
    // Scan range
    // 0 degrees = East (Right), 90 = South (Down) in image coords? 
    // Standard Math: 0=East, 90=North (Up)?
    // Image Y is Down.
    // So 0=(1,0), 90=(0,1) [Down].
    // Let's scan in steps.
    
    for (int deg = 0; deg < 360; deg += 2) {
        float rad = deg * M_PI / 180.0f;
        float dx = cos(rad);
        float dy = sin(rad); // Y down
        
        float sum = 0;
        int count = 0;
        // Ray cast
        for (int r = 5; r < radius; r++) {
            int px = cx + (int)(r * dx);
            int py = cy + (int)(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                sum += img[py * w + px];
                count++;
            }
        }
        
        if (count > 0) {
             float avg = sum / count;
             if (avg < min_sum) {
                 min_sum = avg;
                 best_angle = (float)deg;
             }
        }
    }
    
    return best_angle;
}

float AnalogReader::angle_to_value(float angle, const DialConfig& dial) {
    // Map angle [min_angle, max_angle] to [min_value, max_value]
    // Input 'angle' is in Image Space: 0=East, 90=South (Clockwise).
    // User Configuration 'angle_offset' is Relative to NORTH (Clockwise).
    // We want to convert Image Angle to Dial Angle.
    
    // 1. Convert Image Angle (0=East) to North-Referenced Angle (0=North)
    // Image 270 (North) -> 0
    // Image 0 (East) -> 90
    // Formula: NorthRef = ImageAng + 90
    float north_ref_angle = angle + 90.0f;
    
    // 2. Shift by User Offset (Where is the Zero mark relative to North?)
    // Dial Angle = NorthRef - Offset
    float dial_angle = north_ref_angle - dial.angle_offset;
    
    // 3. Normalize angle to range relative to min_angle
    float rel_angle = dial_angle - dial.min_angle;
    
    // 4. Normalize to 0-360 positive
    while (rel_angle < 0) rel_angle += 360.0f;
    while (rel_angle >= 360.0f) rel_angle -= 360.0f;
    
    // Determine range
    float range_angle = dial.max_angle - dial.min_angle;
    if (range_angle <= 0) range_angle += 360.0f; // Handle wrap-around definition if needed or assume user error
    
    float range_val = dial.max_value - dial.min_value;

    float fraction = rel_angle / range_angle;
    
    // Hard clamp for sanity if somehow out of bounds
    if (fraction > 1.0f) fraction = 1.0f;
    if (fraction < 0.0f) fraction = 0.0f;
    
    return dial.min_value + (fraction * range_val);
}

}  // namespace analog_reader
}  // namespace esphome
