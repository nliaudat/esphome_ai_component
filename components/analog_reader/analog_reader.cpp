#include "analog_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <memory>
#include <algorithm>

namespace esphome {
namespace analog_reader {

static const char *const TAG = "analog_reader";

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
    }
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

void AnalogReader::process_image(std::shared_ptr<esphome::camera::CameraImage> image) {
  if (this->camera_ == nullptr || dials_.empty()) return;
  
  processing_frame_ = true;
  
  float total_value = 0.0f;
  std::string debug_str = "";
  
  for (const auto& dial : dials_) {
      // Reconfigure ImageProcessor for this dial's dimensions
      camera_coord_.update_image_processor_config(
          dial.crop_w, dial.crop_h, 
          1, // 1 Channel (Grayscale)
          1, // Uint8
          false, // No normalize
          "NHWC"
      );

      std::vector<esphome::esp32_camera_utils::CropZone> zones;
      zones.push_back({dial.crop_x, dial.crop_y, dial.crop_w, dial.crop_h});
      
      // Process via Coordinator
      auto results = camera_coord_.process_frame(image, zones);
      
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
      
      ESP_LOGD(TAG, "Dial %s: Angle=%.1f, Val=%.2f", dial.id.c_str(), angle, val);
      
      // Aggregate
      total_value += val * dial.scale;
      debug_str += (debug_str.empty() ? "" : ", ") + std::to_string(val);
  }
  
  // Validation
  // Validate the TOTAL value against the previous TOTAL value
  int validated_int_val = (int)total_value;
  // Using 1.0 confidence for basic analog reading until we implement confidence metric
  bool valid = validation_coord_.validate_reading((int)total_value, 1.0f, validated_int_val);

  float final_val = (float)validated_int_val;

  if (valid) {
      ESP_LOGI(TAG, "Result: VALID (Raw: %.4f, Validated: %.4f) [%s]", total_value, final_val, debug_str.c_str());
      if (value_sensor_) {
          value_sensor_->publish_state(final_val);
      }
  } else {
      ESP_LOGW(TAG, "Result: INVALID (Raw: %.4f) [%s]", total_value, debug_str.c_str());
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
