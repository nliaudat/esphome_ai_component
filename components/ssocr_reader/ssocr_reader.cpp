#include "ssocr_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cmath>
#include <memory> 

namespace esphome {
namespace ssocr_reader {

static const char *const TAG = "ssocr_reader";

void SSOCRReader::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SSOCR Reader...");

  // 1. Setup Validation
  // External component handles this


  // 2. Setup Camera Coordinator
  if (this->camera_) {
      this->camera_coord_.set_camera(static_cast<esp32_camera::ESP32Camera*>(this->camera_));
      
      // We assume img_width_ and img_height_ were set via set_resolution
      std::string fmt_str = "JPEG"; // Default
      // Map enum to string for coordinator: Use "JPEG" default or whatever global config set.
      // camera_coord_ takes a string representation which matches the internal camera config.
  }
  
  if (this->img_width_ > 0 && this->img_height_ > 0) {
       this->camera_coord_.set_config(this->img_width_, this->img_height_, this->pixel_format_str_);
  }

  // 3. Setup Flashlight (Default: None)
  this->flashlight_coord_.setup(this, nullptr, nullptr);

  // 4. Register Listener
  if (this->camera_) {
      this->camera_->add_listener(this);
  }
  
  // 5. Configure Image Processor
  int effective_crop_w = (this->crop_w_ > 0) ? this->crop_w_ : this->img_width_ - this->crop_x_;
  int effective_crop_h = (this->crop_h_ > 0) ? this->crop_h_ : this->img_height_ - this->crop_y_;
  
  // Input type: 1 = kImageProcessorInputTypeUint8 (Grayscale/RGB888).
  // We explicitly request Uint8 for SSOCR operations.
  
  this->camera_coord_.update_image_processor_config(
      effective_crop_w, effective_crop_h, 1, 
      1, // Input Type: Uint8
      true, // Normalize
      "NHWC"
  );
}

void SSOCRReader::dump_config() {
  ESP_LOGCONFIG(TAG, "SSOCR Reader:");
  ESP_LOGCONFIG(TAG, "  Threshold Level: %d", this->threshold_level_);
  ESP_LOGCONFIG(TAG, "  Crop: x=%d, y=%d, w=%d, h=%d", this->crop_x_, this->crop_y_, this->crop_w_, this->crop_h_);
  ESP_LOGCONFIG(TAG, "  Digit Count: %d", this->digit_count_);
}

void SSOCRReader::update() {
  // Flash scheduling
  if (this->flashlight_coord_.update_scheduling()) {
      return;
  }

  // Request frame if ready
  if (!this->frame_requested_ && !this->processing_frame_) {
      this->frame_requested_ = true;
      this->last_request_time_ = millis();
      ESP_LOGD(TAG, "Requesting frame");
  }
}

void SSOCRReader::on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) {
    std::lock_guard<std::mutex> lock(this->frame_mutex_);
    if (this->frame_requested_ && !this->processing_frame_) {
        // Simple lock logic
        this->capture_next_ = true; // Flag for loop
        this->process_image(image);
        this->frame_requested_ = false;
        this->capture_next_ = false;
    }
}

void SSOCRReader::loop() {
    // Timeout check
    if (this->frame_requested_ && millis() - this->last_request_time_ > 5000) {
        ESP_LOGW(TAG, "Frame timeout");
        this->frame_requested_ = false;
    }
}

void SSOCRReader::set_pixel_format_str(const std::string &fmt) {
    this->pixel_format_str_ = fmt;
}

void SSOCRReader::process_image(std::shared_ptr<esphome::camera::CameraImage> image) {
  if (this->camera_ == nullptr) return;
  
  this->processing_frame_ = true;
  
  // 1. Prepare Zones
  std::vector<esphome::esp32_camera_utils::CropZone> zones;
  int w = this->img_width_; 
  int h = this->img_height_;
  
  // Default crop
  int cx = this->crop_x_;
  int cy = this->crop_y_;
  int cw = (this->crop_w_ > 0) ? this->crop_w_ : w - cx;
  int ch = (this->crop_h_ > 0) ? this->crop_h_ : h - cy;
  
  zones.push_back({cx, cy, cx+cw, cy+ch}); // CropZone struct: x1, y1, x2, y2 
  
  // 2. Process via Coordinator
  auto results = this->camera_coord_.process_frame(image, zones);
  
  if (results.empty() || !results[0].data) {
      ESP_LOGE(TAG, "Failed to process image via coordinator");
      this->processing_frame_ = false;
      return;
  }
  
  // Access the TrackedBuffer safely
  auto& buffer = *results[0].data;
  uint8_t* raw_roi = buffer.get();
  
  // Check dimensions match
  // ImageProcessor resizes to configured `model_width/height`.
  // We configured it to effective_crop_w/h.
  // So roi size should be cw * ch * 1.
  
  // 4. Binarize
  // The buffer is Grayscale (uint8). 
  // We need to apply threshold.
  // We can do this in-place (on the raw buffer) or separate buffer.
  // We'll modify the raw buffer in-place since we own it (it's a copy/new buffer from IP).
  int roi_w = cw;
  int roi_h = ch;
  
  // Binarize
  for(int i=0; i < roi_w * roi_h; i++) {
      raw_roi[i] = (raw_roi[i] > this->threshold_level_) ? 255 : 0;
  }
  
  // 5. Vertical Projection & Recognition (Reusing existing logic)
  // Logic adapted to use raw_roi pointer
  
  // Sum pixels
  std::vector<int> col_sums(roi_w, 0);
  for (int j = 0; j < roi_w; j++) {
    for (int i = 0; i < roi_h; i++) {
       if (raw_roi[i * roi_w + j] == 255) {
           col_sums[j]++;
       }
    }
  }

  // Find gaps
  std::vector<std::pair<int, int>> digit_bounds;
  bool in_digit = false;
  int start_x = 0;
  for (int j = 0; j < roi_w; j++) {
      bool has_signal = col_sums[j] > (roi_h * 0.05); 
      if (has_signal && !in_digit) {
          in_digit = true;
          start_x = j;
      } else if (!has_signal && in_digit) {
          in_digit = false;
          if (j - start_x > 5) { 
              digit_bounds.push_back({start_x, j});
          }
      }
  }
  if (in_digit) {
      digit_bounds.push_back({start_x, roi_w});
  }

  if (this->debug_) {
      ESP_LOGD(TAG, "Found %d potential digits", static_cast<int>(digit_bounds.size()));
  }

  // Recognize
  std::string result_str = "";
  for (size_t k = 0; k < digit_bounds.size(); k++) {
      if (k >= static_cast<size_t>(this->digit_count_)) break;
      
      int d_x = digit_bounds[k].first;
      int d_w = digit_bounds[k].second - d_x;
      if (d_w < 3) continue;

      // Extract digit sub-roi
      std::vector<uint8_t> digit_roi(d_w * roi_h);
      for(int r=0; r<roi_h; r++) {
          for(int c=0; c<d_w; c++) {
              digit_roi[r*d_w + c] = raw_roi[r*roi_w + (d_x + c)];
          }
      }
      
      int val = this->recognize_digit(digit_roi, d_w, roi_h);
      if (val >= 0) {
          result_str += std::to_string(val);
      } else {
          result_str += "?";
      }
  }

  // Publish
  ESP_LOGI(TAG, "SSOCR Result: %s", result_str.c_str());
  if (this->value_sensor_) {
      if (result_str.find('?') == std::string::npos && !result_str.empty()) {
          // Parse as int first because validator expects int
          // This assumes the result is an integer number.
          // If we need float support, we must update ValueValidator or implement float logic.
          long v_long = strtol(result_str.c_str(), nullptr, 10);
          int v = static_cast<int>(v_long);
          
          // Validate
          int validated_v = v;
          bool valid = this->validation_coord_.validate_reading(v, 1.0f, validated_v); 

          
          if (valid) {
              this->value_sensor_->publish_state(static_cast<float>(validated_v));
              if (this->confidence_sensor_) this->confidence_sensor_->publish_state(100.0f);
          } else {
              ESP_LOGW(TAG, "Result invalid check logs");
          }
      } else {
           ESP_LOGW(TAG, "SSOCR Failed to read all digits");
           this->value_sensor_->publish_state(NAN); 
      }
  }
  
  this->processing_frame_ = false;
}

int SSOCRReader::recognize_digit(const std::vector<uint8_t> &img, int w, int h) {
    // 7-segment sampling points logic
    // We define 7 segments: A(Top), B(TopRight), C(BotRight), D(Bot), E(BotLeft), F(TopLeft), G(Mid)
    // We sample small patches.
    
    // Normalized coordinates (0.0 - 1.0)
    struct Pt { float x; float y; };
    Pt segments[7] = {
        {0.50, 0.15}, // A: Top
        {0.80, 0.30}, // B: Top Right
        {0.80, 0.70}, // C: Bot Right
        {0.50, 0.85}, // D: Bot
        {0.20, 0.70}, // E: Bot Left
        {0.20, 0.30}, // F: Top Left
        {0.50, 0.50}  // G: Middle
    };
    
    // Map bitmask to digit (0-9)
    // A=1, B=2, C=4, D=8, E=16, F=32, G=64
    // 0: A+B+C+D+E+F = 1+2+4+8+16+32 = 63
    // 1: B+C = 2+4 = 6
    // 2: A+B+D+E+G = 1+2+8+16+64 = 91
    // 3: A+B+C+D+G = 1+2+4+8+64 = 79
    // 4: B+C+F+G = 2+4+32+64 = 102
    // 5: A+C+D+F+G = 1+4+8+32+64 = 109
    // 6: A+C+D+E+F+G = 1+4+8+16+32+64 = 125
    // 7: A+B+C = 1+2+4 = 7
    // 8: All = 127
    // 9: A+B+C+F+G + D. (Usually 9 has D).
    // Mapping: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    const uint8_t digit_map[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
    
    // Map for lookup
    // A B C D F G = 1+2+4+8+32+64 = 111.
    
    uint8_t mask = 0;
    
    for (int i=0; i<7; i++) {
        int sx = static_cast<int>(segments[i].x * w);
        int sy = static_cast<int>(segments[i].y * h);
        
        // Sampling: Check a 3x3 window around point to be robust
        int on_pixels = 0;
        int count = 0;
        for (int dy=-1; dy<=1; dy++) {
            for (int dx=-1; dx<=1; dx++) {
                int nx = sx+dx;
                int ny = sy+dy;
                if (nx >=0 && nx < w && ny >=0 && ny < h) {
                    count++;
                    if (img[ny*w + nx] == 255) on_pixels++;
                }
            }
        }
        
        if (static_cast<float>(on_pixels) / static_cast<float>(count) > 0.5f) {
            mask |= (1 << i);
        }
    }
    
    // Exact match lookup
    for (int i=0; i<10; i++) {
        if (digit_map[i] == mask) return i;
    }
    
    if (this->debug_) {
        ESP_LOGD(TAG, "Unknown mask: %d (0x%02X)", mask, mask);
    } else {
        ESP_LOGV(TAG, "Unknown mask: %d", mask);
    }
    return -1;
}

}  // namespace ssocr_reader
}  // namespace esphome
