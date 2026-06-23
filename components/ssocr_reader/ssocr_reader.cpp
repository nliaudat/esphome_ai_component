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

  if (this->camera_) {
      this->camera_coord_.set_camera(static_cast<esp32_camera::ESP32Camera*>(this->camera_));
  }
  
  if (this->img_width_ > 0 && this->img_height_ > 0) {
       this->camera_coord_.set_config(this->img_width_, this->img_height_, this->pixel_format_str_);
  }

  this->flashlight_coord_.setup(this, nullptr, nullptr);

  if (this->camera_) {
      this->camera_->add_listener(this);
  }
  
  int effective_crop_w = (this->crop_w_ > 0) ? this->crop_w_ : this->img_width_ - this->crop_x_;
  int effective_crop_h = (this->crop_h_ > 0) ? this->crop_h_ : this->img_height_ - this->crop_y_;
  
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
  if (this->flashlight_coord_.update_scheduling()) {
      return;
  }

  // Request frame via single atomic state machine (eliminates TOCTOU CWE-367)
  FrameState expected = FrameState::IDLE;
  if (this->frame_state_.compare_exchange_strong(expected, FrameState::REQUESTED)) {
      this->last_request_time_ = millis();
      ESP_LOGD(TAG, "Requesting frame");
  }
}

void SSOCRReader::on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) {
    std::lock_guard<std::mutex> lock(this->frame_mutex_);
    FrameState expected = FrameState::REQUESTED;
    if (this->frame_state_.compare_exchange_strong(expected, FrameState::AVAILABLE)) {
        this->pending_frame_ = image;
    }
}

void SSOCRReader::loop() {
    // Process pending frame if available (non-blocking camera callback)
    std::shared_ptr<esphome::camera::CameraImage> frame;
    {
        std::lock_guard<std::mutex> lock(this->frame_mutex_);
        if (this->pending_frame_) {
            frame = this->pending_frame_;
            this->pending_frame_ = nullptr;
        }
    }

    if (frame) {
        this->frame_state_.store(FrameState::PROCESSING);
        this->process_image(frame);
        frame.reset();
        this->frame_state_.store(FrameState::IDLE);
    }

    // Watchdog: If frame requested but not arrived, reset state
    FrameState state = this->frame_state_.load();
    if (state == FrameState::REQUESTED && millis() - this->last_request_time_ > 5000) {
        ESP_LOGW(TAG, "Frame timeout");
        FrameState expected = FrameState::REQUESTED;
        if (this->frame_state_.compare_exchange_strong(expected, FrameState::IDLE)) {
            std::lock_guard<std::mutex> lock(this->frame_mutex_);
            this->pending_frame_ = nullptr;
        }
    }
}

void SSOCRReader::process_image(std::shared_ptr<esphome::camera::CameraImage> image) {
  if (this->camera_ == nullptr) return;
  
  // 1. Prepare Zones
  std::vector<esphome::esp32_camera_utils::CropZone> zones;
  int w = this->img_width_; 
  int h = this->img_height_;
  
  int cx = this->crop_x_;
  int cy = this->crop_y_;
  int cw = (this->crop_w_ > 0) ? this->crop_w_ : w - cx;
  int ch = (this->crop_h_ > 0) ? this->crop_h_ : h - cy;
  
  zones.push_back({cx, cy, cx+cw, cy+ch});
  
  auto results = this->camera_coord_.process_frame(image, zones);
  
  if (results.empty() || !results[0].data) {
      ESP_LOGE(TAG, "Failed to process image via coordinator");
      return;
  }
  
  auto& buffer = *results[0].data;
  uint8_t* raw_roi = buffer.get();
  int roi_w = cw;
  int roi_h = ch;
  
  // Binarize
  for(size_t i=0; i < static_cast<size_t>(roi_w) * static_cast<size_t>(roi_h); i++) {
      raw_roi[i] = (raw_roi[i] > this->threshold_level_) ? 255 : 0;
  }
  
  // Vertical projection
  if (this->col_sums_.size() != static_cast<size_t>(roi_w)) {
      this->col_sums_.resize(roi_w);
  }
  std::fill(this->col_sums_.begin(), this->col_sums_.end(), 0);

  for (int j = 0; j < roi_w; j++) {
    for (int i = 0; i < roi_h; i++) {
       if (raw_roi[i * roi_w + j] == 255) {
           this->col_sums_[j]++;
       }
    }
  }

  // Find digit gaps
  this->digit_bounds_.clear();
  bool in_digit = false;
  int start_x = 0;
  for (int j = 0; j < roi_w; j++) {
      bool has_signal = this->col_sums_[j] > (roi_h * 0.05); 
      if (has_signal && !in_digit) {
          in_digit = true;
          start_x = j;
      } else if (!has_signal && in_digit) {
          in_digit = false;
          if (j - start_x > 5) { 
              this->digit_bounds_.push_back({start_x, j});
          }
      }
  }
  if (in_digit) {
      this->digit_bounds_.push_back({start_x, roi_w});
  }

  if (this->debug_) {
      ESP_LOGD(TAG, "Found %d potential digits", static_cast<int>(this->digit_bounds_.size()));
  }

  // Recognize digits
  std::string result_str;
  result_str.reserve(this->digit_bounds_.size() + 1);

  for (size_t k = 0; k < this->digit_bounds_.size(); k++) {
      if (k >= static_cast<size_t>(this->digit_count_)) break;
      
      int d_x = this->digit_bounds_[k].first;
      int d_w = this->digit_bounds_[k].second - d_x;
      if (d_w < 3) continue;

      int val = this->recognize_digit(raw_roi + d_x, d_w, roi_h, roi_w);
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
          long v_long = strtol(result_str.c_str(), nullptr, 10);
          int v = static_cast<int>(v_long);
          
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
           // Skip publishing on failure — don't publish NaN
      }
  }
}

int SSOCRReader::recognize_digit(const uint8_t* img, int w, int h, int stride) {
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
    
    const uint8_t digit_map[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
    
    uint8_t mask = 0;
    
    for (int i=0; i<7; i++) {
        int sx = static_cast<int>(segments[i].x * w);
        int sy = static_cast<int>(segments[i].y * h);
        
        int on_pixels = 0;
        int count = 0;
        for (int dy=-1; dy<=1; dy++) {
            for (int dx=-1; dx<=1; dx++) {
                int nx = sx+dx;
                int ny = sy+dy;
                if (nx >=0 && nx < w && ny >=0 && ny < h) {
                    count++;
                    if (img[ny*stride + nx] == 255) on_pixels++;
                }
            }
        }
        
        if (static_cast<float>(on_pixels) / static_cast<float>(count) > 0.5f) {
            mask |= (1 << i);
        }
    }
    
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