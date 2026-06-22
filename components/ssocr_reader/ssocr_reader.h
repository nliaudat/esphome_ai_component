#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_coordinator.h"
#include "flashlight_coordinator.h"
#include "value_validator_coordinator.h"
#include <vector>
#include <mutex>
#include <atomic>

#include "esphome/core/defines.h"

#ifdef USE_SSOCR_READER

namespace esphome {
namespace ssocr_reader {

class SSOCRReader : public PollingComponent, public esphome::camera::CameraListener {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { this->value_sensor_ = s; }
  void set_confidence_sensor(sensor::Sensor *s) { this->confidence_sensor_ = s; } 
#ifdef USE_VALUE_VALIDATOR
  void set_validator(value_validator::ValueValidator *v) { this->validation_coord_.set_validator(v); }
#endif
  
  void set_threshold_config(int level) { this->threshold_level_ = level; }
  void set_crop_config(int x, int y, int w, int h) {
      this->crop_x_ = x; this->crop_y_ = y; this->crop_w_ = w; this->crop_h_ = h;
  }
  void set_digit_config(int count) { this->digit_count_ = count; }
  void set_camera(esphome::camera::Camera *camera) { this->camera_ = camera; }
  void set_resolution(int w, int h) { this->img_width_ = w; this->img_height_ = h; }
  void set_pixel_format_str(const std::string &fmt) { this->pixel_format_str_ = fmt; }
  void set_debug(bool debug) { this->debug_ = debug; }

 protected:
  // Single atomic state machine — eliminates TOCTOU CWE-367
  enum class FrameState : uint8_t {
    IDLE,
    REQUESTED,
    AVAILABLE,
    PROCESSING,
    TIMEOUT
  };
  std::atomic<FrameState> frame_state_{FrameState::IDLE};
  uint32_t last_request_time_{0};

  // Coordinators
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;

  esphome::camera::Camera *camera_{nullptr};
  int img_width_{0}; 
  int img_height_{0}; 
  std::string pixel_format_str_{"JPEG"};
  sensor::Sensor *value_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  int threshold_level_{128};
  int crop_x_{0};
  int crop_y_{0};
  int crop_w_{0};
  int crop_h_{0};
  int digit_count_{6};
  
  bool debug_{false};
  std::mutex frame_mutex_;
  
  // Async processing (non-blocking camera callback)
  std::shared_ptr<esphome::camera::CameraImage> pending_frame_{nullptr};

  // Optimization: Pre-allocated buffers
  std::vector<int> col_sums_;
  std::vector<std::pair<int, int>> digit_bounds_;

  void process_image(std::shared_ptr<esphome::camera::CameraImage> image);
  int recognize_digit(const uint8_t* img, int width, int height, int stride);
};

}  // namespace ssocr_reader
}  // namespace esphome

#endif  // USE_SSOCR_READER