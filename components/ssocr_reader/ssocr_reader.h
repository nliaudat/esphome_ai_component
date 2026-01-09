#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_coordinator.h"
#include "flashlight_coordinator.h"
#include "value_validator_coordinator.h"
#include <vector>

namespace esphome {
namespace ssocr_reader {

class SSOCRReader : public PollingComponent, public esphome::camera::CameraListener {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { value_sensor_ = s; }
  void set_confidence_sensor(sensor::Sensor *s) { confidence_sensor_ = s; } 
  void set_validator(value_validator::ValueValidator *v) { validation_coord_.set_validator(v); }
  
  void set_threshold_config(int level) { threshold_level_ = level; }
  void set_crop_config(int x, int y, int w, int h) {
      crop_x_ = x; crop_y_ = y; crop_w_ = w; crop_h_ = h;
  }
  void set_digit_config(int count) { digit_count_ = count; }
  void set_camera(esphome::camera::Camera *camera) { camera_ = camera; }
  void set_resolution(int w, int h) { img_width_ = w; img_height_ = h; }
  void set_pixel_format_str(const std::string &fmt);

 protected:
  // Coordinators
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;

  esphome::camera::Camera *camera_{nullptr};
  bool capture_next_{false};
  int img_width_{0}; 
  int img_height_{0}; 
  // pixformat_t pixel_format_{PIXFORMAT_JPEG}; // Managed by camera_coord_
  std::string pixel_format_str_{"JPEG"};
  sensor::Sensor *value_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr}; // Added for validator/consistency
  int threshold_level_{128};
  int crop_x_{0};
  int crop_y_{0};
  int crop_w_{0};
  int crop_h_{0};
  int digit_count_{6};
  
  // State
  bool processing_frame_{false};
  uint32_t last_request_time_{0};
  bool frame_requested_{false};

  void process_image(std::shared_ptr<esphome::camera::CameraImage> image);
  int recognize_digit(const std::vector<uint8_t> &binary_image, int width, int height);
};

}  // namespace ssocr_reader
}  // namespace esphome
