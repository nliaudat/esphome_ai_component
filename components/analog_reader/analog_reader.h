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
namespace analog_reader {

struct DialConfig {
  std::string id;
  float scale{1.0f};
  int crop_x{0};
  int crop_y{0};
  int crop_w{0};
  int crop_h{0};
  float min_angle{0.0f};   // Degrees
  float max_angle{360.0f}; // Degrees
  float angle_offset{0.0f}; // 0 = North, 90 = East
  float min_value{0.0f};
  float max_value{10.0f};
  bool auto_contrast{true}; // Normalization (Min-Max Stretch)
  float contrast{1.0f};      // Multiplier (1.0 = original)
  // Future: needle_color
};

class AnalogReader : public PollingComponent, public esphome::camera::CameraListener {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { value_sensor_ = s; }
  void set_validator(value_validator::ValueValidator *v) { validation_coord_.set_validator(v); }
  
  void set_camera(esphome::camera::Camera *camera) { camera_ = camera; }
  void set_camera_image_format(int width, int height, const std::string &format) {
      img_width_ = width;
      img_height_ = height;
      pixel_format_str_ = format;
  }

  void set_pause_processing(bool paused) { paused_ = paused; }
  
  void set_all_auto_contrast(bool enabled) {
      for (auto &dial : dials_) dial.auto_contrast = enabled;
  }
  
  void set_all_contrast(float contrast) {
      for (auto &dial : dials_) dial.contrast = contrast;
  }
  
  void set_update_interval(uint32_t interval) override;

  void add_dial(DialConfig config) { dials_.push_back(config); }

 protected:
  bool paused_{false};
  // Coordinators
  // Note: We reuse CameraCoordinator from ssocr_reader/meter_reader context
  // Assuming it's available in include path (it is in same 'components' root usually or library)
  // But wait, ssocr_reader includes "camera_coordinator.h". We need to ensure we can link it.
  // We will add it to __init__.py libraries.
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;

  esphome::camera::Camera *camera_{nullptr};
  bool capture_next_{false};
  int img_width_{0}; 
  int img_height_{0}; 
  std::string pixel_format_str_{"JPEG"};
  
  sensor::Sensor *value_sensor_{nullptr};
  std::vector<DialConfig> dials_;
  
  // State
  bool processing_frame_{false};
  uint32_t last_request_time_{0};
  bool frame_requested_{false};

  void process_image(std::shared_ptr<esphome::camera::CameraImage> image);
  float find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial);
  
  // Helpers
  float angle_to_value(float angle, const DialConfig& dial);
};

}  // namespace analog_reader
}  // namespace esphome
