#pragma once
#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_window_control.h"

namespace esphome {
namespace esp32_camera_utils {

class Esp32CameraUtils : public Component {
 public:
  void setup() override;
  void dump_config() override;
  
  void set_camera_window_config(int offset_x, int offset_y, int width, int height) {
    offset_x_ = offset_x;
    offset_y_ = offset_y;
    width_ = width;
    height_ = height;
    has_config_ = true;
  }

  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }

 protected:
  int offset_x_{0};
  int offset_y_{0};
  int width_{0};
  int height_{0};
  bool has_config_{false};
  esp32_camera::ESP32Camera *camera_{nullptr};
  CameraWindowControl window_control_;
};

}  // namespace esp32_camera_utils
}  // namespace esphome
