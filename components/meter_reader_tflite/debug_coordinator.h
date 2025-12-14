#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "tflite_coordinator.h"
#include <vector>
#include <memory>

#ifdef DEBUG_METER_READER_TFLITE
// Only define full class if debug enabled, or just keep it simple/empty
#endif

namespace esphome {
namespace meter_reader_tflite {

class DebugCoordinator {
 public:
  void set_debug_image(const uint8_t* data, size_t size, int width, int height);
  
  // Logic
  void run_debug_tests(TFLiteCoordinator& tflite_coord);
  void test_with_pattern(TFLiteCoordinator& tflite_coord);
  
  void print_info(const TFLiteCoordinator& tflite_coord, int cam_w, int cam_h, const std::string& fmt);

  bool is_debug_mode() const { return debug_mode_; }
  void set_debug_mode(bool en) { debug_mode_ = en; }

 private:
  bool debug_mode_{false};
  std::shared_ptr<camera::CameraImage> debug_image_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome
