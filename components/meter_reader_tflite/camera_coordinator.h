#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/esp32_camera_utils/camera_window_control.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"

#include <memory>
#include <vector>

#include <string>

namespace esphome {
namespace meter_reader_tflite {

class CameraCoordinator {
 public:
  void set_camera(esp32_camera::ESP32Camera* camera);
  void set_config(int width, int height, const std::string& pixel_format);
  void set_rotation(float rot) { rotation_ = rot; }
  
  void unload();

  // Image Processor
  void update_image_processor_config(int model_width, int model_height, int model_channels, 
                                     int input_type, bool normalize, const std::string& input_order);

  using ProcessResult = esphome::esp32_camera_utils::ImageProcessor::ProcessResult;
  std::vector<ProcessResult> process_frame(
      std::shared_ptr<camera::CameraImage> frame,
      const std::vector<esp32_camera_utils::CropZone>& zones); 


  // Window control
  bool set_window(int offset_x, int offset_y, int width, int height); 
  bool reset_window();
  bool apply_window();
  bool supports_window() const;
  
  // State
  int get_width() const { return current_width_; }
  int get_height() const { return current_height_; }
  const std::string& get_format() const { return current_format_; }
  bool is_window_configured() const { return window_configured_; }
  
  // Configuration setters (partial update support)
  void set_window_config(int x, int y, int w, int h) {
      if (x != -1) window_offset_x_ = x;
      if (y != -1) window_offset_y_ = y;
      if (w != -1) window_width_ = w;
      if (h != -1) window_height_ = h;
      window_configured_ = true;
  }
  
  // Helpers
  bool test_camera_after_reset(std::atomic<bool>& frame_available, std::atomic<bool>& frame_requested);
  void basic_recovery();
  
 private:
  esp32_camera::ESP32Camera* camera_{nullptr};
  esp32_camera_utils::CameraWindowControl window_control_;
  std::unique_ptr<esp32_camera_utils::ImageProcessor> image_processor_;

  float rotation_{0.0f};

  int current_width_{0};
  int current_height_{0};
  std::string current_format_;
  
  // Original specs
  int orig_width_{0};
  int orig_height_{0};
  std::string orig_format_;
  
  // Pending window config
  int window_offset_x_{0};
  int window_offset_y_{0};
  int window_width_{0};
  int window_height_{0};
  bool window_configured_{false};
};

}  // namespace meter_reader_tflite
}  // namespace esphome
