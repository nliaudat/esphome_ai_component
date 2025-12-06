#pragma once
#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_window_control.h"
#include "image_processor.h"
#include <memory>
#include <string>

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

  bool set_camera_window(int offset_x, int offset_y, int width, int height);

  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }

  // New methods for image processing and camera management
  void set_camera_image_format(int width, int height, const std::string &pixel_format);
  void reinitialize_image_processor(const ImageProcessorConfig& config_template);
  bool test_camera_after_reset();
  void basic_camera_recovery();
  
  // Reset camera window to full frame
  bool reset_window(int &width, int &height);
  
  // Expose image processor for usage
  ImageProcessor* get_image_processor() { return image_processor_.get(); }
  
  // Helper to process a zone directly
  bool process_zone(std::shared_ptr<camera::CameraImage> frame, const CropZone& zone, 
                   uint8_t* output_buffer, size_t output_size);

 protected:
  int offset_x_{0};
  int offset_y_{0};
  int width_{0};
  int height_{0};
  bool has_config_{false};
  esp32_camera::ESP32Camera *camera_{nullptr};
  CameraWindowControl window_control_;
  
  // Image processing
  std::unique_ptr<ImageProcessor> image_processor_;
  int camera_width_{0};
  int camera_height_{0};
  std::string pixel_format_{"RGB888"};
  
  // Original camera configuration storage
  int original_camera_width_{0};
  int original_camera_height_{0};
  std::string original_pixel_format_;
  
  // Stored config template for reinitialization
  ImageProcessorConfig last_config_template_;
  bool has_processor_config_{false};
};

}  // namespace esp32_camera_utils
}  // namespace esphome
