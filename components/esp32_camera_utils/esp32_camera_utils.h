#pragma once
#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_window_control.h"
#include "image_processor.h"
#include "esphome/components/sensor/sensor.h"
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
  
  // Image rotation configuration (0, 90, 180, 270 degrees clockwise, or arbitrary)
  void set_rotation(float rotation) { rotation_ = rotation; }
  float get_rotation() const { return rotation_; }

  // Modular feature configuration
  void set_scaler_config(int width, int height) {
    scaler_width_ = width;
    scaler_height_ = height;
    has_scaler_config_ = true;
  }
  
  void set_cropper_config(int width, int height, int offset_x = 0, int offset_y = 0) {
    cropper_width_ = width;
    cropper_height_ = height;
    cropper_offset_x_ = offset_x;
    cropper_offset_y_ = offset_y;
    has_cropper_config_ = true;
  }

  // Manually trigger sensor updates
  void update_memory_sensors();

#ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
  void set_camera_buffer_size_sensor(sensor::Sensor *s) { camera_buffer_size_sensor_ = s; }
  void set_camera_free_psram_sensor(sensor::Sensor *s) { camera_free_psram_sensor_ = s; }
#endif


 protected:
  int offset_x_{0};
  int offset_y_{0};
  int width_{0};
  int height_{0};
  bool has_config_{false};
  float rotation_{0.0f};                          ///< Image rotation in degrees
  
  int scaler_width_{0};
  int scaler_height_{0};
  bool has_scaler_config_{false};
  
  int cropper_width_{0};
  int cropper_height_{0};
  int cropper_offset_x_{0};
  int cropper_offset_y_{0};
  bool has_cropper_config_{false};
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

#ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
  sensor::Sensor *camera_buffer_size_sensor_{nullptr};
  sensor::Sensor *camera_free_psram_sensor_{nullptr};
#endif

};

}  // namespace esp32_camera_utils
}  // namespace esphome
