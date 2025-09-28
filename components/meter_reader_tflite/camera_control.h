#pragma once

#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "crop_zones.h"

namespace esphome {
namespace meter_reader_tflite {
namespace camera_control {

class CameraWindowControl {
 public:
  struct WindowConfig {
    int offset_x{0};
    int offset_y{0};
    int width{0};
    int height{0};
    bool enabled{false};
    
    bool validate() const {
      return enabled && offset_x >= 0 && offset_y >= 0 && width > 0 && height > 0;
    }
    
    std::string to_string() const {
      char buffer[128];
      snprintf(buffer, sizeof(buffer), 
               "WindowConfig{enabled:%s, offset(%d,%d), size(%dx%d)}",
               enabled ? "true" : "false", offset_x, offset_y, width, height);
      return std::string(buffer);
    }
  };

  /**
   * @brief Set camera window for digital zoom/region of interest
   */
  bool set_window(esp32_camera::ESP32Camera* camera, const WindowConfig& config);
  
  /**
   * @brief Set camera window with individual parameters
   */
  bool set_window(esp32_camera::ESP32Camera* camera, 
                  int offset_x, int offset_y, int width, int height);
  
  /**
   * @brief Set camera window automatically from crop zones
   */
  bool set_window_from_crop_zones(esp32_camera::ESP32Camera* camera,
                                 const std::vector<CropZone>& zones,
                                 int full_width, int full_height,
                                 float padding_ratio = 0.05f);
  
  /**
   * @brief Reset camera to full frame
   */
  bool reset_to_full_frame(esp32_camera::ESP32Camera* camera);
  
  /**
   * @brief Check if current sensor supports window setting
   */
  bool supports_window(esp32_camera::ESP32Camera* camera) const;
  
  /**
   * @brief Get current sensor information
   */
  std::string get_sensor_info(esp32_camera::ESP32Camera* camera) const;
  
  /**
   * @brief Get supported sensors for window setting
   */
  static const std::vector<std::string>& get_supported_sensors();

  /**
   * @brief Calculate optimal window from crop zones
   */
  static WindowConfig calculate_window_from_zones(
      const std::vector<CropZone>& zones, 
      int full_width, int full_height,
      float padding_ratio = 0.05f);

  /**
   * @brief Update camera dimensions after window change
   * @return New dimensions as pair<width, height>
   */
  std::pair<int, int> update_dimensions_after_window(
      esp32_camera::ESP32Camera* camera,
      const WindowConfig& config,
      int original_width, int original_height) const;

 private:
  static const char *const TAG;
  
  // Sensor detection and specific implementations
  std::string get_sensor_name(sensor_t* sensor) const;
  bool is_sensor_supported(sensor_t* sensor) const;
  bool set_ov2640_window(sensor_t* sensor, const WindowConfig& config);
  bool set_ov3660_window(sensor_t* sensor, const WindowConfig& config);
  bool set_ov5640_window(sensor_t* sensor, const WindowConfig& config);
};

}  // namespace camera_control
}  // namespace meter_reader_tflite
}  // namespace esphome