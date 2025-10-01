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
   * @brief Test window functionality with a simple centered window
   * @return true if window setting works successfully
   */
  bool test_window_stability(esp32_camera::ESP32Camera* camera);

  bool set_ROI(esp32_camera::ESP32Camera* camera, 
                int offset_x, int offset_y, 
                int width, int height);

                
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
      
    /**
     * @brief Set camera window and update dimensions
     */
    bool set_window_with_dimensions(esp32_camera::ESP32Camera* camera,
                                   int offset_x, int offset_y, 
                                   int width, int height,
                                   int& current_width, int& current_height);

    /**
     * @brief Set camera window from crop zones and update dimensions
     */
    bool set_window_from_crop_zones_with_dimensions(esp32_camera::ESP32Camera* camera,
                                                   const std::vector<CropZone>& zones,
                                                   int& current_width, int& current_height);

    /**
     * @brief Reset camera to full frame and restore original dimensions
     */
    bool reset_to_full_frame_with_dimensions(esp32_camera::ESP32Camera* camera,
                                            int original_width, int original_height,
                                            int& current_width, int& current_height);

    /**
     * @brief Check if camera supports window operations
     */
    bool supports_window(esp32_camera::ESP32Camera* camera) const;

    // Helper to get current dimensions after window operations
    std::pair<int, int> get_current_dimensions(esp32_camera::ESP32Camera* camera,
                                              const WindowConfig& config,
                                              int original_width, int original_height) const;

    
    bool hard_reset_camera(esp32_camera::ESP32Camera* camera);
    bool soft_reset_camera(esp32_camera::ESP32Camera* camera);
    bool set_window_with_reset(esp32_camera::ESP32Camera* camera, const WindowConfig& config);
    bool reset_to_full_frame_with_reset(esp32_camera::ESP32Camera* camera);

 private:
  static const char *const TAG;
  
  // Sensor detection and specific implementations
  std::string get_sensor_name(sensor_t* sensor) const;
  bool is_sensor_supported(sensor_t* sensor) const;
  
  // Main window setting function
  bool set_sensor_window(sensor_t* sensor, const WindowConfig& config);
  
  // Sensor-specific implementations
  bool set_ov2640_window(sensor_t* sensor, const WindowConfig& config);
  bool set_ov3660_window(sensor_t* sensor, const WindowConfig& config);
  bool set_ov5640_window(sensor_t* sensor, const WindowConfig& config);
  
  // Helper function to get framesize from dimensions
  framesize_t get_framesize_from_dimensions(int width, int height);
  
  static std::string framesize_to_string(framesize_t framesize);
  framesize_t get_max_framesize(sensor_t* sensor) const;
};

}  // namespace camera_control
}  // namespace meter_reader_tflite
}  // namespace esphome