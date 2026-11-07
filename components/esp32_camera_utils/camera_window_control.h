#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32_CAMERA_UTILS

#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esp_camera.h"
#include "esphome/core/log.h"
#include "crop_zone_handler.h"

#include <string>
#include <utility>
#include <vector>

namespace esphome {
namespace esp32_camera_utils {

/**
 * @brief Thin adapter over the native esp32_camera window API.
 *
 * Window coordinates are in the pixels of the esp32_camera `resolution`, not
 * the full sensor. The native API clamps them to the frame and applies the
 * per-sensor readout rules, so a window must fit inside `resolution`; see
 * window_fits_frame().
 */
class CameraWindowControl {
 public:
  struct WindowConfig {
    int offset_x{0};
    int offset_y{0};
    int width{0};
    int height{0};
    bool enabled{false};

    bool validate() const { return enabled && offset_x >= 0 && offset_y >= 0 && width > 0 && height > 0; }

    std::string to_string() const {
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "WindowConfig{enabled:%s, offset(%d,%d), size(%dx%d)}",
               enabled ? "true" : "false", offset_x, offset_y, width, height);
      return std::string(buffer);
    }
  };

  /**
   * @brief Set camera window for digital zoom/region of interest
   */
  bool set_window(esp32_camera::ESP32Camera *camera, const WindowConfig &config);

  /**
   * @brief Set camera window with individual parameters
   */
  bool set_window(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y, int width, int height);

  /**
   * @brief Set camera window automatically from crop zones
   */
  bool set_window_from_crop_zones(esp32_camera::ESP32Camera *camera, const std::vector<CropZone> &zones, int full_width,
                                  int full_height, float padding_ratio = 0.05f);

  /**
   * @brief Reset camera to full frame
   */
  bool reset_to_full_frame(esp32_camera::ESP32Camera *camera);

  /**
   * @brief Test window functionality with a simple centered window
   * @return true if window setting works successfully
   */
  bool test_window_stability(esp32_camera::ESP32Camera *camera);

  bool set_ROI(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y, int width, int height);

  /**
   * @brief Get current sensor information
   */
  std::string get_sensor_info(esp32_camera::ESP32Camera *camera) const;

  /**
   * @brief Get supported sensors for window setting
   */
  static const std::vector<std::string> &get_supported_sensors();

  /**
   * @brief Calculate optimal window from crop zones
   */
  static WindowConfig calculate_window_from_zones(const std::vector<CropZone> &zones, int full_width, int full_height,
                                                  float padding_ratio = 0.05f);

  /**
   * @brief Update camera dimensions after window change
   * @return New dimensions as pair<width, height>
   */
  std::pair<int, int> update_dimensions_after_window(esp32_camera::ESP32Camera *camera, const WindowConfig &config,
                                                     int original_width, int original_height) const;

  /**
   * @brief Set camera window and update dimensions
   */
  bool set_window_with_dimensions(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y, int width, int height,
                                  int &current_width, int &current_height);

  /**
   * @brief Set camera window from crop zones and update dimensions
   */
  bool set_window_from_crop_zones_with_dimensions(esp32_camera::ESP32Camera *camera, const std::vector<CropZone> &zones,
                                                  int &current_width, int &current_height);

  /**
   * @brief Reset camera to full frame and restore original dimensions
   */
  bool reset_to_full_frame_with_dimensions(esp32_camera::ESP32Camera *camera, int original_width, int original_height,
                                           int &current_width, int &current_height);

  /**
   * @brief Check if camera supports window operations
   */
  bool supports_window(esp32_camera::ESP32Camera *camera) const;

  // Helper to get current dimensions after window operations
  std::pair<int, int> get_current_dimensions(esp32_camera::ESP32Camera *camera, const WindowConfig &config,
                                             int original_width, int original_height) const;

  bool hard_reset_camera(esp32_camera::ESP32Camera *camera);
  bool soft_reset_camera(esp32_camera::ESP32Camera *camera);
  bool set_window_with_reset(esp32_camera::ESP32Camera *camera, const WindowConfig &config);
  bool reset_to_full_frame_with_reset(esp32_camera::ESP32Camera *camera);

  /**
   * @brief True when the published frame is the window itself.
   *
   * The OV2640 publishes the (JPEG-aligned) window size; the other supported
   * sensors publish the configured resolution and only crop/scale into it.
   */
  bool window_defines_frame(esp32_camera::ESP32Camera *camera) const;

  /**
   * @brief The pixels a window is measured in: the esp32_camera `resolution`.
   *
   * Falls back to the given values while the sensor is not initialised yet.
   */
  std::pair<int, int> frame_size(int fallback_width, int fallback_height) const;

  /**
   * @brief True when the window lies inside a frame of the given size.
   *
   * The camera clamps a larger window, which silently changes the crop.
   */
  static bool window_fits_frame(const WindowConfig &config, int frame_width, int frame_height);

  // Sensor detection and specific implementations
  std::string get_sensor_name(sensor_t *sensor) const;
  bool is_sensor_supported(sensor_t *sensor) const;

 private:
  static const char *const TAG;

  /// Dimensions of the configured frame, falling back to the given values.
  static std::pair<int, int> frame_dimensions(int fallback_width, int fallback_height);
};

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif  // USE_ESP32_CAMERA_UTILS
