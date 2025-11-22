#include "esp32_camera_utils.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "esp32_camera_utils";

void Esp32CameraUtils::setup() {
  if (has_config_) {
    ESP_LOGI(TAG, "Setting up camera window: offset=%d,%d size=%dx%d", offset_x_, offset_y_, width_, height_);
    if (camera_) {
      // We need to wait for camera to be initialized? 
      // Component setup order handles dependencies if configured correctly.
      // But camera setup might be async or take time.
      // We'll try to set it.
      bool success = window_control_.set_window_with_reset(
          camera_, CameraWindowControl::WindowConfig{
              offset_x_, offset_y_, width_, height_, true});
              
      if (success) {
        ESP_LOGI(TAG, "Camera window configured successfully");
      } else {
        ESP_LOGE(TAG, "Failed to configure camera window");
      }
    } else {
      ESP_LOGW(TAG, "Camera instance not available, skipping window configuration");
    }
  }
}

void Esp32CameraUtils::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32 Camera Utils:");
  if (has_config_) {
    ESP_LOGCONFIG(TAG, "  Camera Window: offset=%d,%d size=%dx%d", offset_x_, offset_y_, width_, height_);
  }
}

}  // namespace esp32_camera_utils
}  // namespace esphome
