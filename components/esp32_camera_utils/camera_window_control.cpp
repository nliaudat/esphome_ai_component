#include "camera_window_control.h"

#include <algorithm>

namespace esphome {
namespace esp32_camera_utils {

const char *const CameraWindowControl::TAG = "CameraWindowControl";

namespace {
// JPEG compresses whole 16x8 blocks, so the OV2640 publishes a window rounded to them.
constexpr int OV2640_JPEG_BLOCK_WIDTH = 16;
constexpr int OV2640_JPEG_BLOCK_HEIGHT = 8;

int round_down_at_least(int value, int step) { return std::max(value - value % step, step); }
}  // namespace

bool CameraWindowControl::set_window(esp32_camera::ESP32Camera *camera, const WindowConfig &config) {
  if (!camera || !config.validate()) {
    ESP_LOGE(TAG, "Invalid camera or window configuration");
    return false;
  }

  if (!supports_window(camera)) {
    ESP_LOGE(TAG, "Sensor does not support window setting");
    return false;
  }

  ESP_LOGI(TAG, "Setting camera window: %s", config.to_string().c_str());

#ifdef USE_ESP32_CAMERA_WINDOW
  bool success = camera->set_window(config.offset_x, config.offset_y, config.width, config.height);
#else
  bool success = false;
#endif

  ESP_LOGI(TAG, "Camera window %s", success ? "set successfully" : "failed");
  return success;
}

bool CameraWindowControl::set_window(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y, int width,
                                     int height) {
  WindowConfig config{offset_x, offset_y, width, height, true};
  return set_window(camera, config);
}

bool CameraWindowControl::set_ROI(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y, int width,
                                  int height) {
  WindowConfig config{offset_x, offset_y, width, height, true};
  return set_window(camera, config);
}

bool CameraWindowControl::set_window_from_crop_zones(esp32_camera::ESP32Camera *camera,
                                                     const std::vector<CropZone> &zones, int full_width,
                                                     int full_height, float padding_ratio) {
  if (!camera || !supports_window(camera)) {
    ESP_LOGE(TAG, "Camera doesn't support window setting");
    return false;
  }

  if (zones.empty()) {
    ESP_LOGI(TAG, "No crop zones configured for camera window");
    return false;
  }

  WindowConfig config = calculate_window_from_zones(zones, full_width, full_height, padding_ratio);

  if (!config.validate()) {
    ESP_LOGE(TAG, "Failed to calculate valid window from crop zones");
    return false;
  }

  ESP_LOGI(TAG, "Setting camera window from %d crop zones: %s", static_cast<int>(zones.size()),
           config.to_string().c_str());

  return set_window(camera, config);
}

bool CameraWindowControl::reset_to_full_frame(esp32_camera::ESP32Camera *camera) {
  if (!camera)
    return false;

  ESP_LOGI(TAG, "Resetting camera to full frame");

#ifdef USE_ESP32_CAMERA_WINDOW
  bool success = camera->reset_window();
#else
  bool success = false;
#endif

  if (success) {
    ESP_LOGI(TAG, "Camera reset to full frame successfully");
  } else {
    ESP_LOGE(TAG, "Failed to reset camera to full frame");
  }

  return success;
}

std::string CameraWindowControl::get_sensor_name(sensor_t *sensor) const {
  if (!sensor)
    return "Unknown";

  // Use the esp32-camera function to get sensor info
  camera_sensor_info_t *sensor_info = esp_camera_sensor_get_info(&sensor->id);
  if (sensor_info && sensor_info->name) {
    return std::string(sensor_info->name);
  }

  // Fallback: try to detect from PID if the function fails
  if (sensor->id.PID != 0) {
    switch (sensor->id.PID) {
      case 0x26:
        return "OV2640";
      case 0x36:
      case 0x3660:
        return "OV3660";
      case 0x56:
      case 0x5640:
        return "OV5640";
      case 0x77:
      case 0x7725:
        return "OV7725";
      case 0x76:
      case 0x7670:
        return "OV7670";
      case 0x1410:
        return "NT99141";
      case 0x2145:
        return "GC2145";
      case 0x232a:
        return "GC032A";
      case 0x9b:
        return "GC0308";
      case 0x30:
        return "BF3005";
      case 0x20a6:
        return "BF20A6";
      case 0xda4a:
        return "SC101IOT";
      case 0x9a46:
        return "SC030IOT";
      case 0x0031:
        return "SC031GS";
      case 0x039E:
        return "MEGA_CCM";
      case 0x0955:
        return "HM1055";
      case 0x0360:
        return "HM0360";
      default:
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Unknown(0x%04X)", sensor->id.PID);
        return std::string(buffer);
    }
  }

  return "Unknown Sensor";
}

bool CameraWindowControl::is_sensor_supported(sensor_t *sensor) const {
  if (!sensor)
    return false;

  switch (sensor->id.PID) {
    case OV2640_PID:
    case OV3660_PID:
    case OV5640_PID:
    case SC101IOT_PID:
    case SC030IOT_PID:
      return true;
    default:
      return false;
  }
}

bool CameraWindowControl::test_window_stability(esp32_camera::ESP32Camera *camera) {
  if (!camera)
    return false;

  ESP_LOGI(TAG, "Testing window stability...");

  const bool supported = supports_window(camera);
  if (supported) {
    ESP_LOGI(TAG, "Window stability test PASSED - sensor supports windowing");
  } else {
    ESP_LOGE(TAG, "Window stability test FAILED - sensor does not support windowing");
  }
  return supported;
}

std::string CameraWindowControl::get_sensor_info(esp32_camera::ESP32Camera *camera) const {
  if (!camera)
    return "No camera";

  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor)
    return "No sensor";

  std::string sensor_name = get_sensor_name(sensor);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%s (PID: 0x%04X)", sensor_name.c_str(), sensor->id.PID);
  return std::string(buffer);
}

const std::vector<std::string> &CameraWindowControl::get_supported_sensors() {
  static const std::vector<std::string> supported_sensors = {"OV2640", "OV3660", "OV5640", "SC101IOT", "SC030IOT"};
  return supported_sensors;
}

CameraWindowControl::WindowConfig CameraWindowControl::calculate_window_from_zones(const std::vector<CropZone> &zones,
                                                                                   int full_width, int full_height,
                                                                                   float padding_ratio) {
  WindowConfig config;

  if (zones.empty()) {
    config.enabled = false;
    return config;
  }

  // Calculate bounding box for all crop zones
  int min_x = full_width;
  int min_y = full_height;
  int max_x = 0;
  int max_y = 0;

  for (const auto &zone : zones) {
    min_x = std::min(min_x, zone.x1);
    min_y = std::min(min_y, zone.y1);
    max_x = std::max(max_x, zone.x2);
    max_y = std::max(max_y, zone.y2);
  }

  // Add padding
  int padding_x = full_width * padding_ratio;
  int padding_y = full_height * padding_ratio;

  config.offset_x = std::max(0, min_x - padding_x);
  config.offset_y = std::max(0, min_y - padding_y);
  config.width = std::min(full_width - config.offset_x, (max_x - min_x) + 2 * padding_x);
  config.height = std::min(full_height - config.offset_y, (max_y - min_y) + 2 * padding_y);
  config.enabled = true;

  // Ensure dimensions are valid
  if (config.width <= 0 || config.height <= 0) {
    config.enabled = false;
  }

  // Ensure dimensions are multiples of 4 (some sensors require this)
  config.width = (config.width / 4) * 4;
  config.height = (config.height / 4) * 4;

  ESP_LOGD(TAG, "Calculated window from %d zones: %s", static_cast<int>(zones.size()), config.to_string().c_str());

  return config;
}

std::pair<int, int> CameraWindowControl::frame_dimensions(int fallback_width, int fallback_height) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr && sensor->status.framesize < FRAMESIZE_INVALID) {
    const resolution_info_t &frame = resolution[sensor->status.framesize];
    if (frame.width > 0 && frame.height > 0)
      return {frame.width, frame.height};
  }
  return {fallback_width, fallback_height};
}

bool CameraWindowControl::window_defines_frame(esp32_camera::ESP32Camera *camera) const {
  if (!camera)
    return false;

  sensor_t *sensor = esp_camera_sensor_get();
  // The OV2640 publishes the (JPEG-aligned) window size; the other sensors
  // publish the configured resolution and crop/scale into it.
  return sensor != nullptr && sensor->id.PID == OV2640_PID && sensor->pixformat == PIXFORMAT_JPEG;
}

std::pair<int, int> CameraWindowControl::update_dimensions_after_window(esp32_camera::ESP32Camera *camera,
                                                                        const WindowConfig &config, int original_width,
                                                                        int original_height) const {
  const std::pair<int, int> frame = frame_dimensions(original_width, original_height);

  if (!config.enabled || !config.validate())
    return frame;

  if (this->window_defines_frame(camera)) {
    // Match what the native OV2640 window path publishes: clamp to the frame,
    // then round down to whole JPEG blocks (but keep at least one block).
    int width = round_down_at_least(std::min(config.width, frame.first), OV2640_JPEG_BLOCK_WIDTH);
    int height = round_down_at_least(std::min(config.height, frame.second), OV2640_JPEG_BLOCK_HEIGHT);
    return {width, height};
  }

  return frame;
}

bool CameraWindowControl::set_window_with_dimensions(esp32_camera::ESP32Camera *camera, int offset_x, int offset_y,
                                                     int width, int height, int &current_width, int &current_height) {
  bool success = set_ROI(camera, offset_x, offset_y, width, height);

  if (success) {
    auto new_dims = update_dimensions_after_window(camera, WindowConfig{offset_x, offset_y, width, height, true},
                                                   current_width, current_height);

    current_width = new_dims.first;
    current_height = new_dims.second;
  }

  return success;
}

bool CameraWindowControl::set_window_from_crop_zones_with_dimensions(esp32_camera::ESP32Camera *camera,
                                                                     const std::vector<CropZone> &zones,
                                                                     int &current_width, int &current_height) {
  const std::pair<int, int> frame = frame_dimensions(current_width, current_height);

  bool success = set_window_from_crop_zones(camera, zones, frame.first, frame.second);

  if (success) {
    auto config = calculate_window_from_zones(zones, frame.first, frame.second);
    auto new_dims = update_dimensions_after_window(camera, config, frame.first, frame.second);

    current_width = new_dims.first;
    current_height = new_dims.second;
  }

  return success;
}

bool CameraWindowControl::reset_to_full_frame_with_dimensions(esp32_camera::ESP32Camera *camera, int original_width,
                                                              int original_height, int &current_width,
                                                              int &current_height) {
  bool success = reset_to_full_frame(camera);

  if (success) {
    const std::pair<int, int> frame = frame_dimensions(original_width, original_height);
    current_width = frame.first;
    current_height = frame.second;
  }

  return success;
}

bool CameraWindowControl::supports_window(esp32_camera::ESP32Camera *camera) const {
  if (!camera)
    return false;

#ifdef USE_ESP32_CAMERA_WINDOW
  return camera->supports_window();
#else
  sensor_t *sensor = esp_camera_sensor_get();
  return sensor != nullptr && is_sensor_supported(sensor);
#endif
}

std::pair<int, int> CameraWindowControl::get_current_dimensions(esp32_camera::ESP32Camera *camera,
                                                                const WindowConfig &config, int original_width,
                                                                int original_height) const {
  return update_dimensions_after_window(camera, config, original_width, original_height);
}

bool CameraWindowControl::hard_reset_camera(esp32_camera::ESP32Camera *camera) {
  if (!camera)
    return false;

  ESP_LOGI(TAG, "Performing camera window reset...");
  return reset_to_full_frame(camera);
}

bool CameraWindowControl::soft_reset_camera(esp32_camera::ESP32Camera *camera) {
  if (!camera)
    return false;

  ESP_LOGI(TAG, "Performing soft camera reset...");
  return reset_to_full_frame(camera);
}

bool CameraWindowControl::set_window_with_reset(esp32_camera::ESP32Camera *camera, const WindowConfig &config) {
  if (!camera || !config.validate()) {
    ESP_LOGE(TAG, "Invalid camera or window configuration");
    return false;
  }

  // The native API restores the full readout before applying a new window, so
  // no extra reset is needed here.
  return set_window(camera, config);
}

bool CameraWindowControl::reset_to_full_frame_with_reset(esp32_camera::ESP32Camera *camera) {
  if (!camera)
    return false;

  return reset_to_full_frame(camera);
}

}  // namespace esp32_camera_utils
}  // namespace esphome
