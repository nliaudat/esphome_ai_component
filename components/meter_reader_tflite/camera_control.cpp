#include "camera_control.h"

namespace esphome {
namespace meter_reader_tflite {
namespace camera_control {

// static const char *const TAG = "CameraWindowControl";
const char *const CameraWindowControl::TAG = "CameraWindowControl";

bool CameraWindowControl::set_window(esp32_camera::ESP32Camera* camera, 
                                    const WindowConfig& config) {
  if (!camera || !config.validate()) {
    ESP_LOGE(TAG, "Invalid camera or window configuration");
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) {
    ESP_LOGE(TAG, "Failed to get camera sensor");
    return false;
  }

  if (!supports_window(camera)) {
    ESP_LOGE(TAG, "Sensor does not support window setting");
    return false;
  }

  std::string sensor_name = get_sensor_name(sensor);
  ESP_LOGI(TAG, "Setting camera window for %s: %s", sensor_name.c_str(), config.to_string().c_str());

  bool success = false;
  
  // Try to detect sensor type and apply appropriate settings
  if (is_sensor_supported(sensor)) {
    // For supported sensors, use the set_res_raw method
    success = set_ov2640_window(sensor, config);
  }

  ESP_LOGI(TAG, "Camera window %s", success ? "set successfully" : "failed");
  return success;
}

bool CameraWindowControl::set_window(esp32_camera::ESP32Camera* camera, 
                                    int offset_x, int offset_y, 
                                    int width, int height) {
  WindowConfig config{offset_x, offset_y, width, height, true};
  return set_window(camera, config);
}

bool CameraWindowControl::set_window_from_crop_zones(esp32_camera::ESP32Camera* camera,
                                                    const std::vector<CropZone>& zones,
                                                    int full_width, int full_height,
                                                    float padding_ratio) {
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
  
  ESP_LOGI(TAG, "Setting camera window from %d crop zones: %s", 
           zones.size(), config.to_string().c_str());
  
  return set_window(camera, config);
}

bool CameraWindowControl::reset_to_full_frame(esp32_camera::ESP32Camera* camera) {
  if (!camera) return false;
  
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  
  std::string sensor_name = get_sensor_name(sensor);
  ESP_LOGI(TAG, "Resetting %s to full frame", sensor_name.c_str());
  
  // Reset to UXGA resolution (or whatever your base resolution is)
  int ret = sensor->set_framesize(sensor, FRAMESIZE_UXGA);
  
  bool success = (ret == 0);
  if (success) {
    ESP_LOGI(TAG, "Camera reset to full frame successfully");
  } else {
    ESP_LOGE(TAG, "Failed to reset camera to full frame");
  }
  
  return success;
}

std::string CameraWindowControl::get_sensor_name(sensor_t* sensor) const {
  if (!sensor) return "Unknown";
  
  // Use the esp32-camera function to get sensor info
  camera_sensor_info_t* sensor_info = esp_camera_sensor_get_info(&sensor->id);
  if (sensor_info && sensor_info->name) {
    return std::string(sensor_info->name);
  }
  
  // Fallback: try to detect from PID if the function fails
  if (sensor->id.PID != 0) {
    switch (sensor->id.PID) {
      case 0x26: return "OV2640";
      case 0x36: 
      case 0x3660: return "OV3660";
      case 0x56:
      case 0x5640: return "OV5640";
      case 0x77:
      case 0x7725: return "OV7725";
      case 0x76:
      case 0x7670: return "OV7670";
      case 0x1410: return "NT99141";
      case 0x2145: return "GC2145";
      case 0x232a: return "GC032A";
      case 0x9b: return "GC0308";
      case 0x30: return "BF3005";
      case 0x20a6: return "BF20A6";
      case 0xda4a: return "SC101IOT";
      case 0x9a46: return "SC030IOT";
      case 0x0031: return "SC031GS";
      case 0x039E: return "MEGA_CCM";
      case 0x0955: return "HM1055";
      case 0x0360: return "HM0360";
      default: 
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Unknown(0x%04X)", sensor->id.PID);
        return std::string(buffer);
    }
  }
  
  return "Unknown Sensor";
}

bool CameraWindowControl::is_sensor_supported(sensor_t* sensor) const {
  if (!sensor) return false;
  
  // Use esp32-camera function first
  camera_sensor_info_t* sensor_info = esp_camera_sensor_get_info(&sensor->id);
  if (sensor_info && sensor_info->name) {
    std::string sensor_name = sensor_info->name;
    for (const auto& supported_sensor : get_supported_sensors()) {
      if (sensor_name.find(supported_sensor) != std::string::npos) {
        return true;
      }
    }
  }
  
  // Fallback to manual detection
  std::string sensor_name = get_sensor_name(sensor);
  for (const auto& supported_sensor : get_supported_sensors()) {
    if (sensor_name.find(supported_sensor) != std::string::npos) {
      return true;
    }
  }
  
  return false;
}

bool CameraWindowControl::set_ov2640_window(sensor_t* sensor, const WindowConfig& config) {
  if (!sensor) return false;
  
  int ret = 0;
  
  // Corrected set_res_raw call based on ESP32-Camera implementation
  // Parameters: sensor, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning
  ret |= sensor->set_res_raw(sensor, 
                            0, 0,                    // startX, startY
                            config.width, config.height, // endX, endY
                            0, 0,                    // offsetX, offsetY
                            config.width, config.height, // totalX, totalY
                            config.width, config.height, // outputX, outputY
                            false, false);           // scale, binning
  
  // Alternative: Try to set framesize to match the window
  // Determine appropriate framesize based on window dimensions
  framesize_t target_size = FRAMESIZE_UXGA;
  if (config.width <= 800 && config.height <= 600) target_size = FRAMESIZE_SVGA;
  if (config.width <= 640 && config.height <= 480) target_size = FRAMESIZE_VGA;
  if (config.width <= 400 && config.height <= 296) target_size = FRAMESIZE_CIF;
  if (config.width <= 320 && config.height <= 240) target_size = FRAMESIZE_QVGA;
  
  ret |= sensor->set_framesize(sensor, target_size);
  
  ESP_LOGD(TAG, "OV2640 window setting result: %d (size: %dx%d)", ret, config.width, config.height);
  return (ret == 0);
}

bool CameraWindowControl::set_ov3660_window(sensor_t* sensor, const WindowConfig& config) {
  // OV3660 uses similar method as OV2640
  return set_ov2640_window(sensor, config);
}

bool CameraWindowControl::set_ov5640_window(sensor_t* sensor, const WindowConfig& config) {
  // OV5640 uses similar method as OV2640
  return set_ov2640_window(sensor, config);
}

bool CameraWindowControl::supports_window(esp32_camera::ESP32Camera* camera) const {
  if (!camera) return false;
  
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  
  // Check if sensor supports set_res_raw function
  if (!sensor->set_res_raw) {
    ESP_LOGD(TAG, "Sensor doesn't support set_res_raw function");
    return false;
  }
  
  return is_sensor_supported(sensor);
}

std::string CameraWindowControl::get_sensor_info(esp32_camera::ESP32Camera* camera) const {
  if (!camera) return "No camera";
  
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return "No sensor";
  
  std::string sensor_name = get_sensor_name(sensor);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%s (PID: 0x%04X)", sensor_name.c_str(), sensor->id.PID);
  return std::string(buffer);
}

const std::vector<std::string>& CameraWindowControl::get_supported_sensors() {
  static const std::vector<std::string> supported_sensors = {
    "OV2640", "OV3660", "OV5640", "OV7725", 
    "SC101IOT", "SC030IOT", "SC031GS"
    // Add other sensors that support window settings
  };
  return supported_sensors;
}

CameraWindowControl::WindowConfig CameraWindowControl::calculate_window_from_zones(
    const std::vector<CropZone>& zones, 
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
  
  for (const auto& zone : zones) {
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
  
  ESP_LOGD(TAG, "Calculated window from %d zones: %s", 
           zones.size(), config.to_string().c_str());
  
  return config;
}

std::pair<int, int> CameraWindowControl::update_dimensions_after_window(
    esp32_camera::ESP32Camera* camera,
    const WindowConfig& config,
    int original_width, int original_height) const {
  
  if (config.enabled && config.validate()) {
    // Return window dimensions
    return {config.width, config.height};
  } else {
    // Return original dimensions
    return {original_width, original_height};
  }
}

}  // namespace camera_control
}  // namespace meter_reader_tflite
}  // namespace esphome