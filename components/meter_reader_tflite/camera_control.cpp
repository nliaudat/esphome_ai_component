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

  // Add a small delay to ensure camera is ready
  vTaskDelay(50 / portTICK_PERIOD_MS);

  bool success = set_sensor_window(sensor, config);

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

bool CameraWindowControl::set_sensor_window(sensor_t* sensor, const WindowConfig& config) {
  if (!sensor) return false;
  
  int ret = 0;
  uint16_t sensor_pid = sensor->id.PID;
  
  ESP_LOGI(TAG, "Setting window for sensor PID: 0x%04X", sensor_pid);
  
  // Get the current output resolution (what we want to maintain)
  int output_width = config.width;
  int output_height = config.height;
  
  // For digital zoom/window, we need to use the full sensor resolution as base
  int full_sensor_width, full_sensor_height;
  
  // Determine full sensor resolution based on sensor type
  switch (sensor_pid) {
    case 0x5640: // OV5640
      full_sensor_width = 2592;
      full_sensor_height = 1944;
      break;
    case 0x3660: // OV3660
      full_sensor_width = 2048;
      full_sensor_height = 1536;
      break;
    case 0x26:   // OV2640
    default:
      full_sensor_width = 1600;
      full_sensor_height = 1200;
      break;
  }
  
  ESP_LOGI(TAG, "Full sensor resolution: %dx%d", full_sensor_width, full_sensor_height);
  ESP_LOGI(TAG, "Target window: offset(%d,%d), size(%dx%d), output(%dx%d)", 
           config.offset_x, config.offset_y, config.width, config.height, output_width, output_height);
  
  // Calculate the actual window parameters
  int window_width = config.width;
  int window_height = config.height;
  int window_offset_x = config.offset_x;
  int window_offset_y = config.offset_y;
  
  // Ensure window doesn't exceed sensor bounds
  window_offset_x = std::max(0, std::min(window_offset_x, full_sensor_width - window_width));
  window_offset_y = std::max(0, std::min(window_offset_y, full_sensor_height - window_height));
  
  // For OV2640, we need to ensure dimensions are properly aligned
  if (sensor_pid == 0x26) {
    // OV2640 requires specific alignment
    window_width = (window_width / 4) * 4;
    window_height = (window_height / 4) * 4;
    window_offset_x = (window_offset_x / 4) * 4;
    window_offset_y = (window_offset_y / 4) * 4;
  }
  
  ESP_LOGI(TAG, "Adjusted window: offset(%d,%d), size(%dx%d)", 
           window_offset_x, window_offset_y, window_width, window_height);
  
  // First, set the output framesize
  framesize_t target_size = get_framesize_from_dimensions(output_width, output_height);
  ret |= sensor->set_framesize(sensor, target_size);
  vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay between commands
  
  // Then set the window using set_res_raw
  if (sensor_pid == 0x26) { // OV2640
    // OV2640: set_res_raw(s, 0, 0, 0, 0, xOffset, yOffset, xTotal, yTotal, xOutput, yOutput, false, false)
    ret |= sensor->set_res_raw(sensor, 
                              0, 0,                    // startX, startY
                              0, 0,                    // endX, endY
                              window_offset_x, window_offset_y, // offsetX, offsetY
                              window_width, window_height, // totalX, totalY
                              output_width, output_height, // outputX, outputY
                              false, false);           // scale, binning
  } 
  else { // OV3660 or OV5640
    // OV3660/OV5640: set_res_raw(s, xOffset, yOffset, xOffset + xTotal, yOffset + yTotal, 0, 0, frameSizeX, frameSizeY, xOutput, yOutput, scale, binning)
    bool scale = !(output_width == window_width && output_height == window_height);
    bool binning = (window_width >= (full_sensor_width >> 1));
    
    ret |= sensor->set_res_raw(sensor, 
                              window_offset_x, window_offset_y, // startX, startY
                              window_offset_x + window_width, window_offset_y + window_height, // endX, endY
                              0, 0,                    // offsetX, offsetY
                              full_sensor_width, full_sensor_height, // totalX, totalY
                              output_width, output_height, // outputX, outputY
                              scale, binning);         // scale, binning
  }
  
  vTaskDelay(50 / portTICK_PERIOD_MS); // Allow time for sensor to apply changes
  
  ESP_LOGI(TAG, "Window setting result: %d (offset: %d,%d, window: %dx%d, output: %dx%d)", 
           ret, window_offset_x, window_offset_y, window_width, window_height, output_width, output_height);
  return (ret == 0);
}

framesize_t CameraWindowControl::get_framesize_from_dimensions(int width, int height) {
  if (width <= 96 && height <= 96) return FRAMESIZE_96X96;
  if (width <= 160 && height <= 120) return FRAMESIZE_QQVGA;
  if (width <= 176 && height <= 144) return FRAMESIZE_QCIF;
  if (width <= 240 && height <= 176) return FRAMESIZE_HQVGA;
  if (width <= 240 && height <= 240) return FRAMESIZE_240X240;
  if (width <= 320 && height <= 240) return FRAMESIZE_QVGA;
  if (width <= 320 && height <= 320) return FRAMESIZE_320X320;
  if (width <= 400 && height <= 296) return FRAMESIZE_CIF;
  if (width <= 480 && height <= 320) return FRAMESIZE_HVGA;
  if (width <= 640 && height <= 480) return FRAMESIZE_VGA;
  if (width <= 800 && height <= 600) return FRAMESIZE_SVGA;
  if (width <= 1024 && height <= 768) return FRAMESIZE_XGA;
  if (width <= 1280 && height <= 720) return FRAMESIZE_HD;
  if (width <= 1280 && height <= 1024) return FRAMESIZE_SXGA;
  if (width <= 1600 && height <= 1200) return FRAMESIZE_UXGA;
  if (width <= 1920 && height <= 1080) return FRAMESIZE_FHD;
  
  return FRAMESIZE_UXGA; // default fallback
}

bool CameraWindowControl::set_ov2640_window(sensor_t* sensor, const WindowConfig& config) {
  return set_sensor_window(sensor, config);
}

bool CameraWindowControl::set_ov3660_window(sensor_t* sensor, const WindowConfig& config) {
  return set_sensor_window(sensor, config);
}

bool CameraWindowControl::set_ov5640_window(sensor_t* sensor, const WindowConfig& config) {
  return set_sensor_window(sensor, config);
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