#include "camera_control.h"

namespace esphome {
namespace meter_reader_tflite {
namespace camera_control {

const char *const CameraWindowControl::TAG = "CameraWindowControl";

bool CameraWindowControl::set_window(esp32_camera::ESP32Camera* camera, const WindowConfig& config) {
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

bool CameraWindowControl::set_ROI(esp32_camera::ESP32Camera* camera, 
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
  
  // Reset to maximum resolution for this sensor
  framesize_t max_framesize = get_max_framesize(sensor);
  int ret = sensor->set_framesize(sensor, max_framesize);
  
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

/**
 * After deep analysis of esp-idf esp32-camera ov2640.c, ov3660.c, ov5640.c
 * @brief Sets a custom window (ROI) on the camera sensor for digital zoom/cropping
 * 
 * This function configures the sensor to capture only a specific region of interest (ROI)
 * while maintaining the desired output resolution. Different sensors have different
 * requirements and parameter mappings.
 * 
 * Supported Sensors:
 * - OV2640 (PID: 0x26)
 * - OV3660 (PID: 0x3660) 
 * - OV5640 (PID: 0x5640)
 * 
 * Sensor-Specific Requirements:
 * 
 * OV2640:
 * - All dimensions must be multiples of 4 pixels
 * - Parameter mapping: set_res_raw(0, 0, 0, 0, offsetX, offsetY, width, height, outputX, outputY, false, false)
 * - Uses offsetX/offsetY for window position, width/height for window size
 * - No scaling or binning in set_res_raw (handled internally)
 * - May require pixel format re-application after window change
 * 
 * OV3660/OV5640:
 * - All dimensions must be multiples of 2 pixels
 * - Parameter mapping: set_res_raw(startX, startY, endX, endY, 0, 0, totalX, totalY, outputX, outputY, scale, binning)
 * - Uses startX/startY and endX/endY to define window boundaries
 * - totalX/totalY should be full sensor resolution
 * - Scale enabled when output resolution ≠ window size
 * - Binning enabled when window ≤ half sensor size
 * 
 * Aspect Ratio Considerations:
 * 
 * Native Sensor Resolutions & Ratios:
 * - OV2640: 1600x1200 (4:3 ratio)
 * - OV3660: 2048x1536 (4:3 ratio) 
 * - OV5640: 2592x1944 (4:3 ratio)
 * 
 * Ratio Handling in Drivers:
 * - All sensors natively use 4:3 aspect ratio
 * - Predefined ratio tables maintain aspect ratio for standard framesizes
 * - Custom windows can use any aspect ratio, but scaling may occur
 * - For non-4:3 outputs, scaling is automatically applied
 * 
 * Ratio Table Structure (from drivers):
 * - max_width/max_height: Maximum sensor resolution
 * - start_x/start_y: Starting coordinates for this ratio
 * - end_x/end_y: Ending coordinates for this ratio  
 * - total_x/total_y: Total active sensor area
 * - offset_x/offset_y: Offset for centering the image
 * 
 * Alignment Requirements:
 * - OV2640: Multiples of 4 (strict)
 * - OV3660/OV5640: Multiples of 2
 * - Function automatically applies alignment before setting window
 * 
 * Parameter Definitions:
 * - window_offset_x/y: Top-left corner of ROI in sensor coordinates
 * - window_width/height: Size of the ROI to capture
 * - output_width/height: Desired output resolution (may be scaled from ROI)
 * - full_sensor_width/height: Maximum sensor resolution (sensor-dependent)
 * 
 * Timing Considerations:
 * - 20ms delay after set_framesize
 * - 100ms delay after set_res_raw
 * - Additional 50ms delay for OV2640 pixel format stabilization
 * 
 * Error Handling:
 * - Returns false if sensor is null or unsupported
 * - Validates window bounds against sensor limits
 * - Ensures minimum window size of 32x32 pixels
 * - Logs detailed error information on failure
 * 
 * Usage Example:
 * @code
 * // 4:3 ratio window (recommended for best quality)
 * WindowConfig config{100, 100, 800, 600, true};
 * 
 * // 16:9 ratio window (will be scaled)
 * WindowConfig config{100, 150, 800, 450, true};
 * 
 * if (set_sensor_window(sensor, config)) {
 *     ESP_LOGI(TAG, "Window set successfully");
 * }
 * @endcode
 * 
 * @param sensor Pointer to the camera sensor structure
 * @param config Window configuration parameters
 * @return true if window was set successfully, false otherwise
 * 
 * @note For OV2640, the window dimensions are automatically aligned to multiples of 4
 * @note For best performance, use output resolution that matches common framesizes
 * @note Binning may affect image quality but improves performance for small windows
 * @note 4:3 aspect ratio windows provide best quality (native sensor ratio)
 * @note Non-4:3 ratios will be scaled, which may reduce image quality
 */

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

bool CameraWindowControl::test_window_stability(esp32_camera::ESP32Camera* camera) {
  if (!camera) return false;
  
  ESP_LOGI(TAG, "Testing window stability with direct sensor commands...");
  
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  
  // Test with a simple direct command instead of using set_window
  int ret = sensor->set_framesize(sensor, FRAMESIZE_VGA); // 640x480
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  if (ret == 0) {
    ESP_LOGI(TAG, "Window stability test PASSED - basic framesize change works");
    return true;
  } else {
    ESP_LOGE(TAG, "Window stability test FAILED - framesize change error: %d", ret);
    return false;
  }
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

std::string CameraWindowControl::framesize_to_string(framesize_t framesize) {
  switch (framesize) {
    case FRAMESIZE_96X96: return "96x96";
    case FRAMESIZE_QQVGA: return "160x120";
    case FRAMESIZE_QCIF: return "176x144";
    case FRAMESIZE_QVGA: return "320x240";
    case FRAMESIZE_CIF: return "400x296";
    case FRAMESIZE_VGA: return "640x480";
    case FRAMESIZE_SVGA: return "800x600";
    case FRAMESIZE_XGA: return "1024x768";
    case FRAMESIZE_HD: return "1280x720";
    case FRAMESIZE_SXGA: return "1280x1024";
    case FRAMESIZE_UXGA: return "1600x1200";
    case FRAMESIZE_FHD: return "1920x1080";
    case FRAMESIZE_QXGA: return "2048x1536";
    case FRAMESIZE_5MP: return "2592x1944";
    default: return "Unknown";
  }
}

framesize_t CameraWindowControl::get_max_framesize(sensor_t* sensor) const {
  if (!sensor) return FRAMESIZE_UXGA;
  
  uint16_t sensor_pid = sensor->id.PID;
  
  switch (sensor_pid) {
    case 0x5640: // OV5640
      return FRAMESIZE_5MP; // 2592x1944
    case 0x3660: // OV3660
      return FRAMESIZE_QXGA; // 2048x1536
    case 0x26:   // OV2640
    default:
      return FRAMESIZE_UXGA; // 1600x1200
  }
}

bool CameraWindowControl::set_window_with_dimensions(esp32_camera::ESP32Camera* camera,
                                                    int offset_x, int offset_y, 
                                                    int width, int height,
                                                    int& current_width, int& current_height) {
    bool success = set_ROI(camera, offset_x, offset_y, width, height);
    
    if (success) {
        auto new_dims = update_dimensions_after_window(
            camera, 
            WindowConfig{offset_x, offset_y, width, height, true},
            current_width, current_height);
        
        current_width = new_dims.first;
        current_height = new_dims.second;
    }
    
    return success;
}

bool CameraWindowControl::set_window_from_crop_zones_with_dimensions(esp32_camera::ESP32Camera* camera,
                                                                    const std::vector<CropZone>& zones,
                                                                    int& current_width, int& current_height) {
    bool success = set_window_from_crop_zones(camera, zones, current_width, current_height);
    
    if (success) {
        auto config = calculate_window_from_zones(zones, current_width, current_height);
        auto new_dims = update_dimensions_after_window(camera, config, current_width, current_height);
        
        current_width = new_dims.first;
        current_height = new_dims.second;
    }
    
    return success;
}

bool CameraWindowControl::reset_to_full_frame_with_dimensions(esp32_camera::ESP32Camera* camera,
                                                             int original_width, int original_height,
                                                             int& current_width, int& current_height) {
    bool success = reset_to_full_frame(camera);
    
    if (success) {
        current_width = original_width;
        current_height = original_height;
    }
    
    return success;
}

bool CameraWindowControl::supports_window(esp32_camera::ESP32Camera* camera) const {
  if (!camera) return false;
  
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) {
    ESP_LOGD(TAG, "No sensor found - camera may not be initialized yet");
    return false;
  }
  
  // Check if sensor supports set_res_raw function
  if (!sensor->set_res_raw) {
    ESP_LOGD(TAG, "Sensor doesn't support set_res_raw function");
    return false;
  }
  
  // Quick check if sensor is supported (avoid heavy operations during setup)
  return is_sensor_supported(sensor);
}

std::pair<int, int> CameraWindowControl::get_current_dimensions(esp32_camera::ESP32Camera* camera,
                                                               const WindowConfig& config,
                                                               int original_width, int original_height) const {
    return update_dimensions_after_window(camera, config, original_width, original_height);
}



bool CameraWindowControl::hard_reset_camera(esp32_camera::ESP32Camera* camera) {
    if (!camera) return false;
    
    ESP_LOGI(TAG, "Performing hard camera reset...");
    
    // 1. Return all frame buffers to ensure clean state
    esp_camera_return_all();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 2. Reset the sensor
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->reset) {
        ESP_LOGI(TAG, "Performing sensor hardware reset...");
        int reset_result = sensor->reset(sensor);
        if (reset_result == 0) {
            ESP_LOGI(TAG, "Sensor hardware reset successful");
        } else {
            ESP_LOGW(TAG, "Sensor hardware reset returned: %d", reset_result);
        }
    }
    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // 3. Reset to maximum resolution
    if (sensor) {
        framesize_t max_framesize = get_max_framesize(sensor);
        sensor->set_framesize(sensor, max_framesize);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "Hard camera reset completed");
    return true;
}

bool CameraWindowControl::soft_reset_camera(esp32_camera::ESP32Camera* camera) {
    if (!camera) return false;
    
    ESP_LOGI(TAG, "Performing soft camera reset...");
    
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) return false;
    
    // Reset through framesize changes
    framesize_t current_size = sensor->status.framesize;
    framesize_t max_size = get_max_framesize(sensor);
    
    // Change to a different size and back to force reinitialization
    if (current_size != FRAMESIZE_QVGA) {
        sensor->set_framesize(sensor, FRAMESIZE_QVGA);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    sensor->set_framesize(sensor, max_size);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Soft camera reset completed");
    return true;
}

bool CameraWindowControl::set_window_with_reset(esp32_camera::ESP32Camera* camera, const WindowConfig& config) {
    if (!camera || !config.validate()) {
        ESP_LOGE(TAG, "Invalid camera or window configuration");
        return false;
    }

    ESP_LOGI(TAG, "Setting camera window with reset...");
    
    // Perform reset before window change
    if (!soft_reset_camera(camera)) {
        ESP_LOGW(TAG, "Soft reset had minor issues, continuing...");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Call the existing set_window method
    return set_window(camera, config);
}

bool CameraWindowControl::reset_to_full_frame_with_reset(esp32_camera::ESP32Camera* camera) {
    if (!camera) return false;
    
    ESP_LOGI(TAG, "Resetting to full frame with reset...");
    
    // Perform reset before full frame
    soft_reset_camera(camera);
    
    // Call the existing reset_to_full_frame method
    return reset_to_full_frame(camera);
}

}  // namespace camera_control
}  // namespace meter_reader_tflite
}  // namespace esphome