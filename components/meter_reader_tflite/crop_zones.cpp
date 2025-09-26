#include "crop_zones.h"
#include "esp_log.h"
#include "debug_utils.h"
#include <algorithm>
#include <cstdlib> // for strtol

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "CropZoneHandler";

void CropZoneHandler::apply_global_zones() {
  if (!global_zones_string_.empty()) {
    ESP_LOGI(TAG, "Applying crop zones from global string");
    parse_zones(global_zones_string_);
  } else {
    ESP_LOGD(TAG, "No global crop zones string available");
  }
}

void CropZoneHandler::parse_zones(const std::string &zones_json) {
  ESP_LOGD(TAG, "Parsing crop zones JSON: %s", zones_json.c_str());
  zones_.clear();
  
  // Handle empty or invalid JSON
  if (zones_json.empty() || zones_json == "[]" || zones_json == "\"[]\"") {
    ESP_LOGD(TAG, "No crop zones defined or empty JSON");
    return;
  }

  std::string stripped = zones_json;
  
  // Remove quotes if present (common when coming from globals)
  if (stripped.front() == '"' && stripped.back() == '"') {
    stripped = stripped.substr(1, stripped.length() - 2);
  }
  
  // Remove whitespace
  stripped.erase(std::remove(stripped.begin(), stripped.end(), ' '), stripped.end());
  stripped.erase(std::remove(stripped.begin(), stripped.end(), '\n'), stripped.end());
  stripped.erase(std::remove(stripped.begin(), stripped.end(), '\r'), stripped.end());
  stripped.erase(std::remove(stripped.begin(), stripped.end(), '\t'), stripped.end());

  if (stripped.empty() || stripped == "[]") {
    ESP_LOGD(TAG, "No crop zones defined after cleaning");
    return;
  }

  // Remove outer brackets
  if (stripped.front() == '[' && stripped.back() == ']') {
    stripped = stripped.substr(1, stripped.length() - 2);
  }

  size_t pos = 0;
  while (pos < stripped.length()) {
    size_t start = stripped.find('[', pos);
    if (start == std::string::npos) break;
    size_t end = stripped.find(']', start);
    if (end == std::string::npos) break;

    std::string zone_str = stripped.substr(start + 1, end - start - 1);
    std::vector<int> coords;
    size_t coord_start = 0;
    bool parse_error = false;
    
    while (true) {
      size_t comma_pos = zone_str.find(',', coord_start);
      if (comma_pos == std::string::npos) break;
      
      std::string num_str = zone_str.substr(coord_start, comma_pos - coord_start);
      char *end_ptr;
      long num = strtol(num_str.c_str(), &end_ptr, 10);
      
      if (end_ptr == num_str.c_str() || *end_ptr != '\0') {
        ESP_LOGE(TAG, "Failed to parse coordinate: %s", num_str.c_str());
        parse_error = true;
        break;
      }
      
      coords.push_back(static_cast<int>(num));
      coord_start = comma_pos + 1;
    }

    if (parse_error) {
      pos = end + 1;
      continue;
    }

    // Try to get the last number
    if (coord_start < zone_str.length()) {
      std::string num_str = zone_str.substr(coord_start);
      char *end_ptr;
      long num = strtol(num_str.c_str(), &end_ptr, 10);
      
      if (end_ptr == num_str.c_str() || *end_ptr != '\0') {
        ESP_LOGE(TAG, "Failed to parse coordinate: %s", num_str.c_str());
        pos = end + 1;
        continue;
      }
      
      coords.push_back(static_cast<int>(num));
    }

    if (coords.size() == 4) {
      ESP_LOGD(TAG, "Added zone [%d,%d,%d,%d]", 
               coords[0], coords[1], coords[2], coords[3]);
      zones_.push_back({coords[0], coords[1], coords[2], coords[3]});
    } else {
      ESP_LOGE(TAG, "Invalid zone format (expected 4 coordinates, got %d): %s", 
               coords.size(), zone_str.c_str());
    }

    pos = end + 1;
  }

  ESP_LOGI(TAG, "Parsed %d crop zones", zones_.size());
}

void CropZoneHandler::set_default_zone(int width, int height) {
    zones_.clear();
    zones_.push_back({
        static_cast<int>(width * 0.1f),    // x1 - 10% from left
        static_cast<int>(height * 0.2f),   // y1 - 20% from top
        static_cast<int>(width * 0.9f),    // x2 - 10% from right
        static_cast<int>(height * 0.8f)    // y2 - 20% from bottom
    });
    ESP_LOGI(TAG, "Set default crop zone: [%d,%d,%d,%d]", 
             zones_[0].x1, zones_[0].y1, zones_[0].x2, zones_[0].y2);
}


void CropZoneHandler::set_debug_zones() {
    zones_.clear();
    // Static crop zones for debug image
    zones_ = {
        {80, 233, 116, 307},   // Digit 1
        {144, 235, 180, 307},  // Digit 2
        {202, 234, 238, 308},  // Digit 3
        {265, 233, 304, 306},  // Digit 4
        {328, 232, 367, 311},  // Digit 5
        {393, 231, 433, 310},  // Digit 6
        {460, 235, 499, 311},  // Digit 7
        {520, 235, 559, 342}   // Digit 8
    };
    ESP_LOGI(TAG, "Set debug crop zones (8 zones)");
}


}  // namespace meter_reader_tflite
}  // namespace esphome
