#pragma once

#include "esphome/core/component.h"
#include "esphome/components/globals/globals_component.h"
#include <vector>
#include <string>
#include <functional>

namespace esphome {
namespace esp32_camera_utils {

struct CropZone {
  int x1;
  int y1;
  int x2;
  int y2;
};

class CropZoneHandler {
 public:
  void parse_zones(const std::string &zones_json);
  const std::vector<CropZone>& get_zones() const { return zones_; }
  void set_default_zone(int width, int height);
  void set_debug_zones();
  
  // Global variable management
  void set_crop_zones_global(globals::GlobalsComponent<std::string> *global_var) {
      crop_zones_global_ = global_var;
      if (crop_zones_global_) {
          last_global_value_ = crop_zones_global_->value();
      }
  }
  
  globals::GlobalsComponent<std::string>* get_crop_zones_global() const {
      return crop_zones_global_;
  }
  
  // Update global variable with new zones
  void update_global_zones(const std::string &zones_json) {
      if (crop_zones_global_) {
          crop_zones_global_->value() = zones_json;
          last_global_value_ = zones_json;
          ESP_LOGI(TAG, "Updated global crop zones variable");
      }
  }
  
  // Check if global variable has changed
  bool has_global_zones_changed() {
      if (!crop_zones_global_) return false;
      
      std::string current_global = crop_zones_global_->value();
      if (current_global != last_global_value_) {
          last_global_value_ = current_global;
          return true;
      }
      return false;
  }
  
  // Apply zones from global variable
  void apply_global_zones() {
      if (crop_zones_global_) {
          std::string zones_value = crop_zones_global_->value();
          if (!zones_value.empty() && zones_value != "[]") {
              parse_zones(zones_value);
              ESP_LOGI(TAG, "Applied crop zones from global variable: %s", zones_value.c_str());
          } else {
              ESP_LOGD(TAG, "Global crop zones variable is empty");
          }
      }
  }
  
  // Method to update zones from any source (text sensor, API, etc.)
  void update_zones(const std::string &zones_json) {
    parse_zones(zones_json);
    
    // Also update the global variable to keep them in sync
    update_global_zones(zones_json);
    
    if (on_zones_changed_) {
      on_zones_changed_();
    }
  }

  // Callback support for dynamic updates
  void set_on_zones_changed_callback(std::function<void()> callback) {
    on_zones_changed_ = callback;
  }

 private:
  static const char *const TAG;
  globals::GlobalsComponent<std::string> *crop_zones_global_{nullptr};
  std::string last_global_value_;

 protected:
  std::vector<CropZone> zones_;
  std::function<void()> on_zones_changed_; 
};

}  // namespace esp32_camera_utils
}  // namespace esphome
