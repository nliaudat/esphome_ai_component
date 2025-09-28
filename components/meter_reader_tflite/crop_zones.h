#pragma once

#include "esphome/core/component.h"
#include "esphome/components/globals/globals_component.h"
#include <vector>
#include <string>

namespace esphome {
namespace meter_reader_tflite {

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
  
  // Simple method to set the global string
  void set_global_zones_string(const std::string &zones_str) {
    global_zones_string_ = zones_str;
  }
  
  // Check and apply global variable if available
  void apply_global_zones();
  
  // callback support for dynamic updates
  void set_on_zones_changed_callback(std::function<void()> callback) {
    on_zones_changed_ = callback;
  }
  
  // Method to update zones and notify
  void update_zones(const std::string &zones_json) {
    parse_zones(zones_json);
    if (on_zones_changed_) {
      on_zones_changed_();
    }
  }

 protected:
  std::vector<CropZone> zones_;
  std::string global_zones_string_;
  std::function<void()> on_zones_changed_; 
};

}  // namespace meter_reader_tflite
}  // namespace esphome