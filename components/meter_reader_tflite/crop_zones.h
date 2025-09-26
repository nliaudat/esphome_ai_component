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

 protected:
  std::vector<CropZone> zones_;
  std::string global_zones_string_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome