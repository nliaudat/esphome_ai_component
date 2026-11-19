#pragma once

#include "esphome/core/component.h"
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
  /// Max zones the readers can process (StaticVector<float,16> in meter_reader_tflite).
  static constexpr size_t MAX_CROP_ZONES = 16;

  void parse_zones(const std::string &zones_json);
  const std::vector<CropZone> &get_zones() const { return zones_; }
  void set_default_zone(int width, int height);
  void set_debug_zones();

  // Backing storage for the runtime-editable crop zones. Callers may pass a
  // `globals` string (via its `value()` reference) or any other std::string that
  // outlives this handler.
  void set_crop_zones_global(std::string *global_var);

  std::string *get_crop_zones_global() const { return crop_zones_global_; }

  // Update global variable with new zones
  void update_global_zones(const std::string &zones_json);

  // Check if global variable has changed
  bool has_global_zones_changed();

  // Apply zones from global variable
  void apply_global_zones();

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
  void set_on_zones_changed_callback(std::function<void()> callback) { on_zones_changed_ = callback; }

 private:
  static const char *const TAG;
  std::string *crop_zones_global_{nullptr};
  std::string last_global_value_;

 protected:
  std::vector<CropZone> zones_;
  std::function<void()> on_zones_changed_;
};

}  // namespace esp32_camera_utils
}  // namespace esphome
