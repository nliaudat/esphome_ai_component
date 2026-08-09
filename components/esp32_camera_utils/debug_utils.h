#pragma once

#include "esphome/core/hal.h"  // For millis()
#include "esphome/core/log.h"

// E9: RAII ScopedDuration, active when the component's debug option is enabled
// (DEBUG_ESP32_CAMERA_UTILS). Zero-cost (no millis() call) otherwise.
#ifdef DEBUG_ESP32_CAMERA_UTILS
namespace esphome {
namespace esp32_camera_utils {

class ScopedDuration {
 public:
  explicit ScopedDuration(const char *tag) : tag_(tag), start_(esphome::millis()) {}

  uint32_t elapsed() const { return esphome::millis() - this->start_; }

  void log_duration(const char *func) { ESP_LOGD(this->tag_, "%s duration: %lums", func, this->elapsed()); }

  void log(const char *msg, uint32_t val) { ESP_LOGD(this->tag_, "%s: %lums", msg, val); }

 private:
  const char *tag_;
  uint32_t start_;
};

}  // namespace esp32_camera_utils
}  // namespace esphome
#else
// Zero-cost no-op when DEBUG_ESP32_CAMERA_UTILS is not defined.
//
// Note: the _dur variable in crop_zone_handler.cpp is still instantiated, but
// this trivial type has no members, no constructor work, and no destructor --
// the compiler eliminates it entirely at -Os.
namespace esphome {
namespace esp32_camera_utils {
struct ScopedDuration {
  explicit ScopedDuration(const char *) {}
};
}  // namespace esp32_camera_utils
}  // namespace esphome
#endif  // DEBUG_ESP32_CAMERA_UTILS
