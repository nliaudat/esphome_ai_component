#pragma once

#include "esphome/core/hal.h"  // For millis()
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_camera_utils {

// RAII ScopedDuration -- replaces DURATION_START/END/LOG macros
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
