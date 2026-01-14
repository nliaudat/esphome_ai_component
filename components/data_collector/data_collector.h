#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace data_collector {

class DataCollector : public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_upload_url(const std::string &url) { upload_url_ = url; }
  void set_web_submit_switch(switch_::Switch *s) { web_submit_switch_ = s; }
  void set_auth(const std::string &user, const std::string &password) { username_ = user; password_ = password; }
  void set_api_key(const std::string &key) { api_key_ = key; }
  void set_debug(bool debug) { debug_ = debug; }

  // Main entry point
  // raw_value and confidence are passed for metadata/logging
  void collect_image(std::shared_ptr<camera::CameraImage> frame, float raw_value, float confidence);

 protected:
  std::string upload_url_;
  std::string username_;
  std::string password_;
  std::string api_key_;
  bool debug_{false};
  switch_::Switch *web_submit_switch_{nullptr};

  // Helper to upload
  bool upload_image(const uint8_t *data, size_t len, float raw_value, float confidence);
};

}  // namespace data_collector
}  // namespace esphome
