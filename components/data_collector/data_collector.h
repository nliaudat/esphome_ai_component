#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

#include <memory>
#include <string>
#include <cstring>
#include <atomic>

#include "esphome/core/defines.h"

#ifdef USE_DATA_COLLECTOR

namespace esphome {
namespace data_collector {

class DataCollector : public Component {
 public:
  void setup() override;
  void dump_config() override;
  ~DataCollector();

  void set_upload_url(const std::string &url) { this->upload_url_ = url; }
  void set_web_submit_switch(switch_::Switch *s) { this->web_submit_switch_ = s; }
  void set_auth(const std::string &user, const std::string &password) {
    this->username_ = user;
    this->password_ = password;
  }
  void set_api_key(const std::string &key) { this->api_key_ = key; }
  void set_debug(bool debug) { this->debug_ = debug; }

  // Main entry point
  // raw_value and confidence are passed for metadata/logging
  void collect_image(std::shared_ptr<camera::CameraImage> frame, int width, int height, const std::string &format,
                     const std::string &raw_value, float confidence, const std::string &metadata = "");

 protected:
  std::string upload_url_;
  std::string username_;
  std::string password_;
  std::string api_key_;
  bool debug_{false};
  switch_::Switch *web_submit_switch_{nullptr};

  // Helper to upload
  bool upload_image(const uint8_t *data, size_t len, const std::string &raw_value, float confidence,
                    const char *metadata = nullptr);
  // Internal synchronous upload
  bool process_upload_sync(const uint8_t *data, size_t len, const std::string &raw_value, float confidence,
                           const char *metadata = nullptr);

  // RAII-enabled upload job — frees resources on destruction if not processed
  struct UploadJob {
    uint8_t *data{nullptr};
    size_t len{0};
    char value[32]{};  // Fixed size string buffer
    float confidence{0.0f};
    char *metadata{nullptr};
    size_t metadata_len{0};

    ~UploadJob() {
      if (this->data)
        free(this->data);
      if (this->metadata)
        free(this->metadata);
    }
    // Prevent copy — queue passes by pointer
    UploadJob(const UploadJob &) = delete;
    UploadJob &operator=(const UploadJob &) = delete;
    // Allow move
    UploadJob(UploadJob &&other) noexcept
        : data(other.data),
          len(other.len),
          confidence(other.confidence),
          metadata(other.metadata),
          metadata_len(other.metadata_len) {
      std::memcpy(this->value, other.value, sizeof(this->value));
      other.data = nullptr;
      other.metadata = nullptr;
      other.len = 0;
      other.metadata_len = 0;
    }
    UploadJob() = default;
  };

  QueueHandle_t upload_queue_{nullptr};
  TaskHandle_t upload_task_handle_{nullptr};
  std::atomic<bool> task_running_{false};
  void start_upload_task();
  static void upload_task(void *arg);
};

}  // namespace data_collector
}  // namespace esphome

#endif  // USE_DATA_COLLECTOR
