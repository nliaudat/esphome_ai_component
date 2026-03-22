#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE
#ifdef USE_WEB_SERVER

#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include <vector>
#include <functional>
#include <string>
#include <memory>
#include <mutex>

namespace esphome {
namespace meter_reader_tflite {

struct CropEntry {
  std::shared_ptr<uint8_t> raw_data;
  size_t raw_size{0};
  uint32_t timestamp{0};
};

class CropRingBuffer {
 public:
  static constexpr size_t MAX_CROPS = 8;
  static constexpr uint32_t MAX_AGE_MS = 30000;
  
  void update(std::vector<esp32_camera_utils::ImageProcessor::ProcessResult>& results);
  std::vector<CropEntry> get_crops();
  void clear();
  
 private:
  std::vector<CropEntry> crops_;
  std::mutex mutex_;
  uint32_t last_update_{0};
};

class CropWebHandler : public web_server_idf::AsyncWebHandler {
 public:
  CropWebHandler(std::function<std::vector<CropEntry>()> crop_provider)
      : crop_provider_(crop_provider) {}
  
  virtual ~CropWebHandler() = default;

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;

 private:
  std::function<std::vector<CropEntry>()> crop_provider_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_WEB_SERVER
#endif  // USE_METER_READER_TFLITE
