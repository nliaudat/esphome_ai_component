#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE

#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/camera/camera.h"
#include <vector>
#include <memory>

namespace esphome {
namespace meter_reader_tflite {

// Web handler for serving crop zone images
// Serves individual crops at /crops/<index> and montage at /crops
class CropWebHandler : public web_server_base::Handler {
 public:
  CropWebHandler(std::function<std::vector<std::shared_ptr<camera::CameraImage>>()> crop_provider)
      : crop_provider_(crop_provider) {}

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    String url = request->url();
    return url.startsWith("/crops");
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;

 private:
  std::function<std::vector<std::shared_ptr<camera::CameraImage>>()> crop_provider_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif // USE_METER_READER_TFLITE
