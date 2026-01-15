#pragma once



#ifdef USE_CAMERA_ROTATOR
#ifdef USE_WEB_SERVER
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include "esphome/core/log.h"
#include <memory>
#include <functional>

namespace esphome {
namespace esp32_camera_utils {

class PreviewWebHandler : public web_server_idf::AsyncWebHandler {
 public:
  /**
   * @brief Construct a new Preview Web Handler object
   * 
   * @param image_provider Callback that returns a shared pointer to the current preview image (RotatedPreviewImage)
   */
  PreviewWebHandler(std::function<std::shared_ptr<camera::CameraImage>()> image_provider);

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;

 private:
  std::function<std::shared_ptr<camera::CameraImage>()> image_provider_;
};

}  // namespace esp32_camera_utils
}  // namespace esphome
#endif
#endif


