

#ifdef DEV_ENABLE_ROTATION
#include "preview_web_handler.h"
#include <esp_camera.h>
#include <img_converters.h>


namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "preview_web_handler";

PreviewWebHandler::PreviewWebHandler(std::function<std::shared_ptr<camera::CameraImage>()> image_provider)
    : image_provider_(image_provider) {}

bool PreviewWebHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  return request->url() == "/preview";
}

void PreviewWebHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  if (!image_provider_) {
      request->send(500, "text/plain", "Image provider not configured");
      return;
  }

  std::shared_ptr<camera::CameraImage> img_ptr = image_provider_();
  // Safe downcast functionality should be provided by image_processor.h or static_pointer_cast
  // We use the std::static_pointer_cast approach as defined in the plan
  auto image = std::static_pointer_cast<RotatedPreviewImage>(img_ptr);

  if (!image) {
      ESP_LOGW(TAG, "HTTP Preview requested but no image available");
      request->send(503, "text/plain", "Preview not available yet. Please try again.");
      return;
  }

  size_t len = image->get_data_length();
  uint8_t *buf = image->get_data_buffer();

  if (!buf || len == 0) {
      request->send(500, "text/plain", "Invalid image buffer");
      return;
  }
  
  // If the image is already JPEG, send directly
  if (image->get_format() == PIXFORMAT_JPEG) {
      // NOTE: Using beginResponse to ensure buffer copy if needed by underlying implementation, 
      // though typically for static buffers direct send might work. 
      // However, RotatedPreviewImage owns the buffer. 
      // AsyncWebServerRequest::send with data copies it. 
      // request->send(200, "image/jpeg", buf, len) logic for binary?
      // Better use beginResponse for binary safe handling.
      web_server_idf::AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", (const uint8_t*)buf, len);
      request->send(response);
      ESP_LOGI(TAG, "Serving existing JPEG preview (%u bytes)", len);
      return;
  }

  // If RGB/Gray, convert to JPEG
  uint8_t *jpeg_buf = nullptr;
  size_t jpeg_len = 0;
  
  // Use fmt2jpg from esp coversion lib
  bool converted = fmt2jpg(buf, len, image->get_width(), image->get_height(), image->get_format(), 80, &jpeg_buf, &jpeg_len);
  
  if (!converted || !jpeg_buf) {
      ESP_LOGE(TAG, "JPEG compression failed");
      request->send(500, "text/plain", "JPEG compression failed");
      return;
  }

  // Send the converted JPEG
  // beginResponse will copy the buffer content so we can free our local jpeg_buf
  web_server_idf::AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", (const uint8_t*)jpeg_buf, jpeg_len);
  request->send(response);
  
  // Free the buffer allocated by fmt2jpg
  free(jpeg_buf);
  
  ESP_LOGD(TAG, "Serving converted JPEG preview (%u bytes)", jpeg_len);
}

}  // namespace esp32_camera_utils
}  // namespace esphome
#endif


