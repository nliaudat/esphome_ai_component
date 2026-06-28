#include "esphome/core/defines.h"
#ifdef USE_WEB_SERVER
#include "preview_web_handler.h"
#include <esp_camera.h>
#include <img_converters.h>
#include <esp_heap_caps.h>


namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "preview_web_handler";

PreviewWebHandler::PreviewWebHandler(std::function<std::shared_ptr<camera::CameraImage>()> image_provider)
    : image_provider_(image_provider) {}

bool PreviewWebHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(url_buf) == "/preview";
}

void PreviewWebHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  if (!this->image_provider_) {
      request->send(500, "text/plain", "Image provider not configured");
      return;
  }

  std::shared_ptr<camera::CameraImage> img_ptr = this->image_provider_();
  if (!img_ptr) {
      ESP_LOGW(TAG, "HTTP Preview requested but no image available");
      // Send HTML with meta-refresh to auto-retry every 5 seconds
      const char* html = "<html><head><meta http-equiv=\"refresh\" content=\"5\">"
                         "<title>Camera Preview</title></head><body>"
                         "<p>Preview not available yet. Waiting for first frame...</p>"
                         "<p>Page auto-refreshes every 5 seconds.</p></body></html>";
      web_server_idf::AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
      request->send(response);
      return;
  }

  // std::static_pointer_cast: safe because get_preview_image() always returns RotatedPreviewImage
  auto image = std::static_pointer_cast<RotatedPreviewImage>(img_ptr);

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
      // Use beginResponse for binary safe handling to ensure buffer is properly managed.
      web_server_idf::AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", (const uint8_t*)buf, len);
      request->send(response);
      ESP_LOGI(TAG, "Serving existing JPEG preview (%u bytes)", len);
      return;
  }

  // If RGB/Gray, convert to JPEG
  uint8_t *jpeg_buf = nullptr;
  size_t jpeg_len = 0;

  // Debug: Log memory state before compression
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  ESP_LOGI(TAG, "Preview Debug: Image %dx%d fmt=%d | Heap: Free=%u, Int=%u, SPI=%u",
           image->get_width(), image->get_height(), image->get_format(),
           free_heap, free_internal, free_spiram);

  // Use fmt2jpg from esp coversion lib
  // Reverting to 80 for debug as requested, logs will show if OOM matches reduced memory
  bool converted = fmt2jpg(buf, len, image->get_width(), image->get_height(), image->get_format(), 80, &jpeg_buf, &jpeg_len);

  if (!converted || !jpeg_buf) {
      ESP_LOGE(TAG, "JPEG compression failed");
      request->send(500, "text/plain", "JPEG compression failed");
      return;
  }

  // RAII for jpeg_buf
  struct FreeDeleter {
      void operator()(uint8_t* p) const { free(p); }
  };
  std::unique_ptr<uint8_t[], FreeDeleter> jpeg_buf_ptr(jpeg_buf);

  // Send the converted JPEG
  // beginResponse will copy the buffer content so we can free our local jpeg_buf (via RAII)
  web_server_idf::AsyncWebServerResponse *response = request->beginResponse(200, "image/jpeg", (const uint8_t*)jpeg_buf, jpeg_len);
  request->send(response);

  ESP_LOGD(TAG, "Serving converted JPEG preview (%u bytes)", jpeg_len);
}

}  // namespace esp32_camera_utils
}  // namespace esphome
#endif
