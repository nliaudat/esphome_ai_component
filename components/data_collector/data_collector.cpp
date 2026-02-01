#include "data_collector.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

// For JPEG encoding
// #include "esphome/components/esp32_camera/camera_image.h"
#include "img_converters.h" // ESP32 Camera Utils

// For HTTP Client
#include "esp_http_client.h"

namespace esphome {
namespace data_collector {

static const char *const TAG = "data_collector";

void DataCollector::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Data Collector...");
}

void DataCollector::dump_config() {
  ESP_LOGCONFIG(TAG, "Data Collector:");
  ESP_LOGCONFIG(TAG, "  Upload URL: %s", this->upload_url_.c_str());
  if (this->web_submit_switch_) {
    ESP_LOGCONFIG(TAG, "  Web Submit Switch: %s", this->web_submit_switch_->get_name().c_str());
  }
}

void DataCollector::collect_image(std::shared_ptr<camera::CameraImage> frame, int width, int height, const std::string& format, float raw_value, float confidence) {
  if (!frame) {
    ESP_LOGW(TAG, "No frame provided for collection");
    return;
  }

  // Check switch state if configured
  if (this->web_submit_switch_ && !this->web_submit_switch_->state) {
    ESP_LOGD(TAG, "Data collection skipped (switch off)");
    return;
  }

  if (this->upload_url_.empty()) {
    ESP_LOGW(TAG, "No upload URL configured");
    return;
  }

  uint32_t start_time = millis();
  ESP_LOGI(TAG, "Starting data collection (Value: %.2f, Confidence: %.2f%%, Size: %dx%d, Fmt: %s)", 
           raw_value, confidence * 100.0f, width, height, format.c_str());

  // Determine format
  pixformat_t pix_fmt = PIXFORMAT_GRAYSCALE;
  if (format == "JPEG") pix_fmt = PIXFORMAT_JPEG;
  else if (format == "RGB565") pix_fmt = PIXFORMAT_RGB565;
  else if (format == "RGB888") pix_fmt = PIXFORMAT_RGB888;
  else if (format == "YUV422") pix_fmt = PIXFORMAT_YUV422;
  
  // Basic JPEG encoding
  uint8_t *jpeg_buf = nullptr;
  size_t jpeg_len = 0;
  bool success = false;
  
  if (pix_fmt == PIXFORMAT_JPEG) {
      // Already JPEG, just cast (copy might be needed if upload logic modifies it, but here we likely just send)
      jpeg_buf = (uint8_t*)frame->get_data_buffer();
      jpeg_len = frame->get_data_length();
      
      // We don't own this buffer, so we shouldn't free it unless we copied it.
      // But upload_image takes raw pointer.
      this->upload_image(jpeg_buf, jpeg_len, raw_value, confidence);
      // No free needed for frame buffer
      return; 
  }

  // Convert to JPEG
  bool converted = fmt2jpg((uint8_t*)frame->get_data_buffer(), frame->get_data_length(), width, height, pix_fmt, 90, &jpeg_buf, &jpeg_len);
  
  if (!converted || !jpeg_buf) {
    ESP_LOGE(TAG, "JPEG compression failed");
    return;
  }
  
  success = this->upload_image(jpeg_buf, jpeg_len, raw_value, confidence);
  
  if (jpeg_buf) {
    free(jpeg_buf);
  }
}

bool DataCollector::upload_image(const uint8_t *data, size_t len, float raw_value, float confidence) {
  if (this->upload_url_.empty()) return false;

  ESP_LOGI(TAG, "Uploading image to %s...", this->upload_url_.c_str());

  esp_http_client_config_t config = {};
  config.url = this->upload_url_.c_str();
  config.method = HTTP_METHOD_POST;
  config.timeout_ms = 10000;

  if (!username_.empty()) {
      config.username = username_.c_str();
      config.password = password_.c_str();
      config.auth_type = HTTP_AUTH_TYPE_BASIC;
  }

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (!client) {
    ESP_LOGE(TAG, "Failed to allow http client");
    return false;
  }

  // Set Headers
  esp_http_client_set_header(client, "Content-Type", "image/jpeg");
  
  if (!api_key_.empty()) {
       // Check if it starts with "Bearer " or just a key
       if (api_key_.rfind("Bearer ", 0) == 0) {
           esp_http_client_set_header(client, "Authorization", api_key_.c_str());
       } else {
           // Default to X-Api-Key if no Bearer token structure is detected.
           esp_http_client_set_header(client, "X-Api-Key", api_key_.c_str());
       }
  }
  
  // Add metadata headers
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", raw_value);
  esp_http_client_set_header(client, "X-Meter-Value", buf);
  
  snprintf(buf, sizeof(buf), "%.4f", confidence);
  esp_http_client_set_header(client, "X-Meter-Confidence", buf);

  esp_http_client_set_post_field(client, (const char *)data, len);

  esp_err_t err = esp_http_client_perform(client);
  bool success = false;
  
  if (err == ESP_OK) {
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld", 
             status_code, esp_http_client_get_content_length(client));
    if (this->debug_) {
        // Log response body if small? Or just detailed info
        // esp_http_client_read handling requires event loop or full read.
        // For now, log that we finished.
        ESP_LOGD(TAG, "Full upload metrics: Value=%.2f, Conf=%.4f, Size=%zu", raw_value, confidence, len);
        ESP_LOGD(TAG, "Headers sent: X-Meter-Value, X-Meter-Confidence, Content-Type, Authorization");
    }
    success = (status_code >= 200 && status_code < 300);
  } else {
    ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return success;
}

}  // namespace data_collector
}  // namespace esphome
