#include "crop_web_handler.h"

#ifdef USE_METER_READER_TFLITE

#include <esp_camera.h>
#include <img_converters.h>

namespace esphome {
namespace meter_reader_tflite {

void CropRingBuffer::update(std::vector<esp32_camera_utils::ImageProcessor::ProcessResult>& results) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  uint32_t now = millis();
  crops_.clear();
  
  for (size_t i = 0; i < results.size() && i < MAX_CROPS; ++i) {
    auto& result = results[i];
    if (result.data && result.size > 0) {
      CropEntry entry;
      entry.raw_size = result.size;
      entry.timestamp = now;
      
      entry.raw_data = std::shared_ptr<uint8_t>(
        result.data->get(),
        [](uint8_t*) {}
      );
      
      crops_.push_back(std::move(entry));
    }
  }
  
  last_update_ = now;
}

std::vector<CropEntry> CropRingBuffer::get_crops() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (millis() - last_update_ > MAX_AGE_MS) {
    return {};
  }
  
  return crops_;
}

void CropRingBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  crops_.clear();
}

bool CropWebHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  std::string url = request->url().c_str();
  return url.find("/crops") == 0;
}

void CropWebHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  std::string url = request->url().c_str();
  
  auto crops = crop_provider_();
  
  if (crops.empty()) {
    request->send(503, "text/plain", "No crops available yet. Run inference first.");
    return;
  }
  
  int crop_index = -1;
  if (url.length() > 7) {
    std::string index_str = url.substr(7);
    crop_index = std::stoi(index_str);
  }
  
  if (crop_index >= 0 && crop_index < (int)crops.size()) {
    auto& crop = crops[crop_index];
    if (!crop.raw_data || crop.raw_size == 0) {
      request->send(404, "text/plain", "Crop not found");
      return;
    }
    
    const size_t expected_uint8_size = 20 * 32 * 3;
    const size_t expected_float_size = expected_uint8_size * sizeof(float);
    
    std::vector<uint8_t> rgb_buffer;
    uint8_t* source_data = crop.raw_data.get();
    size_t source_size = crop.raw_size;
    
    if (source_size == expected_float_size) {
      rgb_buffer.resize(expected_uint8_size);
      float* float_data = reinterpret_cast<float*>(source_data);
      for (size_t i = 0; i < expected_uint8_size; ++i) {
        int val = static_cast<int>(float_data[i]);
        rgb_buffer[i] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
      }
      source_data = rgb_buffer.data();
    } else if (source_size != expected_uint8_size) {
      request->send(500, "text/plain", ("Unexpected crop size: " + std::to_string(source_size)).c_str());
      return;
    }
    
    uint8_t* jpeg_buf = nullptr;
    size_t jpeg_len = 0;
    
    bool converted = fmt2jpg(source_data, expected_uint8_size, 20, 32, PIXFORMAT_RGB888, 80, &jpeg_buf, &jpeg_len);
    
    if (!converted || !jpeg_buf) {
      request->send(500, "text/plain", "JPEG conversion failed");
      return;
    }
    
    web_server_idf::AsyncWebServerResponse *response = 
        request->beginResponse(200, "image/jpeg", jpeg_buf, jpeg_len);
    request->send(response);
    
    free(jpeg_buf);
    return;
  }
  
  std::string html = "<!DOCTYPE html><html><head>"
                "<title>Crop Zones</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; background: #222; color: white; padding: 20px; }"
                ".crop-container { display: inline-block; margin: 10px; text-align: center; }"
                ".crop-container img { border: 2px solid #444; background: #333; }"
                ".crop-label { margin-top: 5px; font-size: 14px; }"
                "</style></head><body>"
                "<h1>Crop Zones</h1>"
                "<div>";
  
  for (size_t i = 0; i < crops.size(); i++) {
    html += "<div class='crop-container'>"
            "<img src='/crops/" + std::to_string(i) + "' width='64' height='40' alt='Crop " + std::to_string(i) + "'>"
            "<div class='crop-label'>Zone " + std::to_string(i) + "</div>"
            "</div>";
  }
  
  html += "</div><p><a href='/preview'>View full preview</a></p>"
          "</body></html>";
  
  request->send(200, "text/html", html.c_str());
}

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_METER_READER_TFLITE
