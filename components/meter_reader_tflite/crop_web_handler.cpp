#include "crop_web_handler.h"

#ifdef USE_METER_READER_TFLITE

namespace esphome {
namespace meter_reader_tflite {

void CropWebHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  String url = request->url();
  
  // Get crops from provider
  auto crops = crop_provider_();
  
  if (crops.empty()) {
    request->send(503, "text/plain", "No crops available yet. Run inference first.");
    return;
  }
  
  // Check if requesting individual crop: /crops/0, /crops/1, etc.
  int crop_index = -1;
  if (url.length() > 6) {  // "/crops" is 6 chars
    String index_str = url.substring(7);  // Skip "/crops/"
    crop_index = index_str.toInt();
  }
  
  if (crop_index >= 0 && crop_index < (int)crops.size()) {
    // Serve individual crop
    auto crop = crops[crop_index];
    if (!crop) {
      request->send(404, "text/plain", "Crop not found");
      return;
    }
    
    // For now, just send raw data if it's JPEG, or convert if needed
    size_t len = crop->get_data_length();
    uint8_t *buf = crop->get_data_buffer();
    
    if (!buf || len == 0) {
      request->send(500, "text/plain", "Invalid crop buffer");
      return;
    }
    
    // Send as JPEG
    web_server_idf::AsyncWebServerResponse *response = 
        request->beginResponse(200, "image/jpeg", (const uint8_t*)buf, len);
    request->send(response);
    return;
  }
  
  // Serve montage HTML page at /crops
  String html = "<!DOCTYPE html><html><head>"
                "<title>Crop Zones</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; background: #222; color: white; padding: 20px; }"
                ".crop-container { display: inline-block; margin: 10px; text-align: center; }"
                ".crop-container img { border: 2px solid #444; background: #333; }"
                ".crop-label { margin-top: 5px; font-size: 14px; }"
                "</style></head><body>"
                "<h1>Crop Zones</h1>"
                "<p>Last captured at: " + String(millis() / 1000) + "s uptime</p>"
                "<div>";
  
  for (size_t i = 0; i < crops.size(); i++) {
    html += "<div class='crop-container'>"
            "<img src='/crops/" + String(i) + "' width='64' height='40' alt='Crop " + String(i) + "'>"
            "<div class='crop-label'>Zone " + String(i) + "</div>"
            "</div>";
  }
  
  html += "</div><p><a href='/preview'>View full preview</a></p>"
          "</body></html>";
  
  request->send(200, "text/html", html);
}

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif // USE_METER_READER_TFLITE
