#include "crop_web_handler.h"

#ifdef USE_METER_READER_TFLITE

#include <esp_camera.h>
#include <img_converters.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <climits>

static const char* TAG = "low_conf_crop";

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
      
      entry.raw_data = CropEntry::allocate_crop_buffer(result.size);
      if (entry.raw_data) {
        std::memcpy(entry.raw_data.get(), result.data->get(), result.size);
        crops_.push_back(std::move(entry));
      }
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

LowConfidenceCropBuffer::LowConfidenceCropBuffer() {
  allocate_in_psram();
}

LowConfidenceCropBuffer::~LowConfidenceCropBuffer() {
  if (sets_) {
    free(sets_);
    sets_ = nullptr;
  }
}

void LowConfidenceCropBuffer::allocate_in_psram() {
  size_t alloc_size = sizeof(LowConfidenceCropSet) * MAX_SETS;
  sets_ = static_cast<LowConfidenceCropSet*>(heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM));
  
  if (!sets_) {
    ESP_LOGE(TAG, "Failed to allocate low confidence crop buffer in PSRAM (need %zu bytes)", alloc_size);
    return;
  }
  
  for (size_t i = 0; i < MAX_SETS; ++i) {
    sets_[i].valid = false;
    sets_[i].num_crops = 0;
  }
  
  ESP_LOGI(TAG, "Low confidence crop buffer allocated in PSRAM: %zu bytes (%zu sets)", alloc_size, MAX_SETS);
}

void LowConfidenceCropBuffer::save_low_confidence_set(const std::vector<CropEntry>& crops, float confidence, const std::string& reading, const std::vector<float>& digit_confidences) {
  ESP_LOGD(TAG, "save_low_confidence_set called: confidence=%.2f, threshold=%.2f, crops=%zu", 
           confidence, threshold_, crops.size());
  
  if (!sets_) {
    ESP_LOGE(TAG, "CANNOT SAVE: Low confidence crop buffer not allocated in PSRAM!");
    ESP_LOGE(TAG, "  -> Check PSRAM is enabled and has free space");
    return;
  }
  
  if (crops.empty()) {
    ESP_LOGD(TAG, "Skipping save: no crops available");
    return;
  }
  
  ESP_LOGD(TAG, "Checking threshold: confidence %.2f >= threshold %.2f ? %s", 
           confidence, threshold_, (confidence >= threshold_) ? "YES (skip)" : "NO (save)");
  
  if (confidence >= threshold_) {
    ESP_LOGD(TAG, "Skipping save: confidence %.2f >= threshold %.2f", confidence, threshold_);
    return;
  }
  
  ESP_LOGI(TAG, "Saving low confidence set: reading='%s', confidence=%.2f, crops=%zu", 
           reading.c_str(), confidence, crops.size());
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto& set = sets_[current_index_];
  set.confidence = confidence;
  std::strncpy(set.reading, reading.c_str(), LowConfidenceCropSet::MAX_READING_LEN - 1);
  set.reading[LowConfidenceCropSet::MAX_READING_LEN - 1] = '\0';
  set.uptime_ms = millis();
  set.valid = true;
  set.num_crops = std::min(crops.size(), LowConfidenceCropSet::MAX_CROPS_PER_SET);
  
  // Store per-digit confidences
  for (size_t i = 0; i < LowConfidenceCropSet::MAX_CROPS_PER_SET; ++i) {
    if (i < digit_confidences.size()) {
      set.digit_confidences[i] = digit_confidences[i];
    } else {
      set.digit_confidences[i] = 0.0f;
    }
  }
  
  for (size_t i = 0; i < set.num_crops; ++i) {
    if (crops[i].raw_data && crops[i].raw_size > 0) {
      const uint8_t* source_data = crops[i].raw_data.get();
      size_t source_size = crops[i].raw_size;
      
      if (source_size == CropImageSpec::FLOAT_SIZE) {
        const float* float_data = reinterpret_cast<const float*>(source_data);
        for (size_t j = 0; j < CropImageSpec::UINT8_SIZE; ++j) {
          int val = static_cast<int>(float_data[j]);
          set.crop_data[i][j] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
        }
        set.crop_sizes[i] = CropImageSpec::UINT8_SIZE;
      } else if (source_size == CropImageSpec::UINT8_SIZE) {
        std::memcpy(set.crop_data[i].data(), source_data, CropImageSpec::UINT8_SIZE);
        set.crop_sizes[i] = CropImageSpec::UINT8_SIZE;
      } else {
        ESP_LOGW(TAG, "Unexpected crop size %zu for crop %zu", source_size, i);
        set.crop_sizes[i] = 0;
      }
    } else {
      set.crop_sizes[i] = 0;
    }
  }
  
  current_index_ = (current_index_ + 1) % MAX_SETS;
}

std::vector<const LowConfidenceCropSet*> LowConfidenceCropBuffer::get_low_confidence_sets() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<const LowConfidenceCropSet*> result;
  if (!sets_) {
    return result;
  }
  
  result.reserve(MAX_SETS);
  
  for (size_t i = 0; i < MAX_SETS; ++i) {
    size_t idx = (current_index_ + MAX_SETS - 1 - i) % MAX_SETS;
    if (sets_[idx].valid) {
      result.push_back(&sets_[idx]);
    }
  }
  
  return result;
}

const LowConfidenceCropSet* LowConfidenceCropBuffer::get_set(size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!sets_ || index >= MAX_SETS) {
    return nullptr;
  }
  
  size_t idx = (current_index_ + MAX_SETS - 1 - index) % MAX_SETS;
  if (sets_[idx].valid) {
    return &sets_[idx];
  }
  
  return nullptr;
}

void LowConfidenceCropBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!sets_) {
    return;
  }
  
  for (size_t i = 0; i < MAX_SETS; ++i) {
    sets_[i].valid = false;
    sets_[i].num_crops = 0;
  }
  current_index_ = 0;
}

static int parse_index_safe(const std::string& s) {
  if (s.empty()) return -1;
  for (char c : s) {
    if (!std::isdigit(static_cast<unsigned char>(c))) return -1;
  }
  char* end;
  long val = std::strtol(s.c_str(), &end, 10);
  if (end != s.c_str() + s.length() || val < 0 || val > INT_MAX) return -1;
  return static_cast<int>(val);
}

static std::string escape_html(const char* input) {
  std::string output;
  for (const char* p = input; *p; ++p) {
    switch (*p) {
      case '&': output += "&amp;"; break;
      case '<': output += "&lt;"; break;
      case '>': output += "&gt;"; break;
      case '"': output += "&quot;"; break;
      case '\'': output += "&#x27;"; break;
      default: output += *p; break;
    }
  }
  return output;
}

static std::string format_age(uint32_t timestamp_ms) {
  uint32_t now = millis();
  uint32_t age_ms;
  if (now >= timestamp_ms) {
    age_ms = now - timestamp_ms;
  } else {
    age_ms = (0xFFFFFFFF - timestamp_ms) + now + 1;
  }
  
  if (age_ms < 60000) {
    return std::to_string(age_ms / 1000) + "s ago";
  } else if (age_ms < 3600000) {
    return std::to_string(age_ms / 60000) + "m " + std::to_string((age_ms % 60000) / 1000) + "s ago";
  } else {
    return std::to_string(age_ms / 3600000) + "h " + std::to_string((age_ms % 3600000) / 60000) + "m ago";
  }
}

std::string LowConfidenceCropBuffer::generate_html_page() const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!sets_) {
    return "<html><body><h1>Buffer not allocated</h1><p>PSRAM allocation failed.</p></body></html>";
  }
  
  bool any_valid = false;
  for (size_t i = 0; i < MAX_SETS; ++i) {
    if (sets_[i].valid) {
      any_valid = true;
      break;
    }
  }
  
  if (!any_valid) {
    std::string html = "<html><body><h1>No low-confidence readings captured yet</h1><p>Readings below " + 
                       std::to_string(static_cast<int>(threshold_ * 100)) + 
                       "% confidence will appear here.</p></body></html>";
    return html;
  }
  
  std::string html = "<!DOCTYPE html><html><head>"
                "<title>Low Confidence Readings</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; background: #222; color: white; padding: 20px; }"
                ".reading-set { border: 2px solid #444; margin: 20px 0; padding: 15px; background: #333; }"
                ".reading-header { font-size: 18px; margin-bottom: 10px; color: #ff6b6b; }"
                ".crop-container { display: inline-block; margin: 5px; text-align: center; }"
                ".crop-container img { border: 2px solid #555; background: #444; }"
                ".crop-label { margin-top: 3px; font-size: 12px; }"
                "</style></head><body>"
                "<h1>Low Confidence Readings</h1>"
                "<p>Threshold: " + std::to_string(static_cast<int>(threshold_ * 100)) + "%</p>";
  
  for (size_t i = 0; i < MAX_SETS; ++i) {
    size_t idx = (current_index_ + MAX_SETS - 1 - i) % MAX_SETS;
    const auto& set = sets_[idx];
    if (!set.valid) continue;
    
    std::string time_str = format_age(set.uptime_ms);
    
    html += "<div class='reading-set'>"
            "<div class='reading-header'>Reading: " + escape_html(set.reading) + 
            " | Avg: " + std::to_string(static_cast<int>(set.confidence * 100)) + "%"
            " | " + time_str +
            " | Set #" + std::to_string(i) + "</div>"
            "<div style='display: flex; flex-wrap: wrap;'>";
    
    for (size_t crop_idx = 0; crop_idx < set.num_crops; ++crop_idx) {
      int digit_conf = static_cast<int>(set.digit_confidences[crop_idx] * 100);
      html += "<div class='crop-container'>"
              "<img src='/low-confidence-crops/" + std::to_string(i) + "/" + std::to_string(crop_idx) + 
              "' width='128' height='80' alt='Crop " + std::to_string(crop_idx) + "'>"
              "<div class='crop-label'>" + std::to_string(digit_conf) + "%</div>"
              "</div>";
    }
    
    html += "</div></div>";
  }
  
  html += "<p><a href='/crops'>View current crops</a></p>"
          "</body></html>";
  
  return html;
}

bool CropWebHandler::canHandle(web_server_idf::AsyncWebServerRequest *request) const {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url = std::string(request->url_to(url_buf));
  return url.find("/crops") == 0 || url.find("/low-confidence-crops") == 0;
}

void CropWebHandler::handleRequest(web_server_idf::AsyncWebServerRequest *request) {
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url = std::string(request->url_to(url_buf));
  
  if (url.find("/low-confidence-crops") == 0) {
    handleLowConfidenceRequest(request);
    return;
  }
  
  auto crops = crop_provider_();
  
  if (crops.empty()) {
    request->send(503, "text/plain", "No crops available yet. Run inference first.");
    return;
  }
  
  int crop_index = -1;
  if (url.length() > 7) {
    std::string index_str = url.substr(7);
    crop_index = parse_index_safe(index_str);
  }
  
  if (crop_index >= 0 && crop_index < (int)crops.size()) {
    auto& crop = crops[crop_index];
    if (!crop.raw_data || crop.raw_size == 0) {
      request->send(404, "text/plain", "Crop not found");
      return;
    }
    
    std::vector<uint8_t> rgb_buffer;
    uint8_t* source_data = crop.raw_data.get();
    size_t source_size = crop.raw_size;
    
    if (source_size == CropImageSpec::FLOAT_SIZE) {
      rgb_buffer.resize(CropImageSpec::UINT8_SIZE);
      float* float_data = reinterpret_cast<float*>(source_data);
      for (size_t i = 0; i < CropImageSpec::UINT8_SIZE; ++i) {
        int val = static_cast<int>(float_data[i]);
        rgb_buffer[i] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
      }
      source_data = rgb_buffer.data();
    } else if (source_size != CropImageSpec::UINT8_SIZE) {
      request->send(500, "text/plain", ("Unexpected crop size: " + std::to_string(source_size)).c_str());
      return;
    }
    
    uint8_t* jpeg_buf = nullptr;
    size_t jpeg_len = 0;
    
  bool converted = fmt2jpg(const_cast<uint8_t*>(source_data), CropImageSpec::UINT8_SIZE, 
                           CropImageSpec::WIDTH, CropImageSpec::HEIGHT, PIXFORMAT_RGB888, 80, &jpeg_buf, &jpeg_len);
    
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
            "<img src='/crops/" + std::to_string(i) + "' width='256' height='160' alt='Crop " + std::to_string(i) + "'>"
            "<div class='crop-label'>Zone " + std::to_string(i) + "</div>"
            "</div>";
  }
  
  html += "</div><p><a href='/preview'>View full preview</a></p>"
          "<p><a href='/low-confidence-crops'>View low confidence captures</a></p>"
          "</body></html>";
  
  request->send(200, "text/html", html.c_str());
}

void CropWebHandler::handleLowConfidenceRequest(web_server_idf::AsyncWebServerRequest *request) const {
  if (!low_conf_buffer_) {
    request->send(503, "text/plain", "Low confidence buffer not available");
    return;
  }
  
  char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url = std::string(request->url_to(url_buf));
  
  if (url.length() > 22) {
    std::string path = url.substr(22);
    size_t slash1 = path.find('/');
    if (slash1 != std::string::npos) {
      // Parse indices manually to avoid exceptions (disabled in ESP-IDF)
      auto parse_index = [](const std::string& s) -> int {
        if (s.empty()) return -1;
        // Check all characters are digits
        for (char c : s) {
          if (!std::isdigit(static_cast<unsigned char>(c))) return -1;
        }
        // Use strtol which doesn't throw
        char* end;
        long val = std::strtol(s.c_str(), &end, 10);
        if (end != s.c_str() + s.length() || val < 0 || val > INT_MAX) return -1;
        return static_cast<int>(val);
      };
      
      int set_index = parse_index(path.substr(0, slash1));
      int crop_index = parse_index(path.substr(slash1 + 1));
      
      if (set_index < 0 || crop_index < 0) {
        request->send(400, "text/plain", "Invalid index format");
        return;
      }
      handleLowConfidenceCropRequest(request, set_index, crop_index);
      return;
    }
  }
  
  std::string html = low_conf_buffer_->generate_html_page();
  request->send(200, "text/html", html.c_str());
}

void CropWebHandler::handleLowConfidenceCropRequest(web_server_idf::AsyncWebServerRequest *request, int set_index, int crop_index) const {
  const auto* set = low_conf_buffer_->get_set(set_index);
  
  if (!set || !set->valid || crop_index < 0 || crop_index >= static_cast<int>(set->num_crops)) {
    request->send(404, "text/plain", "Crop not found");
    return;
  }
  
  if (set->crop_sizes[crop_index] == 0) {
    request->send(404, "text/plain", "Crop data empty");
    return;
  }
  
  std::vector<uint8_t> rgb_buffer;
  const uint8_t* source_data = set->crop_data[crop_index].data();
  size_t source_size = set->crop_sizes[crop_index];
  
  if (source_size == CropImageSpec::FLOAT_SIZE) {
    rgb_buffer.resize(CropImageSpec::UINT8_SIZE);
    const float* float_data = reinterpret_cast<const float*>(source_data);
    for (size_t i = 0; i < CropImageSpec::UINT8_SIZE; ++i) {
      int val = static_cast<int>(float_data[i]);
      rgb_buffer[i] = static_cast<uint8_t>(std::max(0, std::min(255, val)));
    }
    source_data = rgb_buffer.data();
  } else if (source_size != CropImageSpec::UINT8_SIZE) {
    request->send(500, "text/plain", "Unexpected crop size");
    return;
  }
  
  uint8_t* jpeg_buf = nullptr;
  size_t jpeg_len = 0;
  
  bool converted = fmt2jpg(const_cast<uint8_t*>(source_data), CropImageSpec::UINT8_SIZE,
                           CropImageSpec::WIDTH, CropImageSpec::HEIGHT, PIXFORMAT_RGB888, 80, &jpeg_buf, &jpeg_len);
  
  if (!converted || !jpeg_buf) {
    request->send(500, "text/plain", "JPEG conversion failed");
    return;
  }
  
  web_server_idf::AsyncWebServerResponse *response = 
      request->beginResponse(200, "image/jpeg", jpeg_buf, jpeg_len);
  request->send(response);
  
  free(jpeg_buf);
}

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_METER_READER_TFLITE
