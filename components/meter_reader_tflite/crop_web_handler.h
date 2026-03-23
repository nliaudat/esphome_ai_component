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
#include <array>

namespace esphome {
namespace meter_reader_tflite {

struct CropEntry {
  std::shared_ptr<uint8_t> raw_data;
  size_t raw_size{0};
  uint32_t timestamp{0};
  
  static std::shared_ptr<uint8_t> allocate_crop_buffer(size_t size) {
    uint8_t* ptr = static_cast<uint8_t*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!ptr) {
      return nullptr;
    }
    return std::shared_ptr<uint8_t>(ptr, [](uint8_t* p) {
      if (p) heap_caps_free(p);
    });
  }
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

struct CropImageSpec {
  static constexpr size_t WIDTH = 20;
  static constexpr size_t HEIGHT = 32;
  static constexpr size_t CHANNELS = 3;
  static constexpr size_t UINT8_SIZE = WIDTH * HEIGHT * CHANNELS;
  static constexpr size_t FLOAT_SIZE = UINT8_SIZE * sizeof(float);
};

struct LowConfidenceCropSet {
  static constexpr size_t MAX_CROPS_PER_SET = 8;
  static constexpr size_t CROP_DATA_SIZE = CropImageSpec::UINT8_SIZE;
  static constexpr size_t MAX_READING_LEN = 32;
  
  float confidence{0.0f};
  char reading[MAX_READING_LEN];
  std::array<float, MAX_CROPS_PER_SET> digit_confidences;
  uint32_t uptime_ms{0};
  bool valid{false};
  
  std::array<std::array<uint8_t, CROP_DATA_SIZE>, MAX_CROPS_PER_SET> crop_data;
  std::array<size_t, MAX_CROPS_PER_SET> crop_sizes;
  size_t num_crops{0};
};

class LowConfidenceCropBuffer {
 public:
  static constexpr size_t MAX_SETS = 5;
  static constexpr float DEFAULT_THRESHOLD = 0.90f;
  
  LowConfidenceCropBuffer();
  ~LowConfidenceCropBuffer();
  
  void save_low_confidence_set(const std::vector<CropEntry>& crops, float confidence, const std::string& reading, const std::vector<float>& digit_confidences = {});
  std::vector<const LowConfidenceCropSet*> get_low_confidence_sets() const;
  const LowConfidenceCropSet* get_set(size_t index) const;
  void set_threshold(float threshold) { threshold_ = threshold; }
  float get_threshold() const { return threshold_; }
  bool is_allocated() const { return sets_ != nullptr; }
  size_t get_buffer_size() const { return sizeof(LowConfidenceCropSet) * MAX_SETS; }
  void clear();
  std::string generate_html_page() const;
  
 private:
  LowConfidenceCropSet* sets_{nullptr};
  size_t current_index_{0};
  float threshold_{DEFAULT_THRESHOLD};
  mutable std::mutex mutex_;
  
  void allocate_in_psram();
};

class CropWebHandler : public web_server_idf::AsyncWebHandler {
 public:
  CropWebHandler(std::function<std::vector<CropEntry>()> crop_provider,
                 LowConfidenceCropBuffer* low_conf_buffer = nullptr)
      : crop_provider_(crop_provider), low_conf_buffer_(low_conf_buffer) {}
  
  virtual ~CropWebHandler() = default;

  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override;
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override;

 private:
  std::function<std::vector<CropEntry>()> crop_provider_;
  LowConfidenceCropBuffer* low_conf_buffer_;
  
  void handleLowConfidenceRequest(web_server_idf::AsyncWebServerRequest *request) const;
  void handleLowConfidenceCropRequest(web_server_idf::AsyncWebServerRequest *request, int set_index, int crop_index) const;
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_WEB_SERVER
#endif  // USE_METER_READER_TFLITE
