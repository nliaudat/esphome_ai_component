#include "tflite_coordinator.h"
#include "esphome/core/log.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "tflite_coordinator";

bool TFLiteCoordinator::process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult &result,
                                             float *value, float *confidence) {
  const uint8_t *src_data = result.data ? result.data->get() : nullptr;
  if (!src_data) {
    ESP_LOGE(TAG, "Null result data buffer");
    return false;
  }

  auto output = this->tflite_.run_inference(src_data, result.size);
  *value = output.value;
  *confidence = output.confidence;

  if (*confidence < 0.0001f) {
    ESP_LOGW(TAG, "Zero confidence detected");
  }
  return true;
}

std::vector<TFLiteCoordinator::InferenceResult> TFLiteCoordinator::run_inference(
    std::span<const ProcessResult> processed_zones) {
  std::lock_guard<std::mutex> lock(this->model_mutex_);

  std::vector<InferenceResult> results;
  if (!this->tflite_.is_model_loaded()) {
    ESP_LOGE(TAG, "Cannot run inference - Model not ready");
    return results;
  }

  for (const auto &zone_result : processed_zones) {
    float val, conf;
    if (this->process_model_result(zone_result, &val, &conf)) {
      results.push_back({val, conf, true});
    } else {
      results.push_back({0.0f, 0.0f, false});
    }
  }
  return results;
}

#ifdef DEBUG_TFLITE_MICRO_HELPER
void TFLiteCoordinator::debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data) {
  this->tflite_.debug_test_parameters(zone_data);
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome
