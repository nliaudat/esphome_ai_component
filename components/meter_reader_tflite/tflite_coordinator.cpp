#include "tflite_coordinator.h"
#include "esphome/core/log.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "tflite_coordinator";

int TFLiteCoordinator::to_processor_input_type(tflite_micro_runtime::InputType input_type) {
  switch (input_type) {
    case tflite_micro_runtime::InputType::INPUT_TYPE_FLOAT32:
      return static_cast<int>(esp32_camera_utils::ImageProcessorInputType::kInputTypeFloat32);
    case tflite_micro_runtime::InputType::INPUT_TYPE_UINT8:
    case tflite_micro_runtime::InputType::INPUT_TYPE_INT8:
      return static_cast<int>(esp32_camera_utils::ImageProcessorInputType::kInputTypeUInt8);
    case tflite_micro_runtime::InputType::INPUT_TYPE_UNKNOWN:
    default:
      return static_cast<int>(esp32_camera_utils::ImageProcessorInputType::kInputTypeUnknown);
  }
}

std::string TFLiteCoordinator::to_input_order_name(tflite_micro_runtime::InputOrder input_order) {
  switch (input_order) {
    case tflite_micro_runtime::InputOrder::INPUT_ORDER_BGR:
      return "BGR";
    case tflite_micro_runtime::InputOrder::INPUT_ORDER_GRAY:
      return "GRAY";
    case tflite_micro_runtime::InputOrder::INPUT_ORDER_RGB:
    default:
      return "RGB";
  }
}

bool TFLiteCoordinator::load_model() {
  if (this->tflite_ == nullptr) {
    ESP_LOGE(TAG, "No TFLiteMicroHelper set. Call set_tflite() first.");
    return false;
  }
  if (this->tflite_->is_model_loaded()) {
    return true;
  }
  // setup() loads an image model, so reaching here means the first load failed or
  // the model was unloaded on purpose; retrying is what the reload button expects.
  return this->tflite_->load_model();
}

void TFLiteCoordinator::unload_model(bool reset_config) {
  std::lock_guard<std::mutex> lock(this->model_mutex_);
  if (this->tflite_ != nullptr) {
    this->tflite_->unload_model(reset_config);
  }
}

bool TFLiteCoordinator::is_model_loaded() const {
  return this->tflite_ != nullptr && this->tflite_->is_model_loaded();
}

bool TFLiteCoordinator::process_model_result(const ProcessResult &result, float *value, float *confidence) {
  if (this->tflite_ == nullptr || !this->tflite_->is_model_loaded()) {
    ESP_LOGE(TAG, "Cannot run inference - Model not loaded");
    return false;
  }
  const uint8_t *src_data = result.data ? result.data->get() : nullptr;
  if (!src_data) {
    ESP_LOGE(TAG, "Null result data buffer");
    return false;
  }
  // nullopt means the run failed (buffer size mismatch, invoke error, ...); the
  // reason is logged by the runtime. A failed run is never reported as a reading.
  auto output = this->tflite_->run_inference(src_data, result.size);
  if (!output.has_value()) {
    ESP_LOGW(TAG, "Inference failed for zone (%zu bytes)", result.size);
    return false;
  }
  *value = output->value;
  *confidence = output->confidence;
  if (*confidence < 0.0001f) {
    ESP_LOGW(TAG, "Zero confidence detected");
  }
  return true;
}

std::vector<TFLiteCoordinator::InferenceResult> TFLiteCoordinator::run_inference(
    std::span<const ProcessResult> processed_zones) {
  std::lock_guard<std::mutex> lock(this->model_mutex_);
  std::vector<InferenceResult> results;
  results.reserve(processed_zones.size());
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

#ifdef USE_TFLITE_MODEL_DEBUG
void TFLiteCoordinator::debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data) {
  if (this->tflite_ != nullptr) {
    this->tflite_->debug_test_parameters(zone_data);
  }
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome
