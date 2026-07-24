#include "tflite_coordinator.h"
#include "esphome/core/log.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "tflite_coordinator";

void TFLiteCoordinator::set_model(const uint8_t *model, size_t length) {
  if (!this->legacy_tflite_) {
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  }
  this->legacy_tflite_->set_model(model, length);
}

// Legacy setters — delegate to legacy instance (or create one)
void TFLiteCoordinator::set_model_type(const std::string &t) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_model_type(t);
}
void TFLiteCoordinator::set_tensor_arena_size(size_t size) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_tensor_arena_size(size);
}
void TFLiteCoordinator::set_debug(bool debug) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_debug(debug);
}
void TFLiteCoordinator::set_input_type(const std::string &t) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_input_type(t);
}
void TFLiteCoordinator::set_input_channels(int c) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_input_channels(c);
}
void TFLiteCoordinator::set_input_width(int w) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_input_width(w);
}
void TFLiteCoordinator::set_input_height(int h) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_input_height(h);
}
void TFLiteCoordinator::set_output_processing(const std::string &p) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_output_processing(p);
}
void TFLiteCoordinator::set_scale_factor(float f) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_scale_factor(f);
}
void TFLiteCoordinator::set_input_order(const std::string &o) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_input_order(o);
}
void TFLiteCoordinator::set_normalize(bool n) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_normalize(n);
}
void TFLiteCoordinator::set_invert(bool i) {
  if (!this->legacy_tflite_)
    this->legacy_tflite_ = std::make_unique<tflite_micro_helper::TFLiteMicroHelper>();
  this->legacy_tflite_->set_invert(i);
}

bool TFLiteCoordinator::load_model() {
  if (this->tflite_ != nullptr) {
    if (!this->tflite_->is_model_loaded()) {
      return this->tflite_->load_model();
    }
    return true;
  }
  if (this->legacy_tflite_) {
    return this->legacy_tflite_->load_model();
  }
  ESP_LOGE(TAG, "No TFLiteMicroHelper set. Call set_tflite() or set_model() first.");
  return false;
}

void TFLiteCoordinator::unload_model() {
  std::lock_guard<std::mutex> lock(this->model_mutex_);
  if (this->tflite_) {
    this->tflite_->unload_model();
  }
  if (this->legacy_tflite_) {
    this->legacy_tflite_->unload_model();
  }
}

bool TFLiteCoordinator::is_model_loaded() const {
  if (this->tflite_ != nullptr) {
    return this->tflite_->is_model_loaded();
  }
  if (this->legacy_tflite_) {
    return this->legacy_tflite_->is_model_loaded();
  }
  return false;
}

bool TFLiteCoordinator::process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult &result,
                                             float *value, float *confidence) {
  tflite_micro_helper::TFLiteMicroHelper *tflite = this->tflite_ ? this->tflite_ : this->legacy_tflite_.get();
  if (!tflite || !tflite->is_model_loaded()) {
    ESP_LOGE(TAG, "Cannot run inference - Model not loaded");
    return false;
  }
  const uint8_t *src_data = result.data ? result.data->get() : nullptr;
  if (!src_data) {
    ESP_LOGE(TAG, "Null result data buffer");
    return false;
  }
  auto output = tflite->run_inference(src_data, result.size);
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

#if defined(DEBUG_TFLITE_MICRO_HELPER) && defined(USE_TFLITE_MICRO_HELPER)
void TFLiteCoordinator::debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data) {
  tflite_micro_helper::TFLiteMicroHelper *tflite = this->tflite_ ? this->tflite_ : this->legacy_tflite_.get();
  if (tflite) {
    tflite->debug_test_parameters(zone_data);
  }
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome
