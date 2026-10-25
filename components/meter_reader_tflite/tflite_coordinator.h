#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include "esphome/components/tflite_micro_runtime/tflite_micro_runtime.h"
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <span>

namespace esphome {
namespace meter_reader_tflite {

/**
 * @brief Domain-specific TFLite coordinator for image zone inference.
 *
 * Holds a reference to a TFLiteMicroHelper instance (owned and configured by
 * the tflite_micro_runtime component's __init__.py, one instance per model).
 * This layer handles only the zone-based image inference pattern specific to
 * meter reading: it never loads, configures or unloads a model on its own.
 */
class TFLiteCoordinator {
 public:
  struct InferenceResult {
    float value;
    float confidence;
    bool success;
  };

  /// @brief Set a reference to an externally-owned, already-configured TFLiteMicroHelper.
  void set_tflite(tflite_micro_runtime::TFLiteMicroHelper *tflite) { this->tflite_ = tflite; }

  /// @brief Make sure the referenced model is loaded (setup() already does it for image models).
  [[nodiscard]] bool load_model();
  /// @brief Unload the referenced model (and its arena).
  /// @param reset_config If true, also reset the runtime's model configuration to
  ///   defaults. Use when switching to a *different* model.
  void unload_model(bool reset_config = false);
  [[nodiscard]] bool is_model_loaded() const;

  // -- Enum mapping for esp32_camera_utils ---------------------------
  /// @brief Map the runtime's input type onto esp32_camera_utils::ImageProcessorInputType.
  static int to_processor_input_type(tflite_micro_runtime::InputType input_type);
  /// @brief Map the runtime's channel order onto ImageProcessorConfig::input_order.
  static std::string to_input_order_name(tflite_micro_runtime::InputOrder input_order);

  // -- Accessors (delegate to the referenced TFLiteMicroHelper) ------
  int get_input_width() const {
    const auto *t = this->tflite_;
    return t ? t->get_input_width() : 0;
  }
  int get_input_height() const {
    const auto *t = this->tflite_;
    return t ? t->get_input_height() : 0;
  }
  int get_input_channels() const {
    const auto *t = this->tflite_;
    return t ? t->get_input_channels() : 0;
  }
  size_t get_tensor_arena_size() const {
    const auto *t = this->tflite_;
    return t ? t->get_tensor_arena_size() : 0;
  }
  size_t get_tensor_arena_size_actual() const {
    const auto *t = this->tflite_;
    return t ? t->get_tensor_arena_size_actual() : 0;
  }
  size_t get_model_size_bytes() const {
    const auto *t = this->tflite_;
    return t ? t->get_model_size_bytes() : 0;
  }
  size_t get_arena_used_bytes() const {
    const auto *t = this->tflite_;
    return t ? t->get_arena_used_bytes() : 0;
  }
  size_t get_arena_peak_bytes() const {
    const auto *t = this->tflite_;
    return t ? t->get_arena_stats().used_bytes : 0;
  }
  tflite_micro_runtime::ModelSpec get_model_spec() const {
    const auto *t = this->tflite_;
    return t ? t->get_model_spec() : tflite_micro_runtime::ModelSpec{};
  }
  tflite_micro_runtime::ArenaStats get_arena_stats() const {
    const auto *t = this->tflite_;
    return t ? t->get_arena_stats() : tflite_micro_runtime::ArenaStats{};
  }
  void report_memory_status() {
    auto *t = this->tflite_;
    if (t)
      t->report_memory_status();
  }
#ifdef USE_TFLITE_MODEL_DEBUG
  void debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data);
#endif

  // -- Domain-specific: Zone-based inference -------------------------
  using ProcessResult = esphome::esp32_camera_utils::ImageProcessor::ProcessResult;
  std::vector<InferenceResult> run_inference(std::span<const ProcessResult> processed_zones);

 private:
  bool process_model_result(const ProcessResult &result, float *value, float *confidence);

  // Pointer to the externally-owned TFLiteMicroHelper (owned by tflite_micro_runtime)
  tflite_micro_runtime::TFLiteMicroHelper *tflite_{nullptr};
  mutable std::mutex model_mutex_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_METER_READER_TFLITE
