#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include "esphome/components/tflite_micro_helper/tflite_micro_helper.h"
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
 * Holds a reference to a TFLiteMicroHelper instance (owned and configured
 * by the tflite_micro_helper component's __init__.py). This layer handles
 * only the zone-based image inference pattern specific to meter reading.
 */
class TFLiteCoordinator {
 public:
  struct InferenceResult {
    float value;
    float confidence;
    bool success;
  };

  /// @brief Set a reference to an externally-owned, already-loaded TFLiteMicroHelper.
  void set_tflite(tflite_micro_helper::TFLiteMicroHelper *tflite) { this->tflite_ = tflite; }

  /// @brief Legacy: load model from raw data (for backward compat with old __init__.py)
  void set_model(const uint8_t *model, size_t length);
  [[nodiscard]] bool load_model();

  /// @brief Unload the active model (and its arena).
  /// @param reset_config If true, also reset the underlying TFLiteMicroHelper's
  ///   model configuration to defaults (P1). Use when switching to a *different*
  ///   model -- re-issue set_model() and all relevant setters before load_model().
  void unload_model(bool reset_config = false);
  [[nodiscard]] bool is_model_loaded() const;

  // Legacy: config setters (delegate to legacy instance, used by old __init__.py)
  void set_model_type(const std::string &t);
  void set_tensor_arena_size(size_t size);
  void set_debug(bool debug);
  void set_input_type(const std::string &t);
  void set_input_channels(int c);
  void set_input_width(int w);
  void set_input_height(int h);
  void set_output_processing(const std::string &p);
  void set_scale_factor(float f);
  void set_input_order(const std::string &o);
  void set_normalize(bool n);
  void set_invert(bool i);

  // -- Accessors (delegate to active TFLiteMicroHelper) --------------
  int get_input_width() const {
    const auto *t = this->active_tflite();
    return t ? t->get_input_width() : 0;
  }
  int get_input_height() const {
    const auto *t = this->active_tflite();
    return t ? t->get_input_height() : 0;
  }
  int get_input_channels() const {
    const auto *t = this->active_tflite();
    return t ? t->get_input_channels() : 0;
  }
  size_t get_tensor_arena_size() const {
    const auto *t = this->active_tflite();
    return t ? t->get_tensor_arena_size() : 0;
  }
  size_t get_tensor_arena_size_actual() const {
    const auto *t = this->active_tflite();
    return t ? t->get_tensor_arena_size_actual() : 0;
  }
  size_t get_model_size_bytes() const {
    const auto *t = this->active_tflite();
    return t ? t->get_model_size_bytes() : 0;
  }
  size_t get_arena_used_bytes() const {
    const auto *t = this->active_tflite();
    return t ? t->get_arena_used_bytes() : 0;
  }
  size_t get_arena_peak_bytes() const {
    const auto *t = this->active_tflite();
    return t ? t->get_arena_stats().used_bytes : 0;
  }
  tflite_micro_helper::ModelSpec get_model_spec() const {
    const auto *t = this->active_tflite();
    return t ? t->get_model_spec() : tflite_micro_helper::ModelSpec{};
  }
  tflite_micro_helper::ArenaStats get_arena_stats() const {
    const auto *t = this->active_tflite();
    return t ? t->get_arena_stats() : tflite_micro_helper::ArenaStats{};
  }
  void report_memory_status() {
    auto *t = this->active_tflite();
    if (t)
      t->report_memory_status();
  }
#if defined(DEBUG_TFLITE_MICRO_HELPER) && defined(USE_TFLITE_MICRO_HELPER)
  void debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data);
#endif

  // -- Domain-specific: Zone-based inference -------------------------
  using ProcessResult = esphome::esp32_camera_utils::ImageProcessor::ProcessResult;
  std::vector<InferenceResult> run_inference(std::span<const ProcessResult> processed_zones);

 private:
  /** @brief Return the active helper: tflite_ (external) or legacy_tflite_ (owned). */
  tflite_micro_helper::TFLiteMicroHelper *active_tflite() const {
    return this->tflite_ ? this->tflite_ : this->legacy_tflite_.get();
  }

  // Pointer to externally-owned TFLiteMicroHelper (or nullptr for legacy mode)
  tflite_micro_helper::TFLiteMicroHelper *tflite_{nullptr};

  // Legacy: owned instance for backward compat (when old __init__.py passes raw data)
  std::unique_ptr<tflite_micro_helper::TFLiteMicroHelper> legacy_tflite_;
  mutable std::mutex model_mutex_;

  bool process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult &result, float *value,
                            float *confidence);
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_METER_READER_TFLITE
