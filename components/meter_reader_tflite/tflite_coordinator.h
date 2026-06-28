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
 * Delegates all generic TFLite operations (loading, arena, config, invoke)
 * to the reusable tflite_micro_helper component. This layer handles only
 * the zone-based image inference pattern specific to meter reading.
 */
class TFLiteCoordinator {
 public:
  struct InferenceResult {
    float value;
    float confidence;
    bool success;
  };

  // -- Config Setters (delegate to TFLiteMicroHelper) ----------------
  void set_model_type(const std::string &t) { this->tflite_.set_model_type(t); }
  void set_tensor_arena_size(size_t size) { this->tflite_.set_tensor_arena_size(size); }
  void set_debug(bool debug) { this->tflite_.set_debug(debug); }
  void set_input_type(const std::string &t) { this->tflite_.set_input_type(t); }
  void set_input_channels(int c) { this->tflite_.set_input_channels(c); }
  void set_input_width(int w) { this->tflite_.set_input_width(w); }
  void set_input_height(int h) { this->tflite_.set_input_height(h); }
  void set_output_processing(const std::string &p) { this->tflite_.set_output_processing(p); }
  void set_scale_factor(float f) { this->tflite_.set_scale_factor(f); }
  void set_input_order(const std::string &o) { this->tflite_.set_input_order(o); }
  void set_normalize(bool n) { this->tflite_.set_normalize(n); }
  void set_invert(bool i) { this->tflite_.set_invert(i); }

  // -- Model management (delegate to TFLiteMicroHelper) --------------
  void set_model(const uint8_t *model, size_t length) { this->tflite_.set_model(model, length); }
  [[nodiscard]] bool load_model() { return this->tflite_.load_model(); }
  void unload_model() { this->tflite_.unload_model(); }
  [[nodiscard]] bool is_model_loaded() const { return this->tflite_.is_model_loaded(); }
  tflite_micro_helper::ModelSpec get_model_spec() const { return this->tflite_.get_model_spec(); }

  // -- Accessors (delegate to TFLiteMicroHelper) ---------------------
  int get_input_width() const { return this->tflite_.get_input_width(); }
  int get_input_height() const { return this->tflite_.get_input_height(); }
  int get_input_channels() const { return this->tflite_.get_input_channels(); }
  size_t get_tensor_arena_size() const { return this->tflite_.get_tensor_arena_size(); }
  size_t get_tensor_arena_size_actual() const { return this->tflite_.get_tensor_arena_size_actual(); }
  size_t get_model_size_bytes() const { return this->tflite_.get_model_size_bytes(); }
  size_t get_arena_used_bytes() const { return this->tflite_.get_arena_used_bytes(); }
  size_t get_arena_peak_bytes() const { return this->tflite_.get_arena_stats().used_bytes; }
  tflite_micro_helper::ArenaStats get_arena_stats() const { return this->tflite_.get_arena_stats(); }
  void report_memory_status() { this->tflite_.report_memory_status(); }
#ifdef DEBUG_TFLITE_MICRO_HELPER
  void debug_test_parameters(const std::vector<std::vector<uint8_t>> &zone_data);
#endif

  // -- Domain-specific: Zone-based inference -------------------------
  using ProcessResult = esphome::esp32_camera_utils::ImageProcessor::ProcessResult;
  std::vector<InferenceResult> run_inference(std::span<const ProcessResult> processed_zones);

 private:
  // Reusable TFLite helper -- handles loading, arena, config, buffer inference
  tflite_micro_helper::TFLiteMicroHelper tflite_;
  mutable std::mutex model_mutex_;

  bool process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult &result, float *value,
                            float *confidence);
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif  // USE_METER_READER_TFLITE
