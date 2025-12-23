#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include "esphome/components/tflite_micro_helper/model_handler.h"
#include "esphome/components/tflite_micro_helper/memory_manager.h"
#include "model_config.h"

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace esphome {
namespace meter_reader_tflite {

class TFLiteCoordinator {
 public:
  struct ModelSpec {
      int input_width;
      int input_height;
      int input_channels;
      bool normalize;
      std::string input_order;
      int input_type; // 0=UINT8, 1=FLOAT
  };

  struct InferenceResult {
      float value;
      float confidence;
      bool success;
  };

  void set_model_type(const std::string& model_type) { model_type_ = model_type; }
  void set_tensor_arena_size(size_t size) { tensor_arena_size_requested_ = size; }
  
  void setup(const std::string& model_type, size_t tensor_arena_size);
  
  // Model management
  void set_model(const uint8_t *model, size_t length);
  bool load_model();
  bool is_model_loaded() const { return model_loaded_; }
  
  // Inference
  using ProcessResult = esphome::esp32_camera_utils::ImageProcessor::ProcessResult;
  std::vector<InferenceResult> run_inference(const std::vector<ProcessResult>& processed_zones);

  // Getters
  ModelSpec get_model_spec() const;
  int get_input_width() const { return model_handler_.get_input_width(); }
  int get_input_height() const { return model_handler_.get_input_height(); }
  int get_input_channels() const { return model_handler_.get_input_channels(); }


  // Arena Statistics for monitoring fragmentation
  struct ArenaStats {
      size_t total_size;        // Total arena size allocated
      size_t used_bytes;        // Peak bytes used during inference
      size_t wasted_bytes;      // Unused portion (total - used)
      float efficiency;         // Percentage efficiency (used / total)
  };

  size_t get_arena_peak_bytes() const { 
      std::lock_guard<std::mutex> lock(arena_stats_mutex_);
      return cached_arena_stats_.used_bytes; 
  }
  // Aliases for consistency (thread-safe cached values)
  size_t get_arena_used_bytes() const { 
      std::lock_guard<std::mutex> lock(arena_stats_mutex_);
      return cached_arena_stats_.used_bytes; 
  }
  size_t get_tensor_arena_size() const { return tensor_arena_size_requested_; }
  size_t get_tensor_arena_size_actual() const { return tensor_arena_allocation_.actual_size; }
  size_t get_model_size_bytes() const { return model_length_; }
  
  // Get cached arena statistics (thread-safe for dual-core access)
  ArenaStats get_arena_stats() const;
  
  // Update arena stats cache (call after inference on Core 0)
  void update_arena_stats_cache();

  
  // Debug
  void report_memory_status();
  void debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data);

 private:
  // Model Config
  std::string model_type_{"default"};
  size_t tensor_arena_size_requested_{50 * 1024};
  const uint8_t *model_{nullptr};
  size_t model_length_{0};
  
  // State
  bool model_loaded_{false};
  
  // Arena stats cache for thread-safe dual-core access (Core 0 inference, Core 1 main loop)
  mutable std::mutex arena_stats_mutex_;
  ArenaStats cached_arena_stats_{};
  
  // Components
  tflite_micro_helper::MemoryManager memory_manager_;
  tflite_micro_helper::MemoryManager::AllocationResult tensor_arena_allocation_;
  tflite_micro_helper::ModelHandler model_handler_;
  // Last known specs
  float rotation_{0.0f};

  bool allocate_tensor_arena();
  bool process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult& result, float* value, float* confidence);
};

}  // namespace meter_reader_tflite
}  // namespace esphome
