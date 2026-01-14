#pragma once

#include <memory>
#include <string>
#include <vector>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "op_resolver.h"
#include "memory_manager.h"

namespace esphome {
namespace tflite_micro_helper {

// Maximum number of operators to register
constexpr int MAX_OPERATORS = 30;

struct ModelConfig {
  std::string description;
  std::string tensor_arena_size;
  std::string output_processing;
  float scale_factor{1.0f};
  std::string input_type;
  int input_channels{3};
  std::string input_order{"RGB"};
  std::vector<int> input_size;
  bool normalize{false};
  bool invert{false};
};

struct ProcessedOutput {
  float value;
  float confidence;
};
struct ConfigTestResult {
  ModelConfig config;
  float avg_confidence;
  std::vector<float> zone_confidences;
  std::vector<float> zone_values;
};

class ModelHandler {
 public:
  // High-level load model that handles memory allocation
  [[nodiscard]] bool load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config);
  
  // Unload model and free resources
  void unload();

  // Low-level load model (kept for internal use or specific cases)
  [[nodiscard]] bool load_model_with_arena(const uint8_t *model_data, size_t model_size,
                 uint8_t* tensor_arena, size_t tensor_arena_size,
                 const ModelConfig &config);

  TfLiteStatus invoke() {
    return interpreter_->Invoke();
  }

  TfLiteTensor* input_tensor() const {
    return interpreter_->input(0);
  }

  TfLiteTensor* output_tensor() const {
    return interpreter_->output(0);
  }
  
  // Helper methods for input dimensions
  int get_input_width() const {
    TfLiteTensor* input = input_tensor();
    if (!input || input->dims->size < 4) return 0;
    return input->dims->data[2]; // [batch, height, width, channels]
  }
  
  int get_input_height() const {
    TfLiteTensor* input = input_tensor();
    if (!input || input->dims->size < 4) return 0;
    return input->dims->data[1];
  }
  
  int get_input_channels() const {
    TfLiteTensor* input = input_tensor();
    if (!input || input->dims->size < 4) return 0;
    return input->dims->data[3];
  }

  ProcessedOutput process_output(const float *output_data) const;
  ProcessedOutput process_output(TfLiteTensor* output_tensor) const;
  
  const ModelConfig& get_config() const { return config_; }
  
  size_t get_arena_used_bytes() const {
    return interpreter_->arena_used_bytes();
  }
  
  // Memory management helpers
  size_t get_tensor_arena_size() const { return tensor_arena_allocation_.actual_size; }
  
  void report_memory_status();
  
  // Debug helpers
  void log_input_stats() const;
  void debug_input_pattern() const;
  
  // CRC32 verification
  static uint32_t calculate_crc32(const uint8_t *data, size_t length);
  bool verify_model_crc(const uint8_t *model_data, size_t length);
  void debug_model_architecture() const;
  bool validate_model_config() const;
  
  // Advanced Debugging
  void debug_input_quantization_analysis(const uint8_t* input_data, size_t input_size, const std::string& stage) const;
  void debug_input_tensor_details() const;
  void debug_tensor_types() const;
  void debug_input_data_stats(const uint8_t* input_data, size_t input_size) const;
  void debug_quantized_input_details(TfLiteTensor* input, size_t input_size) const;
  void debug_int8_conversion_details(TfLiteTensor* input, const uint8_t* input_data, size_t input_size) const;
  void debug_pre_inference_state() const;
  void debug_output_tensor_details(TfLiteTensor* output) const;
  void debug_raw_outputs(TfLiteTensor* output) const;
  void debug_qat_model_output() const;

  // Parameter Sweeping / Testing
  std::vector<ModelConfig> generate_debug_configs() const;
  void test_configuration(const ModelConfig& config, 
                          const std::vector<std::vector<uint8_t>>& zone_data,
                          std::vector<ConfigTestResult>& results);
  void debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data);
  bool invoke_model(const uint8_t* data, size_t len); // Helper for tests
  
  static void feed_watchdog();
  void set_debug(bool debug) { debug_ = debug; }

 private:
  const tflite::Model *tflite_model_{nullptr};
  std::unique_ptr<tflite::MicroInterpreter> interpreter_;
  ModelConfig config_;
  int output_size_{0};
  
  // Memory management
  MemoryManager memory_manager_;
  MemoryManager::AllocationResult tensor_arena_allocation_;
  size_t tensor_arena_size_requested_{0};
  size_t model_length_{0};
  
  std::unique_ptr<tflite::MicroMutableOpResolver<MAX_OPERATORS>> resolver_;
  bool debug_{false};
};

}  // namespace tflite_micro_helper
}  // namespace esphome
