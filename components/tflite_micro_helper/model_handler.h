#pragma once

#include <memory>
#include <string>
#include <vector>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "op_resolver.h"

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

class ModelHandler {
 public:
  bool load_model(const uint8_t *model_data, size_t model_size,
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
  
  const ModelConfig& get_config() const { return config_; }
  
  size_t get_arena_used_bytes() const {
    return interpreter_->arena_used_bytes();
  }
  
  // Debug helpers
  void log_input_stats() const;
  void debug_input_pattern() const;
  
  // CRC32 verification
  static uint32_t calculate_crc32(const uint8_t *data, size_t length);
  bool verify_model_crc(const uint8_t *model_data, size_t length);
  void debug_model_architecture() const;
  bool validate_model_config() const;

 private:
  const tflite::Model *tflite_model_{nullptr};
  std::unique_ptr<tflite::MicroInterpreter> interpreter_;
  ModelConfig config_;
  int output_size_{0};
};

}  // namespace tflite_micro_helper
}  // namespace esphome
