#pragma once

#include "model_handler.h"
#include "memory_manager.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tflite_micro_helper {

class TFLiteMicroHelper {
 public:
  TFLiteMicroHelper() = default;
  ~TFLiteMicroHelper() = default;

  /**
   * @brief Load the model and allocate memory.
   * 
   * @param model_data Pointer to the model data.
   * @param model_size Size of the model data.
   * @param config Model configuration.
   * @return true if successful, false otherwise.
   */
  bool load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config);

  /**
   * @brief Invoke the interpreter.
   * 
   * @return TfLiteStatus
   */
  TfLiteStatus invoke() {
    return model_handler_.invoke();
  }

  /**
   * @brief Get the input tensor.
   * 
   * @return TfLiteTensor*
   */
  TfLiteTensor* input_tensor() const {
    return model_handler_.input_tensor();
  }

  /**
   * @brief Get the output tensor.
   * 
   * @return TfLiteTensor*
   */
  TfLiteTensor* output_tensor() const {
    return model_handler_.output_tensor();
  }

  /**
   * @brief Process the output tensor.
   * 
   * @param output_data Pointer to the output data.
   * @return ProcessedOutput
   */
  ProcessedOutput process_output(const float *output_data) const {
    return model_handler_.process_output(output_data);
  }

  // Accessors for model dimensions
  int get_input_width() const { return model_handler_.get_input_width(); }
  int get_input_height() const { return model_handler_.get_input_height(); }
  int get_input_channels() const { return model_handler_.get_input_channels(); }
  const ModelConfig& get_config() const { return model_handler_.get_config(); }

  // Memory usage reporting
  size_t get_arena_used_bytes() const { return model_handler_.get_arena_used_bytes(); }
  size_t get_tensor_arena_size() const { return tensor_arena_allocation_.actual_size; }
  
  void report_memory_status() {
    memory_manager_.report_memory_status(
        tensor_arena_size_requested_,
        tensor_arena_allocation_.actual_size,
        model_handler_.get_arena_used_bytes(),
        model_length_
    );
  }

 private:
  ModelHandler model_handler_;
  MemoryManager memory_manager_;
  MemoryManager::AllocationResult tensor_arena_allocation_;
  size_t tensor_arena_size_requested_{0};
  size_t model_length_{0};
};

}  // namespace tflite_micro_helper
}  // namespace esphome
