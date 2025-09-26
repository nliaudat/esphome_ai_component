#pragma once

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "op_resolver.h"
#include <memory>
#include <string>
#include <vector>

// #ifdef DEBUG_METER_READER_TFLITE
#include <esp_task_wdt.h>
#include <cstdint>   // for crc check uint32_t
#include <inttypes.h> // for crc check PRIu32 formatting
// #endif





namespace esphome {
namespace meter_reader_tflite {
    
    
// for model_data.h (not progmem way)
// extern const uint8_t model_data[];
// extern const unsigned int model_data_len;

uint32_t crc32_runtime(const uint8_t* data, size_t length);

struct ModelConfig {
  std::string description;
  std::string tensor_arena_size;
  std::string output_processing; 
  float scale_factor;
  std::string input_type;
  int input_channels = 3;
  std::string input_order; // "BGR" or "RGB"
  std::pair<int, int> input_size = {0, 0};
  bool normalize = false;
  bool invert = false;
};

struct ProcessedOutput {
  float value;
  float confidence;
};

// #ifdef DEBUG_METER_READER_TFLITE
struct ConfigTestResult {
    ModelConfig config;
    float avg_confidence;
    std::vector<float> zone_confidences;
    std::vector<float> zone_values;
};


// #endif

constexpr size_t MAX_OPERATORS = 30;

class ModelHandler {
 public:
  bool load_model(const uint8_t *model_data, size_t model_size, 
                uint8_t* tensor_arena, size_t tensor_arena_size,
                const ModelConfig &config);
                

  bool invoke_model(const uint8_t* input_data, size_t input_size);
  const float* get_output() const { return model_output_; }
  int get_output_size() const { return output_size_; }
                  
  size_t get_arena_peak_bytes() const;
  
  TfLiteTensor* input_tensor() const;
  TfLiteTensor* output_tensor() const;
  
  ProcessedOutput get_processed_output() const { return processed_output_; } 
  
  int get_input_width() const {
    if (!interpreter_ || !input_tensor()) return 0;
    if (input_tensor()->dims->size == 4) {
      return input_tensor()->dims->data[2];  // [batch, height, width, channels]
    } else if (input_tensor()->dims->size == 3) {
      return input_tensor()->dims->data[1];  // [height, width, channels]
    }
    return 0;
  }

  int get_input_height() const {
    if (!interpreter_ || !input_tensor()) return 0;
    if (input_tensor()->dims->size == 4) {
      return input_tensor()->dims->data[1];  // [batch, height, width, channels]
    } else if (input_tensor()->dims->size == 3) {
      return input_tensor()->dims->data[0];  // [height, width, channels]
    }
    return 0;
  }

  int get_input_channels() const {
    if (!interpreter_ || !input_tensor()) return 0;
    if (input_tensor()->dims->size == 4) {
      return input_tensor()->dims->data[3];  // [batch, height, width, channels]
    } else if (input_tensor()->dims->size == 3) {
      return input_tensor()->dims->data[2];  // [height, width, channels]
    }
    return 0;
  }

  const ModelConfig& get_config() const { return config_; }
  void set_config(const ModelConfig &config) { config_ = config; }
  
  bool is_model_quantized() const {
      return interpreter_ && input_tensor() && 
           input_tensor()->type == kTfLiteUInt8;
    }
    
  const uint8_t* get_quantized_output() const {
      return interpreter_ && output_tensor() ? 
           output_tensor()->data.uint8 : nullptr;
    }
    
  float get_output_scale() const {
      return interpreter_ && output_tensor() ? 
           output_tensor()->params.scale : 1.0f;
    }
    
  int get_output_zero_point() const {
      return interpreter_ && output_tensor() ? 
           output_tensor()->params.zero_point : 0;
    }

  void log_input_stats() const;
  void debug_input_pattern() const;
  void debug_model_architecture() const;
  
// #ifdef DEBUG_METER_READER_TFLITE
  void debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data);
  void debug_test_parameters(unsigned char*, size_t&);
  void test_configuration(const ModelConfig& config, 
                         const std::vector<std::vector<uint8_t>>& zone_data,
                         std::vector<ConfigTestResult>& results);
  void debug_test_with_pattern();
  std::vector<ModelConfig> generate_debug_configs() const; 
  void feed_watchdog();
  void verify_model_crc(const uint8_t* model_data, size_t model_size);
  // uint32_t crc32_runtime(const uint8_t* data, size_t length);
// #endif

 protected:
  ProcessedOutput process_output(const float* output_data) const;
  bool validate_model_config();

 private:
  const tflite::Model* tflite_model_{nullptr};
  std::unique_ptr<tflite::MicroInterpreter> interpreter_;
  ModelConfig config_;
  
  ProcessedOutput processed_output_ = {0.0f, 0.0f};
  const float* model_output_ = nullptr;
  int output_size_ = 0;
  mutable std::vector<float> dequantized_output_;
};

}  // namespace meter_reader_tflite
}  // namespace esphome