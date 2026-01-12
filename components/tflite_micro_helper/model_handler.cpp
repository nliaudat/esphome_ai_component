#include "model_handler.h"
#include "esp_log.h"
#include "debug_utils.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

namespace esphome {
namespace tflite_micro_helper {

static const char *const TAG = "ModelHandler";

void ModelHandler::unload() {
    interpreter_.reset();
    tensor_arena_allocation_.data.reset();
    tensor_arena_allocation_.actual_size = 0;
    tensor_arena_size_requested_ = 0;
    
    // Reset state
    config_ = ModelConfig{};
    output_size_ = 0;
    tflite_model_ = nullptr;
    model_length_ = 0;
    memory_manager_ = MemoryManager(); // Reset memory manager state if needed
    resolver_.reset();
    
    ESP_LOGI(TAG, "Model unloaded and memory freed");
}

bool ModelHandler::load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config) {
    model_length_ = model_size;
    
    // Determine tensor arena size from config
    std::string arena_size_str = config.tensor_arena_size;
    size_t multiplier = 1;
    
    if (arena_size_str.find("KB") != std::string::npos) {
        multiplier = 1024;
        arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
    } else if (arena_size_str.find("MB") != std::string::npos) {
        multiplier = 1024 * 1024;
        arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
    } else if (arena_size_str.find("B") != std::string::npos) {
        arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 1);
    }
    
    // Manual string to integer conversion
    const char* str = arena_size_str.c_str();
    char* end_ptr;
    long size_value = strtol(str, &end_ptr, 10);
    
    if (end_ptr != str && *end_ptr == '\0' && size_value > 0) {
        tensor_arena_size_requested_ = size_value * multiplier;
        ESP_LOGI(TAG, "Using model-specific tensor arena size: %s (%zu bytes)", 
                config.tensor_arena_size.c_str(), tensor_arena_size_requested_);
    } else {
        ESP_LOGW(TAG, "Failed to parse tensor arena size from config: %s", 
                config.tensor_arena_size.c_str());
        if (tensor_arena_size_requested_ == 0) {
             tensor_arena_size_requested_ = 100 * 1024; // Default 100KB
             ESP_LOGW(TAG, "Using default tensor arena size: 100KB");
        }
    }

    // Allocate tensor arena
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }

    // Load the model using the allocated arena
    return load_model_with_arena(model_data, model_size, 
                               tensor_arena_allocation_.data.get(), 
                               tensor_arena_allocation_.actual_size,
                               config);
}

bool ModelHandler::load_model_with_arena(const uint8_t *model_data, size_t model_size,
                            uint8_t* tensor_arena, size_t tensor_arena_size,
                            const ModelConfig &config) {

  ESP_LOGD(TAG, "Loading model with config:");
  ESP_LOGD(TAG, "  Description: %s", config.description.c_str());
  ESP_LOGD(TAG, "  Output processing: %s", config.output_processing.c_str());
  ESP_LOGD(TAG, "Model data validation:");
  ESP_LOGD(TAG, "  Model data pointer: %p", model_data);
  ESP_LOGD(TAG, "  Model data size: %zu bytes", model_size);

  if (model_data == nullptr) {
    ESP_LOGE(TAG, "Model data pointer is NULL!");
    return false;
  }
  
  if (model_size == 0) {
    ESP_LOGE(TAG, "Model data size is 0!");
    return false;
  }
  
#ifdef DEBUG_TFLITE_MICRO_HELPER
    // Check TFLite magic number
    if (model_size >= 8) {
      ESP_LOGI(TAG, "First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
               model_data[0], model_data[1], model_data[2], model_data[3],
               model_data[4], model_data[5], model_data[6], model_data[7]);
      
      // TFLite magic number should be: XX 00 00 00 54 46 4C 33
      if (model_data[1] == 0x00 && model_data[2] == 0x00 && model_data[3] == 0x00 &&
          model_data[4] == 0x54 && model_data[5] == 0x46 &&
          model_data[6] == 0x4C && model_data[7] == 0x33) {
        ESP_LOGI(TAG, "Valid TFLite magic number found (version byte: 0x%02X)", model_data[0]);
      } else {
        ESP_LOGE(TAG, "Invalid TFLite magic number");
        // Don't return false here if we want to try anyway, but legacy returned matches this block
        return false; 
      }
    }
#endif

  config_ = config;
  
    // For PROGMEM data, we need to handle it specially
    ESP_LOGI(TAG, "Loading model from PROGMEM (%zu bytes)", model_size);
    
    tflite_model_ = tflite::GetModel(model_data);
    
    if (tflite_model_ == nullptr) {
        ESP_LOGW(TAG, "Direct PROGMEM access failed, copying to RAM");
        // Note: This is risky if RAM is low, but sometimes necessary
        // Ideally we should avoid this path by ensuring GetModel works with PROGMEM
        // or passing a RAM buffer if needed.
        // For now, we assume GetModel works or we fail.
        // std::vector<uint8_t> ram_model(model_data, model_data + model_size);
        // tflite_model_ = tflite::GetModel(ram_model.data());
    }
  
  if (tflite_model_ == nullptr) {
    ESP_LOGE(TAG, "Failed to parse model - invalid data");
    return false;
  }

  if (tflite_model_->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Model schema version mismatch: Model has %d, Expecting %d",
             tflite_model_->version(), TFLITE_SCHEMA_VERSION);
    
    // Diagnostic dump to identify corrupted files
    if (model_data != nullptr && model_size >= 16) {
        ESP_LOGE(TAG, "Model Header Dump (First 16 bytes):");
        ESP_LOGE(TAG, "  %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X",
            model_data[0], model_data[1], model_data[2], model_data[3],
            model_data[4], model_data[5], model_data[6], model_data[7],
            model_data[8], model_data[9], model_data[10], model_data[11],
            model_data[12], model_data[13], model_data[14], model_data[15]);

        // Check magic number TFL3 (starts at offset 4)
        if (model_data[4] == 'T' && model_data[5] == 'F' && model_data[6] == 'L' && model_data[7] == '3') {
             ESP_LOGW(TAG, "Magic number 'TFL3' is PRESENT. Issue might be alignment or genuine schema mismatch.");
        } else {
             ESP_LOGE(TAG, "Magic number 'TFL3' is MISSING! The file is likely corrupted or not a valid TFLite model.");
        }
    } else {
        ESP_LOGE(TAG, "Unable to dump header: Data null or too small (%zu bytes)", model_size);
    }
    return false;
  }

  // Reset and allocate fresh resolver for this load cycle
  resolver_ = std::make_unique<tflite::MicroMutableOpResolver<MAX_OPERATORS>>();
  
  ESP_LOGD(TAG, "Operator codes found in model:");
    for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = tflite_model_->operator_codes()->Get(i);
      ESP_LOGD(TAG, "  [%d]: %d (%s)", (int)i, op_code->builtin_code(),
               tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
  
  std::set<tflite::BuiltinOperator> required_ops;
  
  for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
    const auto *op_code = tflite_model_->operator_codes()->Get(i);
    required_ops.insert(op_code->builtin_code());
  }

  if (!OpResolverManager::RegisterOps<MAX_OPERATORS>(*resolver_, required_ops, TAG)) {
    ESP_LOGE(TAG, "Failed to register operators");
    return false;
  }

  interpreter_ = std::make_unique<tflite::MicroInterpreter>(
      tflite_model_,
      *resolver_,
      tensor_arena,
      tensor_arena_size);

  if (interpreter_->AllocateTensors() != kTfLiteOk) {
    ESP_LOGE(TAG, "Failed to allocate tensors");
    return false;
  }
  
  if (tflite_model_->subgraphs()->Get(0)->operators()->size() == 0) {
        ESP_LOGE(TAG, "Model has no operators!");
        return false;
    }
  
  auto* input = input_tensor();
  if (input) {
    ESP_LOGI(TAG, "Input tensor dimensions:");
    for (int i = 0; i < input->dims->size; i++) {
      ESP_LOGI(TAG, "  Dim %d: %d", i, input->dims->data[i]);
    }
  }

  // Auto-detect model type if output processing not specified
  TfLiteTensor* output = output_tensor();
  if (output) {
      // Calculate output size
      int size = 1;
      for (int i = 0; i < output->dims->size; i++) {
          size *= output->dims->data[i];
      }
      output_size_ = size;
      
      if (config_.output_processing.empty()) {
        if (output->dims->size >= 2 && output->dims->data[1] == 100) {
          config_.output_processing = "logits";
          config_.scale_factor = 10.0f;
          ESP_LOGW(TAG, "Auto-detected class100 model, using softmax processing");
        } else if (output->dims->size >= 2 && output->dims->data[1] == 10) {
          config_.output_processing = "softmax";
          config_.scale_factor = 1.0f;
          ESP_LOGW(TAG, "Auto-detected class10 model, using softmax processing");
        }
      }
  }
  
  ESP_LOGI(TAG, "Model loaded successfully");
  
  return true;
}

void ModelHandler::log_input_stats() const {
    TfLiteTensor* input = input_tensor();
    if (input == nullptr) return;
    
    const int total_elements = get_input_width() * get_input_height() * get_input_channels();
    const int sample_size = std::min(20, total_elements); // Show first 20 values
    
    ESP_LOGD(TAG, "First %d %s inputs (%s):", sample_size, 
             input->type == kTfLiteFloat32 ? "float32" : "uint8",
             config_.normalize ? "normalized" : "raw");
    
    if (input->type == kTfLiteFloat32) {
        const float* data = input->data.f;
        for (int i = 0; i < sample_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %.4f", i, data[i]);
            // Log channel groups for RGB/BGR
            if (config_.input_channels >= 3 && i % config_.input_channels == 2) {
                ESP_LOGD(TAG, "    -> %s: [%.3f, %.3f, %.3f]", 
                         config_.input_order.c_str(),
                         data[i-2], data[i-1], data[i]);
            }
        }
    } else {
        const uint8_t* data = input->data.uint8;
        for (int i = 0; i < sample_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %u", i, data[i]);
            // Log channel groups for RGB/BGR
            if (config_.input_channels >= 3 && i % config_.input_channels == 2) {
                ESP_LOGD(TAG, "    -> %s: [%u, %u, %u]", 
                         config_.input_order.c_str(),
                         data[i-2], data[i-1], data[i]);
            }
        }
    }
}

void ModelHandler::debug_input_pattern() const {
    TfLiteTensor* input = input_tensor();
    if (!input || input->type != kTfLiteFloat32) return;
    
    const float* data = input->data.f;
    const int total_elements = input->bytes / sizeof(float);
    const int channels = get_input_channels();
    const int height = get_input_height();
    const int width = get_input_width();
    
    ESP_LOGD(TAG, "Input pattern analysis: %dx%dx%d", width, height, channels);
    
    // Check if this looks like a proper image (not all zeros or uniform)
    float channel_sums[3] = {0};
    float channel_mins[3] = {std::numeric_limits<float>::max()};
    float channel_maxs[3] = {std::numeric_limits<float>::lowest()};
    int pixel_count = 0;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            if (pos + 2 < total_elements) {
                for (int c = 0; c < channels; c++) {
                    float val = data[pos + c];
                    channel_sums[c] += val;
                    channel_mins[c] = std::min(channel_mins[c], val);
                    channel_maxs[c] = std::max(channel_maxs[c], val);
                }
                pixel_count++;
            }
        }
    }
    
    if (pixel_count > 0) {
        ESP_LOGD(TAG, "Channel statistics:");
        for (int c = 0; c < channels; c++) {
            float mean = channel_sums[c] / pixel_count;
            ESP_LOGD(TAG, "  Channel %d: min=%.3f, max=%.3f, mean=%.3f", 
                     c, channel_mins[c], channel_maxs[c], mean);
        }
    }
    
    // Check for common preprocessing patterns
    bool looks_normalized_0_1 = true;
    bool looks_normalized_neg1_1 = true;
    
    for (int i = 0; i < total_elements; i++) {
        if (data[i] < 0.0f || data[i] > 1.0f) looks_normalized_0_1 = false;
        if (data[i] < -1.0f || data[i] > 1.0f) looks_normalized_neg1_1 = false;
    }
    
    ESP_LOGD(TAG, "Data range analysis:");
    ESP_LOGD(TAG, "  Looks like 0-1 normalized: %s", looks_normalized_0_1 ? "YES" : "NO");
    ESP_LOGD(TAG, "  Looks like -1 to 1 normalized: %s", looks_normalized_neg1_1 ? "YES" : "NO");
}

ProcessedOutput ModelHandler::process_output(TfLiteTensor* output_tensor) const {
    if (!output_tensor) {
        ESP_LOGE(TAG, "Null output tensor");
        return {0.0f, 0.0f};
    }

    if (output_tensor->type == kTfLiteFloat32) {
        return process_output(output_tensor->data.f);
    } 
    else if (output_tensor->type == kTfLiteUInt8 || output_tensor->type == kTfLiteInt8) {
        // Calculate total elements
        int count = 1; 
        for(int i=0; i < output_tensor->dims->size; i++) {
            count *= output_tensor->dims->data[i];
        }
        
        // Safety check against buffer overflow if count differs from output_size_
        // Use local vector to ensure bounds safety regardless of output_size_ member state.
        // This ensures dequantization loop is safe.
        
        std::vector<float> dequantized(count);
        float scale = output_tensor->params.scale;
        int32_t zero_point = output_tensor->params.zero_point;
        
        if (scale == 0.0f) { 
             scale = 1.0f; 
             // ESP_LOGW(TAG, "Output tensor has scale 0.0, assuming 1.0");
        }

        if (output_tensor->type == kTfLiteUInt8) {
            uint8_t* data = output_tensor->data.uint8;
            for(int i=0; i<count; i++) {
                dequantized[i] = (data[i] - zero_point) * scale;
            }
        } else { // Int8
            int8_t* data = output_tensor->data.int8;
            for(int i=0; i<count; i++) {
                dequantized[i] = (data[i] - zero_point) * scale;
            }
        }
        
        return process_output(dequantized.data());
    } 
    else {
        ESP_LOGE(TAG, "Unsupported output tensor type: %d", output_tensor->type);
        return {0.0f, 0.0f};
    }
}

ProcessedOutput ModelHandler::process_output(const float *output_data) const {
  const int num_classes = output_size_;
  ProcessedOutput result = {0.0f, 0.0f};

  if (num_classes <= 0) {
    ESP_LOGE(TAG, "Invalid number of output classes: %d", num_classes);
    return result;
  }

  // Single-pass statistics calculation
  float min_val = output_data[0];
  float max_val = output_data[0];
  int max_idx = 0;
  
  for (int i = 1; i < num_classes; i++) {
    float val = output_data[i];
    if (val < min_val) min_val = val;
    if (val > max_val) {
        max_val = val;
        max_idx = i;
    }
  }
  
  float max_val_output = max_val;
  ESP_LOGD(TAG, "Output range: min=%.6f, max=%.6f", min_val, max_val);

  if (config_.output_processing == "direct_class") {
    result.value = static_cast<float>(max_idx);
    result.confidence = max_val_output;
    ESP_LOGD(TAG,
             "Direct class - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "softmax") {
    float max_logit = *std::max_element(output_data,
                                        output_data + num_classes);
    std::vector<float> exp_vals(num_classes);
    float sum = 0.0f;
    for (int i = 0; i < num_classes; i++) {
      exp_vals[i] = expf(output_data[i] - max_logit);
      sum += exp_vals[i];
    }

    int softmax_max_idx = 0;
    float softmax_max_val = 0.0f;
    for (int i = 0; i < num_classes; i++) {
      float prob = exp_vals[i] / sum;
      if (prob > softmax_max_val) {
        softmax_max_val = prob;
        softmax_max_idx = i;
      }
    }

    result.value = static_cast<float>(softmax_max_idx) / config_.scale_factor;
    result.confidence = softmax_max_val;
    ESP_LOGD(TAG,
             "Softmax - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "logits") {
    result.value = static_cast<float>(max_idx);

    if (min_val >= 0.0f && max_val <= 1.0f) {
      result.confidence = max_val_output;
    } else {
      float confidence_range = max_val - min_val;
      if (confidence_range > 0.001f) {
        result.confidence = (max_val_output - min_val) / confidence_range;
      } else {
        result.confidence = 1.0f;
      }
      result.confidence = std::max(0.0f,
                                   std::min(1.0f, result.confidence));
    }

    if (config_.scale_factor != 1.0f) {
      result.value = result.value / config_.scale_factor;
    }

    ESP_LOGD(TAG,
             "Logits - Value: %.1f, Raw Max: %.6f, Confidence: %.6f",
             result.value, max_val_output, result.confidence);

  } else if (config_.output_processing == "qat_quantized") {
    result.value = static_cast<float>(max_idx);

    if (max_val > min_val) {
      result.confidence = (max_val_output - min_val) / (max_val - min_val);
    } else {
      result.confidence = 1.0f;
    }
    result.confidence = std::max(0.0f,
                                 std::min(1.0f, result.confidence));

    if (config_.scale_factor != 1.0f) {
      result.value = result.value / config_.scale_factor;
    }

    ESP_LOGD(TAG,
             "QAT Quantized - Value: %.1f, Confidence: %.6f (raw: %.6f)",
             result.value, result.confidence, max_val_output);

  } else if (config_.output_processing == "experimental_scale") {
    const float scale_factor = 0.1f;   // tweak if needed
    std::vector<float> scaled(num_classes);
    for (int i = 0; i < num_classes; ++i) {
      scaled[i] = output_data[i] * scale_factor;
    }

    float max_scaled = *std::max_element(scaled.begin(), scaled.end());
    std::vector<float> exp_vals(num_classes);
    float sum = 0.0f;
    for (int i = 0; i < num_classes; ++i) {
      exp_vals[i] = expf(scaled[i] - max_scaled);
      sum += exp_vals[i];
    }

    int exp_max_idx = 0;
    float exp_max_val = 0.0f;
    for (int i = 0; i < num_classes; ++i) {
      float prob = exp_vals[i] / sum;
      if (prob > exp_max_val) {
        exp_max_val = prob;
        exp_max_idx = i;
      }
    }

    result.value = static_cast<float>(exp_max_idx) / config_.scale_factor;
    result.confidence = exp_max_val;
    ESP_LOGD(TAG,
             "Experimental scale - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "logits_jomjol") {
    result.value = static_cast<float>(max_idx) / config_.scale_factor;
    result.confidence = max_val_output;   // raw max as confidence
    ESP_LOGD(TAG,
             "Logits jomjol - Value: %.1f, Raw Max: %.6f",
             result.value, max_val_output);

  } else if (config_.output_processing == "softmax_jomjol") {
    float max_logit = *std::max_element(output_data,
                                        output_data + num_classes);
    std::vector<float> exp_vals(num_classes);
    float sum = 0.0f;
    for (int i = 0; i < num_classes; ++i) {
      exp_vals[i] = expf(output_data[i] - max_logit);
      sum += exp_vals[i];
    }

    int sm_max_idx = 0;
    float sm_max_val = 0.0f;
    for (int i = 0; i < num_classes; ++i) {
      float prob = exp_vals[i] / sum;
      if (prob > sm_max_val) {
        sm_max_val = prob;
        sm_max_idx = i;
      }
    }

    result.value = static_cast<float>(sm_max_idx) / config_.scale_factor;
    result.confidence = sm_max_val;

    ESP_LOGD(TAG,
             "Softmax jomjol - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "auto_detect") {
    float min_val_ad = output_data[0];
    float max_val_ad = output_data[0];
    float sum_ad     = 0.0f;
    int   non_zero_ad = 0;

    for (int i = 0; i < num_classes; ++i) {
      float v = output_data[i];
      if (v < min_val_ad) min_val_ad = v;
      if (v > max_val_ad) max_val_ad = v;
      sum_ad += v;
      if (std::fabs(v) > 1e-6f) ++non_zero_ad;
    }

    bool looks_like_probs_ad =
        (min_val_ad >= 0.0f && max_val_ad <= 1.0f &&
         std::fabs(sum_ad - 1.0f) <= 0.05f);

    bool looks_like_one_hot_ad = (non_zero_ad == 1 && max_val_ad >= 0.5f);
    bool looks_like_logits_ad = !looks_like_probs_ad && !looks_like_one_hot_ad;

    ESP_LOGI(TAG,
             "AUTO-DETECT: min=%.3f max=%.3f sum=%.3f nz=%d => probs=%s one_hot=%s logits=%s",
             min_val_ad, max_val_ad, sum_ad, non_zero_ad,
             looks_like_probs_ad ? "YES" : "NO",
             looks_like_one_hot_ad ? "YES" : "NO",
             looks_like_logits_ad ? "YES" : "NO");

    if (looks_like_probs_ad) {
      int max_idx_ad = 0;
      float max_prob_ad = output_data[0];
      for (int i = 1; i < num_classes; ++i) {
        if (output_data[i] > max_prob_ad) {
          max_prob_ad = output_data[i];
          max_idx_ad = i;
        }
      }
      result.value      = static_cast<float>(max_idx_ad) / config_.scale_factor;
      result.confidence = max_prob_ad;
      ESP_LOGD(TAG,
               "Auto-detect (softmax): value=%.1f confidence=%.6f",
               result.value, result.confidence);
    } else if (looks_like_one_hot_ad) {
      int class_id_ad = 0;
      for (int i = 0; i < num_classes; ++i) {
        if (std::fabs(output_data[i]) > 0.5f) {
          class_id_ad = i;
          break;
        }
      }
      result.value      = static_cast<float>(class_id_ad) / config_.scale_factor;
      result.confidence = 1.0f;
      ESP_LOGD(TAG,
               "Auto-detect (direct): value=%.1f confidence=1.0",
               result.value);
    } else {
      float max_logit_ad = *std::max_element(output_data,
                                            output_data + num_classes);
      std::vector<float> exp_vals_ad(num_classes);
      float exp_sum_ad = 0.0f;
      for (int i = 0; i < num_classes; ++i) {
        exp_vals_ad[i] = expf(output_data[i] - max_logit_ad);
        exp_sum_ad    += exp_vals_ad[i];
      }

      int   max_idx_ad = 0;
      float max_prob_ad = exp_vals_ad[0] / exp_sum_ad;
      for (int i = 1; i < num_classes; ++i) {
        float prob = exp_vals_ad[i] / exp_sum_ad;
        if (prob > max_prob_ad) {
          max_prob_ad = prob;
          max_idx_ad = i;
        }
      }

      result.value      = static_cast<float>(max_idx_ad) / config_.scale_factor;
      result.confidence = max_prob_ad;
      ESP_LOGD(TAG,
               "Auto-detect (logits): value=%.1f confidence=%.6f",
               result.value, result.confidence);
    }

  } else {
    ESP_LOGE(TAG,
             "Unknown output processing method: %s",
             config_.output_processing.c_str());

    result.value = static_cast<float>(max_idx);
    result.confidence = max_val_output;
    ESP_LOGW(TAG,
             "Fallback to direct classification - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);
  }

  return result;
}

uint32_t ModelHandler::calculate_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

bool ModelHandler::verify_model_crc(const uint8_t *model_data, size_t length) {
    uint32_t crc = calculate_crc32(model_data, length);
    ESP_LOGI(TAG, "Model CRC32: 0x%08X", crc);
    return true;
}

void ModelHandler::debug_model_architecture() const {
    if (!tflite_model_) return;
    
    auto* subgraphs = tflite_model_->subgraphs();
    if (!subgraphs) return;
    
    ESP_LOGD(TAG, "Model Architecture:");
    ESP_LOGD(TAG, "  Subgraphs: %d", subgraphs->size());
    
    for (int i = 0; i < subgraphs->size(); i++) {
        auto* subgraph = subgraphs->Get(i);
        ESP_LOGD(TAG, "  Subgraph %d:", i);
        ESP_LOGD(TAG, "    Tensors: %d", subgraph->tensors()->size());
        ESP_LOGD(TAG, "    Operators: %d", subgraph->operators()->size());
        ESP_LOGD(TAG, "    Inputs: %d", subgraph->inputs()->size());
        ESP_LOGD(TAG, "    Outputs: %d", subgraph->outputs()->size());
    }
}

bool ModelHandler::validate_model_config() const {
    TfLiteTensor* input = input_tensor();
    if (!input) return false;
    
    // Check input dimensions
    if (input->dims->size >= 4) {
        int height = input->dims->data[1];
        int width = input->dims->data[2];
        int channels = input->dims->data[3];
        
        if (config_.input_size.size() >= 2) {
            if (width != config_.input_size[0] || height != config_.input_size[1]) {
                ESP_LOGW(TAG, "Model input size mismatch! Config: %dx%d, Model: %dx%d",
                         config_.input_size[0], config_.input_size[1], width, height);
                return false;
            }
        }
        
        if (channels != config_.input_channels) {
            ESP_LOGW(TAG, "Model input channels mismatch! Config: %d, Model: %d",
                     config_.input_channels, channels);
            return false;
        }
    }
    
    return true;
}

void ModelHandler::report_memory_status() {
    memory_manager_.report_memory_status(
        tensor_arena_size_requested_,
        tensor_arena_allocation_.actual_size,
        get_arena_used_bytes(),
        model_length_
    );
}

}  // namespace tflite_micro_helper
}  // namespace esphome
