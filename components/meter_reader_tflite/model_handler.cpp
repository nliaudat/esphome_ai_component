#include "model_handler.h"
#include "esp_log.h"
#include "debug_utils.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

#ifdef DEBUG_METER_READER_TFLITE
#include <sstream>
#include <iomanip>
#endif

// test with direct model loading (no progmem)
// #include "model_data.h"


namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "ModelHandler";

bool ModelHandler::load_model(const uint8_t *model_data, size_t model_size,
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
  
  // if (model_size != model_data_len) {
    // ESP_LOGE(TAG, "Size mismatch! Passed size: %zu, Header size: %u", 
             // model_size, model_data_len);
    // return false;
  // }
  
  
#ifdef DEBUG_METER_READER_TFLITE
    // Check TFLite magic number
    if (model_size >= 8) {
      ESP_LOGI(TAG, "First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
               model_data[0], model_data[1], model_data[2], model_data[3],
               model_data[4], model_data[5], model_data[6], model_data[7]);
      
      // TFLite magic number should be: XX 00 00 00 54 46 4C 33
      // The first byte can vary (0x1C for older versions, 0x20 for newer)
      // but the last 4 bytes should always be 'T','F','L','3' in ASCII
      if (model_data[1] == 0x00 && model_data[2] == 0x00 && model_data[3] == 0x00 &&
          model_data[4] == 0x54 && model_data[5] == 0x46 &&
          model_data[6] == 0x4C && model_data[7] == 0x33) {
        ESP_LOGI(TAG, "Valid TFLite magic number found (version byte: 0x%02X)", model_data[0]);
      } else {
        ESP_LOGE(TAG, "Invalid TFLite magic number");
        return false;
      }
    }
#endif
  
  config_ = config;
  
    // For PROGMEM data, we need to handle it specially
    ESP_LOGI(TAG, "Loading model from PROGMEM (%zu bytes)", model_size);
    
    // Check if we need to copy from PROGMEM to RAM (if tflite can't handle PROGMEM directly)
    // Some TFLite implementations might require RAM-based data
    
    // Option 1: Try direct access (if TFLite supports PROGMEM)
    tflite_model_ = tflite::GetModel(model_data);
    
    // Option 2: If that fails, copy to RAM first
    if (tflite_model_ == nullptr) {
        ESP_LOGW(TAG, "Direct PROGMEM access failed, copying to RAM");
        std::vector<uint8_t> ram_model(model_data, model_data + model_size);
        tflite_model_ = tflite::GetModel(ram_model.data());
    }
  
  if (tflite_model_ == nullptr) {
    ESP_LOGE(TAG, "Failed to parse model - invalid data");
    
    // Additional debug: check if memory is accessible
    ESP_LOGI(TAG, "Testing memory access...");
    uint8_t test_sum = 0;
    for (size_t i = 0; i < std::min((size_t)100, model_size); i++) {
      test_sum += model_data[i];
    }
    ESP_LOGI(TAG, "Memory access test sum: %u", test_sum);
    
    return false;
  }

  if (tflite_model_->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Model schema version mismatch");
    return false;
  }

#ifdef DEBUG_METER_READER_TFLITE

    // Check basic model structure
    auto* subgraph = tflite_model_->subgraphs()->Get(0);
    ESP_LOGD(TAG, "Model subgraph tensors: %d", subgraph->tensors()->size());
    ESP_LOGD(TAG, "Model subgraph operators: %d", subgraph->operators()->size());


/*  
  ## manual mode 
  static tflite::MicroMutableOpResolver<11> resolver;

  TfLiteStatus status = kTfLiteOk;
  if (resolver.AddQuantize() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddMul() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddAdd() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddConv2D() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddMaxPool2D() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddReshape() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddFullyConnected() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddDequantize() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddRelu() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddSoftmax() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddLeakyRelu() != kTfLiteOk) status = kTfLiteError;
  if (resolver.AddMean() != kTfLiteOk) status = kTfLiteError;
  

  if (status != kTfLiteOk) {
    ESP_LOGE(TAG, "Failed to register one or more operations");
    return false;
  }

  ESP_LOGD(TAG, "All operations registered successfully");
 */
 
    // Use dynamic operator registration for debug mode too
    static tflite::MicroMutableOpResolver<MAX_OPERATORS> resolver;
    
    ESP_LOGD(TAG, "Operator codes found in model:");
    for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = tflite_model_->operator_codes()->Get(i);
      ESP_LOGD(TAG, "  [%d]: %d (%s)", i, op_code->builtin_code(),
               tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
  
    std::set<tflite::BuiltinOperator> required_ops;
    
    for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = tflite_model_->operator_codes()->Get(i);
      required_ops.insert(op_code->builtin_code());
    }

    if (!OpResolverManager::RegisterOps<MAX_OPERATORS>(resolver, required_ops, TAG)) {
      ESP_LOGE(TAG, "Failed to register operators");
      return false;
    }

    ESP_LOGD(TAG, "All operations registered successfully");
 
#else
  // Your existing dynamic registration
  static tflite::MicroMutableOpResolver<MAX_OPERATORS> resolver;
  
  ESP_LOGD(TAG, "Operator codes found in model:");
    for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = tflite_model_->operator_codes()->Get(i);
      ESP_LOGD(TAG, "  [%d]: %d (%s)", i, op_code->builtin_code(),
               tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
  
  std::set<tflite::BuiltinOperator> required_ops;
  
  for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
    const auto *op_code = tflite_model_->operator_codes()->Get(i);
    required_ops.insert(op_code->builtin_code());
  }

  if (!OpResolverManager::RegisterOps<MAX_OPERATORS>(resolver, required_ops, TAG)) {
    ESP_LOGE(TAG, "Failed to register operators");
    return false;
  }
#endif

  interpreter_ = std::make_unique<tflite::MicroInterpreter>(
      tflite_model_,
      resolver,
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

#ifdef DEBUG_METER_READER_TFLITE
  if (!validate_model_config()) {
    ESP_LOGE(TAG, "Model configuration validation failed");
    return false;
  }
#endif

  // Auto-detect model type if output processing not specified
  TfLiteTensor* output = output_tensor();
  if (output && config_.output_processing.empty()) {
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
  

  ESP_LOGI(TAG, "Model loaded successfully");
  
#ifdef DEBUG_METER_READER_TFLITE
  debug_model_architecture();   
  verify_model_crc(model_data, model_size);
#endif  
  
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



ProcessedOutput ModelHandler::process_output(const float *output_data) const {
  const int num_classes = output_size_;
  ProcessedOutput result = {0.0f, 0.0f};

  if (num_classes <= 0) {
    ESP_LOGE(TAG, "Invalid number of output classes: %d", num_classes);
    return result;
  }

  /* -----------------------------------------------------------------------
   *  DEBUG: raw model output before any processing
   * ----------------------------------------------------------------------- */
#ifdef DEBUG_METER_READER_TFLITE
  ESP_LOGD(TAG, "Raw model outputs before any processing:");
  for (int i = 0; i < num_classes; i++) {
    ESP_LOGD(TAG, "  Class %d: %.6f", i, output_data[i]);
  }
#endif

  /* -----------------------------------------------------------------------
   *  Debug: output range
   * ----------------------------------------------------------------------- */
  float min_val = *std::min_element(output_data, output_data + num_classes);
  float max_val = *std::max_element(output_data, output_data + num_classes);
  ESP_LOGD(TAG, "Output range: min=%.6f, max=%.6f", min_val, max_val);

  /* -----------------------------------------------------------------------
   *  Find the index of the maximum value (used by several branches)
   * ----------------------------------------------------------------------- */
  int max_idx = 0;
  float max_val_output = output_data[0];
  for (int i = 1; i < num_classes; i++) {
    if (output_data[i] > max_val_output) {
      max_val_output = output_data[i];
      max_idx = i;
    }
  }

  /* -----------------------------------------------------------------------
   *  Process according to the selected output_processing mode
   * ----------------------------------------------------------------------- */
  if (config_.output_processing == "direct_class") {
    /* ---------------------------------------------------------------
     *  Direct class – the tensor already contains the class index.
     * --------------------------------------------------------------- */
    result.value = static_cast<float>(max_idx);
    result.confidence = max_val_output;
    ESP_LOGD(TAG,
             "Direct class - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "softmax") {
    /* ---------------------------------------------------------------
     *  Soft-max – compute probabilities from raw logits.
     * --------------------------------------------------------------- */
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
    /* ---------------------------------------------------------------
     *  Logits – raw scores, need a confidence derived from the range.
     * --------------------------------------------------------------- */
    result.value = static_cast<float>(max_idx);

    /* ----- Better confidence calculation for logits ----- */
    if (min_val >= 0.0f && max_val <= 1.0f) {
      /* Values already in 0-1 range (behave like probabilities). */
      result.confidence = max_val_output;
    } else {
      /* Wider range – normalise to 0-1. */
      float confidence_range = max_val - min_val;
      if (confidence_range > 0.001f) {
        result.confidence = (max_val_output - min_val) / confidence_range;
      } else {
        result.confidence = 1.0f;
      }
      result.confidence = std::max(0.0f,
                                   std::min(1.0f, result.confidence));
    }

    /* Apply optional scale factor (used for 100-class models). */
    if (config_.scale_factor != 1.0f) {
      result.value = result.value / config_.scale_factor;
    }

    ESP_LOGD(TAG,
             "Logits - Value: %.1f, Raw Max: %.6f, Confidence: %.6f",
             result.value, max_val_output, result.confidence);

  } else if (config_.output_processing == "qat_quantized") {
    /* ---------------------------------------------------------------
     *  QAT-quantized – de-quantised output, treat similarly to logits
     *  but clamp confidence to [0,1].
     * --------------------------------------------------------------- */
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
    /* ---------------------------------------------------------------
     *  Experimental – scale logits before soft-max (helps very negative
     *  values).  The scale factor can be tuned per model.
     * --------------------------------------------------------------- */
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
    /* ---------------------------------------------------------------
     *  Logits Jomjol – same as “logits” but confidence is the raw max.
     * --------------------------------------------------------------- */
    result.value = static_cast<float>(max_idx) / config_.scale_factor;
    result.confidence = max_val_output;   // raw max as confidence
    ESP_LOGD(TAG,
             "Logits jomjol - Value: %.1f, Raw Max: %.6f",
             result.value, max_val_output);

  } else if (config_.output_processing == "softmax_jomjol") {
    /* ---------------------------------------------------------------
     *  Softmax Jomjol – replicate the Python script: always apply soft-max,
     *  then take the arg-max.
     * --------------------------------------------------------------- */
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

#ifdef DEBUG_METER_READER_TFLITE
    ESP_LOGD(TAG, "Softmax jomjol probabilities:");
    for (int i = 0; i < num_classes; ++i) {
      float prob = exp_vals[i] / sum;
      ESP_LOGD(TAG, "  Class %d: %.6f", i, prob);
    }
#endif

    ESP_LOGD(TAG,
             "Softmax jomjol - Value: %.1f, Confidence: %.6f",
             result.value, result.confidence);

  } else if (config_.output_processing == "auto_detect") {
    /* ---------------------------------------------------------------
     *  AUTO-DETECT – decide at run-time whether the tensor already
     *  contains probabilities, a one-hot class index, or raw logits.
     * --------------------------------------------------------------- */
    // ---- gather basic statistics ------------------------------------------------
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

    // ---- heuristic 1: already-normalised probabilities -------------------------
    bool looks_like_probs_ad =
        (min_val_ad >= 0.0f && max_val_ad <= 1.0f &&
         std::fabs(sum_ad - 1.0f) <= 0.05f);

    // ---- heuristic 2: one-hot class index ------------------------------------
    bool looks_like_one_hot_ad = (non_zero_ad == 1 && max_val_ad >= 0.5f);

    // ---- heuristic 3: treat everything else as logits -------------------------
    bool looks_like_logits_ad = !looks_like_probs_ad && !looks_like_one_hot_ad;

    ESP_LOGI(TAG,
             "AUTO-DETECT: min=%.3f max=%.3f sum=%.3f nz=%d => probs=%s one_hot=%s logits=%s",
             min_val_ad, max_val_ad, sum_ad, non_zero_ad,
             looks_like_probs_ad ? "YES" : "NO",
             looks_like_one_hot_ad ? "YES" : "NO",
             looks_like_logits_ad ? "YES" : "NO");

    if (looks_like_probs_ad) {
      // ----- Already soft-maxed probabilities -----
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
      // ----- Direct class index (one-hot) -----
      int class_id_ad = 0;
      for (int i = 0; i < num_classes; ++i) {
        if (std::fabs(output_data[i]) > 0.5f) {
          class_id_ad = i;
          break;
        }
      }
      result.value      = static_cast<float>(class_id_ad) / config_.scale_factor;
      result.confidence = 1.0f;   // no probability info, assume full confidence
      ESP_LOGD(TAG,
               "Auto-detect (direct): value=%.1f confidence=1.0",
               result.value);
    } else {
      // ----- Raw logits – compute soft-max ourselves -----
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
    /* ---------------------------------------------------------------
     *  Unknown processing method – fall back to direct classification.
     * --------------------------------------------------------------- */
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


/////// original
/* ProcessedOutput ModelHandler::process_output(const float* output_data) const {
  const int num_classes = output_size_;
  ProcessedOutput result = {0.0f, 0.0f};
  
  if (num_classes <= 0) {
    ESP_LOGE(TAG, "Invalid number of output classes: %d", num_classes);
    return result;
  }
  
#ifdef DEBUG_METER_READER_TFLITE
  //// RAW model output
  // ESP_LOGD(TAG, "Raw model outputs before any processing:");
    // for (int i = 0; i < num_classes; i++) {
        // ESP_LOGD(TAG, "  Class %d: %.6f", i, output_data[i]);
    // }

#endif

  // Debug: log output range
  float min_val = *std::min_element(output_data, output_data + num_classes);
  float max_val = *std::max_element(output_data, output_data + num_classes);
  ESP_LOGD(TAG, "Output range: min=%.2f, max=%.2f", min_val, max_val);

  // Find the max value and its index
  int max_idx = 0;
  float max_val_output = output_data[0];
  for (int i = 1; i < num_classes; i++) {
    if (output_data[i] > max_val_output) {
      max_val_output = output_data[i];
      max_idx = i;
    }
  }

  // Process based on output processing method
  if (config_.output_processing == "direct_class") {
    result.value = static_cast<float>(max_idx);
    result.confidence = max_val_output;
    ESP_LOGD(TAG, "Direct class - Value: %.1f, Confidence: %.6f", 
             result.value, result.confidence);
  }
  else if (config_.output_processing == "softmax") {
    // Calculate softmax probabilities
    float sum = 0.0f;
    std::vector<float> exp_values(num_classes);
    
    // Subtract max value for numerical stability
    float max_val = *std::max_element(output_data, output_data + num_classes);
        for (int i = 0; i < num_classes; i++) {
            exp_values[i] = expf(output_data[i] - max_val);
            sum += exp_values[i];
        }
    
    // Find the class with highest probability after softmax
    int softmax_max_idx = 0;
    float softmax_max_val = 0.0f;
    for (int i = 0; i < num_classes; i++) {
        float prob = exp_values[i] / sum;
        if (prob > softmax_max_val) {
            softmax_max_val = prob;
            softmax_max_idx = i;
        }
    }
    
    result.value = static_cast<float>(softmax_max_idx) / config_.scale_factor;  // ÷ 10.0
    result.confidence = softmax_max_val;
    ESP_LOGD(TAG, "Softmax - Value: %.1f, Confidence: %.6f", 
             result.value, result.confidence);
  }
  else if (config_.output_processing == "logits") { 
    // Treat raw outputs as logits - just find maximum value
    result.value = static_cast<float>(max_idx) / config_.scale_factor;
    
    // For logits, confidence values can be very large, so we normalize to 0-1 range
    // using a simple sigmoid-like transformation for better interpretability
    float confidence_range = max_val - min_val;
    if (confidence_range > 0) {
      // Normalize to 0-1 range based on distance from min value
      result.confidence = (max_val_output - min_val) / confidence_range;
    } else {
      result.confidence = 1.0f; // All values are the same
    }
    
    ESP_LOGD(TAG, "Logits - Value: %.1f, Raw Max: %.2f, Confidence: %.6f", 
             result.value, max_val_output, result.confidence);
  }
  else if (config_.output_processing == "experimental_scale") {
    // Experimental: try to handle very negative logits
    // Scale the outputs to make them more reasonable for softmax
    std::vector<float> scaled_outputs(num_classes);
    float scale_factor = 0.1f; // Adjust this based on observed output range
    
    for (int i = 0; i < num_classes; i++) {
      scaled_outputs[i] = output_data[i] * scale_factor;
    }
    
    // Then apply softmax to the scaled values
    float sum = 0.0f;
    std::vector<float> exp_values(num_classes);
    
    float max_val = *std::max_element(scaled_outputs.begin(), scaled_outputs.end());
    for (int i = 0; i < num_classes; i++) {
      exp_values[i] = expf(scaled_outputs[i] - max_val);
      sum += exp_values[i];
    }
    
    // Find the class with highest probability after softmax
    int softmax_max_idx = 0;
    float softmax_max_val = 0.0f;
    for (int i = 0; i < num_classes; i++) {
      float prob = exp_values[i] / sum;
      if (prob > softmax_max_val) {
        softmax_max_val = prob;
        softmax_max_idx = i;
      }
    }
    
    result.value = static_cast<float>(softmax_max_idx) / config_.scale_factor;
    result.confidence = softmax_max_val;
    ESP_LOGD(TAG, "Experimental scale - Value: %.1f, Confidence: %.6f", 
             result.value, result.confidence);
  }
  else if (config_.output_processing == "logits_jomjol") {
    // Exact replication of GetOutClassification() behavior
    // Simply find the maximum value and return its index (scaled by 10)
    result.value = static_cast<float>(max_idx) / config_.scale_factor;
    
    // For confidence, use the raw maximum value
    // Original C++ code didn't calculate confidence, so we use raw value
    result.confidence = max_val_output;
    
    ESP_LOGD(TAG, "Logits jomjol - Value: %.1f, Raw Max: %.6f", 
             result.value, max_val_output);
  }
  else if (config_.output_processing == "softmax_jomjol") {
    // Exact replication of Python script behavior
    // Always apply softmax first, then find max probability
    float sum = 0.0f;
    std::vector<float> exp_values(num_classes);
    
    // Subtract max for numerical stability (like TensorFlow does)
    float max_val = *std::max_element(output_data, output_data + num_classes);
    for (int i = 0; i < num_classes; i++) {
        exp_values[i] = expf(output_data[i] - max_val);
        sum += exp_values[i];
    }
    
    // Find the class with highest probability after softmax
    int max_idx = 0;
    float max_prob = 0.0f;
    for (int i = 0; i < num_classes; i++) {
        float prob = exp_values[i] / sum;
        if (prob > max_prob) {
            max_prob = prob;
            max_idx = i;
        }
    }
    
    // Apply scaling (like Python script's default case)
    result.value = static_cast<float>(max_idx) / config_.scale_factor;
    result.confidence = max_prob;

  
#ifdef DEBUG_METER_READER_TFLITE    
    // ESP_LOGD(TAG, "Softmax probabilities:");
    // for (int i = 0; i < num_classes; i++) {
        // float prob = exp_values[i] / sum;
        // ESP_LOGD(TAG, "  Class %d: %.6f", i, prob);
    // }
#endif
    
    ESP_LOGD(TAG, "Softmax jomjol - Value: %.1f, Confidence: %.6f", 
             result.value, result.confidence);
  }
  else {
    ESP_LOGE(TAG, "Unknown output processing method: %s", 
             config_.output_processing.c_str());
    // Default to direct classification
    result.value = static_cast<float>(max_idx);
    result.confidence = max_val_output;
  }

  return result;
} */


bool ModelHandler::invoke_model(const uint8_t* input_data, size_t input_size) {
    DURATION_START();

    if (!interpreter_ || !input_tensor()) {
        ESP_LOGE(TAG, "Interpreter or input tensor not available");
        return false;
    }

    TfLiteTensor* input = input_tensor();
    

    
#ifdef DEBUG_METER_READER_TFLITE

    // Verify input quantization before inference
    if (!verify_input_quantization(input_data, input_size)) {
        ESP_LOGW(TAG, "Input quantization verification failed");
    }
    
    // Detailed quantization analysis
    debug_input_quantization_analysis(input_data, input_size, "pre_inference");
    
    // Also debug the input tensor details
    debug_input_tensor_details();
#endif
    
    // Validate input size against the TFLite tensor's expected size in bytes
    if (input_size != static_cast<size_t>(input->bytes)) {
        ESP_LOGE(TAG, "Input size mismatch! Expected %d bytes, got %zu bytes", input->bytes, input_size);
        // Log more details for debugging
        ESP_LOGE(TAG, "Model input dimensions: %dx%dx%d", 
                 get_input_width(), get_input_height(), get_input_channels());
        
        // FIXED: Use correct element size for calculation
        size_t element_size = (input->type == kTfLiteFloat32) ? sizeof(float) : sizeof(uint8_t);
        ESP_LOGE(TAG, "Expected elements: %zu, Actual elements: %zu", 
                 static_cast<size_t>(input->bytes) / element_size,
                 input_size / element_size);
        return false;
    }
  
    // FIXED: Proper element count calculation
    int expected_elements = get_input_width() * get_input_height() * get_input_channels();
    size_t element_size = (input->type == kTfLiteFloat32) ? sizeof(float) : sizeof(uint8_t);
    size_t expected_bytes = expected_elements * element_size;
    
    ESP_LOGD(TAG, "Input validation: %d elements, %zu bytes expected", expected_elements, expected_bytes);
    
    if (input_size != expected_bytes) {
        // FIXED: Use correct element count in error message
        size_t actual_elements = input_size / element_size;
        ESP_LOGE(TAG, "Input dimension mismatch! Expected %d elements (%zu bytes), got %zu elements (%zu bytes)", 
                 expected_elements, expected_bytes, actual_elements, input_size);
        ESP_LOGE(TAG, "Model shape: %dx%dx%d, type: %s", 
                 get_input_width(), get_input_height(), get_input_channels(),
                 input->type == kTfLiteFloat32 ? "float32" : "uint8");
        return false;
    }
    
    // Check if input_data is valid
    ESP_LOGD(TAG, "Input data info: pointer=%p, size=%zu, tensor bytes=%d, type=%s", 
             input_data, input_size, input->bytes,
             input->type == kTfLiteFloat32 ? "float32" : "uint8");
    
    // FIXED: Correct element count for debug logging
    size_t actual_elements = input_size / element_size;
    ESP_LOGD(TAG, "Provided input size: %zu elements (%zu bytes)", actual_elements, input_size);
    
    if (input_data == nullptr) {
        ESP_LOGE(TAG, "Input data is null!");
        return false;
    }

#ifdef DEBUG_METER_READER_TFLITE
    debug_tensor_types(); // Debug tensor types before processing
    debug_input_data_stats(input_data, input_size); // Debug input data statistics
#endif

    // Handle different input types
    if (input->type == kTfLiteUInt8) {
        // For uint8 quantized models, the input is typically the raw pixel values.
        ESP_LOGD(TAG, "Quantized input (uint8) - scale: %.6f, zero_point: %d",
                input->params.scale, input->params.zero_point);
        
        // DIRECT COPY - no dequantization needed for input!
        memcpy(input->data.uint8, input_data, input_size);
        
        // Debug log first 5 values
        ESP_LOGD(TAG, "First 5 quantized inputs:");
        for (int i = 0; i < 5 && i < input_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %u", i, input->data.uint8[i]);
        }
        
#ifdef DEBUG_METER_READER_TFLITE
        debug_quantized_input_details(input, input_size); // Detailed quantized input analysis
#endif
    }
    else if (input->type == kTfLiteInt8) {
        // For int8 quantized models, convert uint8 pixel values (0-255) to int8 (-128 to 127)
        ESP_LOGD(TAG, "Quantized input (int8) - scale: %.6f, zero_point: %d",
                input->params.scale, input->params.zero_point);
        int8_t* dst = input->data.int8;
        for (size_t i = 0; i < input_size; i++) {
            dst[i] = (int8_t)((int)input_data[i] - 128);
        }

        // Debug log first 5 values
        ESP_LOGD(TAG, "First 5 quantized inputs:");
        for (int i = 0; i < 5 && i < input_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %d (from %u)", i, dst[i], input_data[i]);
        }
        
#ifdef DEBUG_METER_READER_TFLITE
        debug_int8_conversion_details(input, input_data, input_size); // Detailed int8 conversion analysis
#endif
    }
    else if (input->type == kTfLiteFloat32) {
        float* dst = input->data.f;
        
        // The loop is replaced with memcpy to fix a buffer overflow.
        memcpy(dst, input_data, input_size);


    // if (config_.normalize) {
        //// Convert 0-255 → 0-1
        // for (size_t i = 0; i < input_size / sizeof(float); ++i) {
            // dst[i] = static_cast<float>(input_data[i]) / 255.0f;
        // }
    // } else {
        // memcpy(dst, input_data, input_size);
    // }
     
        

        // Debug logging
        ESP_LOGD(TAG, "First 5 float32 inputs:");
        for (int i = 0; i < 5 && i < input_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %.4f", i, dst[i]);
        }

        // Add detailed input statistics
        ESP_LOGD(TAG, "Input tensor statistics (float32, 0-255 range):");
        float sum = 0.0f;
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();
        int zero_count = 0;
        int low_count = 0; // < 10
        int high_count = 0; // > 245

        for (int i = 0; i < std::min(50, (int)input_size); i++) {
            float val = dst[i];
            sum += val;
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
            if (val == 0.0f) zero_count++;
            if (val < 10.0f) low_count++;
            if (val > 245.0f) high_count++;
            
            if (i < 10) {
                ESP_LOGD(TAG, "  [%d]: %.1f", i, val);
            }
        }

        float mean = sum / std::min(50, (int)input_size);
        ESP_LOGD(TAG, "  Stats: min=%.1f, max=%.1f, mean=%.1f", min_val, max_val, mean);
        ESP_LOGD(TAG, "  Counts: zeros=%d, low=%d, high=%d", zero_count, low_count, high_count);
        
        // Check for suspicious patterns that indicate channel order issues
        if (zero_count > 30) {
            ESP_LOGW(TAG, "Too many zero values - possible channel order issue");
        }

        // Debug input pattern
        debug_input_pattern();
    }
  
#ifdef DEBUG_METER_READER_TFLITE
    ESP_LOGD(TAG, "First 10 input values:");
    for (int i = 0; i < 10 && i < input_size; i++) {
        ESP_LOGD(TAG, "  [%d]: %u", i, input_data[i]);
    }
    
    debug_pre_inference_state(); // Debug state before inference
#endif
  
    // Perform inference
    if (interpreter_->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Inference failed");
        return false;
    }

    // Handle output
    TfLiteTensor* output = output_tensor();
    if (!output) {
        ESP_LOGE(TAG, "No output tensor");
        return false;
    }

    // Set output size FIRST 
    output_size_ = output->dims->data[1];

#ifdef DEBUG_METER_READER_TFLITE
    debug_output_tensor_details(output); // Debug output tensor details
#endif

    // Handle quantized outputs
    if (output->type == kTfLiteUInt8 || output->type == kTfLiteInt8) {
        // Dequantize the outputs
        const float scale = output->params.scale;
        const int zero_point = output->params.zero_point;
        
        ESP_LOGD(TAG, "Quantized output (%s) - scale: %.6f, zero_point: %d",
                 output->type == kTfLiteUInt8 ? "uint8" : "int8",
                 scale, zero_point);
        
        // Prepare dequantized output buffer
        dequantized_output_.resize(output_size_);
        
        if (output->type == kTfLiteUInt8) {
            for (int i = 0; i < output_size_; i++) {
                dequantized_output_[i] = (static_cast<float>(output->data.uint8[i]) - zero_point) * scale;
            }
        } else { // kTfLiteInt8
            for (int i = 0; i < output_size_; i++) {
                dequantized_output_[i] = (static_cast<float>(output->data.int8[i]) - zero_point) * scale;
            }
        }
        model_output_ = dequantized_output_.data();
        
        ESP_LOGD(TAG, "First 5 dequantized outputs:");
        if (output->type == kTfLiteUInt8) {
            for (int i = 0; i < 5 && i < output_size_; i++) {
                ESP_LOGD(TAG, "  [%d]: %u -> %.6f", i, output->data.uint8[i], dequantized_output_[i]);
            }
        } else {
            for (int i = 0; i < 5 && i < output_size_; i++) {
                ESP_LOGD(TAG, "  [%d]: %d -> %.6f", i, output->data.int8[i], dequantized_output_[i]);
            }
        }
    } 
    else {
        model_output_ = output->data.f;
    }

#ifdef DEBUG_METER_READER_TFLITE
    debug_raw_outputs(output); // Debug raw outputs before processing
    debug_qat_model_output(); // QAT-specific debug output
#endif
    
    // Process the output to get both value and confidence
    processed_output_ = process_output(model_output_);
    ESP_LOGD(TAG, "Processed output - Value: %.1f, Confidence: %.6f", 
             processed_output_.value, processed_output_.confidence);
    
    DURATION_END("ModelHandler::invoke_model");
    return true;
}


/* ///////////// Original

bool ModelHandler::invoke_model(const uint8_t* input_data, size_t input_size) {
    DURATION_START();

    if (!interpreter_ || !input_tensor()) {
        ESP_LOGE(TAG, "Interpreter or input tensor not available");
        return false;
    }

    TfLiteTensor* input = input_tensor();
    
    // Validate input size against the TFLite tensor's expected size in bytes
    if (input_size != static_cast<size_t>(input->bytes)) {
        ESP_LOGE(TAG, "Input size mismatch! Expected %d bytes, got %zu bytes", input->bytes, input_size);
        // Log more details for debugging
        ESP_LOGE(TAG, "Model input dimensions: %dx%dx%d", 
                 get_input_width(), get_input_height(), get_input_channels());
        ESP_LOGE(TAG, "Expected elements: %zu, Actual elements: %zu", 
                 static_cast<size_t>(input->bytes) / (input->type == kTfLiteFloat32 ? sizeof(float) : sizeof(uint8_t)),
                 input_size / (input->type == kTfLiteFloat32 ? sizeof(float) : sizeof(uint8_t)));
        return false;
    }
  
  ESP_LOGD(TAG, "Model input dimensions: %dx%dx%d", 
         get_input_width(), get_input_height(), get_input_channels());
  ESP_LOGD(TAG, "Provided input size: %zu elements (%zu bytes)", 
       input_size / sizeof(float), input_size);

  if (input_size != get_input_width() * get_input_height() * get_input_channels() * sizeof(float)) {
    ESP_LOGE(TAG, "Input dimension mismatch! Expected %d elements, got %zu", 
         get_input_width() * get_input_height() * get_input_channels(),
         input_size / sizeof(float));
    return false;
  }
    
    // Check if input_data is valid
    ESP_LOGD(TAG, "Input data info: pointer=%p, size=%zu, tensor bytes=%d", 
             input_data, input_size, input->bytes);
    if (input_data == nullptr) {
        ESP_LOGE(TAG, "Input data is null!");
        return false;
    }

    // Handle different input types
    if (input->type == kTfLiteUInt8) {
        // Quantized model processing
        const float input_scale = input->params.scale;
        const int input_zero_point = input->params.zero_point;
        
        ESP_LOGD(TAG, "Quantized input - scale: %.6f, zero_point: %d",
                input_scale, input_zero_point);
        
        memcpy(input->data.uint8, input_data, input_size);
        
        // Debug log first 5 values
        ESP_LOGD(TAG, "First 5 quantized inputs:");
        for (int i = 0; i < 5 && i < input_size; i++) {
            ESP_LOGD(TAG, "  [%d]: %u (%.4f)", i, input->data.uint8[i],
                    (input->data.uint8[i] - input_zero_point) * input_scale);
        }
    } 
  else if (input->type == kTfLiteFloat32) {
    
    float* dst = input->data.f;
    
    // The loop is replaced with memcpy to fix a buffer overflow.
    memcpy(dst, input_data, input_size);

    // Debug logging
    ESP_LOGD(TAG, "First 5 float32 inputs:");
    for (int i = 0; i < 5 && i < input_size; i++) {
      ESP_LOGD(TAG, "  [%d]: %.4f", i, dst[i]);
    }

    
    // Add detailed input statistics
        ESP_LOGD(TAG, "Input tensor statistics (float32, 0-255 range):");
        float sum = 0.0f;
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();
        int zero_count = 0;
        int low_count = 0; // < 10
        int high_count = 0; // > 245

        for (int i = 0; i < std::min(50, (int)input_size); i++) {
            float val = dst[i];
            sum += val;
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
            if (val == 0.0f) zero_count++;
            if (val < 10.0f) low_count++;
            if (val > 245.0f) high_count++;
            
            if (i < 10) {
                ESP_LOGD(TAG, "  [%d]: %.1f", i, val);
            }
        }

        float mean = sum / std::min(50, (int)input_size);
        ESP_LOGD(TAG, "  Stats: min=%.1f, max=%.1f, mean=%.1f", min_val, max_val, mean);
        ESP_LOGD(TAG, "  Counts: zeros=%d, low=%d, high=%d", zero_count, low_count, high_count);
        
        // Check for suspicious patterns that indicate channel order issues
        if (zero_count > 30) {
            ESP_LOGW(TAG, "Too many zero values - possible channel order issue");
        }

        // Debug input pattern
        debug_input_pattern();
  }
  
#ifdef DEBUG_METER_READER_TFLITE
  ESP_LOGD(TAG, "First 10 input values:");
  for (int i = 0; i < 10 && i < input_size; i++) {
    ESP_LOGD(TAG, "  [%d]: %u", i, input_data[i]);
  }
#endif
  
    // Perform inference
    if (interpreter_->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Inference failed");
        return false;
    }

    // Handle output
    TfLiteTensor* output = output_tensor();
    if (!output) {
        ESP_LOGE(TAG, "No output tensor");
        return false;
    }

    // Set output size FIRST 
    output_size_ = output->dims->data[1];

    if (output->type == kTfLiteUInt8) {
        // Prepare dequantized output buffer
        dequantized_output_.resize(output_size_);
        const float scale = output->params.scale;
        const int zero_point = output->params.zero_point;
        
        for (int i = 0; i < output_size_; i++) {
            dequantized_output_[i] = (output->data.uint8[i] - zero_point) * scale;
        }
        model_output_ = dequantized_output_.data();
    } 
    else {
        model_output_ = output->data.f;
    }

#ifdef DEBUG_METER_READER_TFLITE
    //// Log raw output values for debugging
    // ESP_LOGD(TAG, "Raw output values (%d classes):", output_size_);
    // for (int i = 0; i < output_size_ && i < 15; i++) {
        // ESP_LOGD(TAG, "  Output[%d]: %.6f", i, model_output_[i]);
    // }
#endif
    
    // Process the output to get both value and confidence
    processed_output_ = process_output(model_output_);
    ESP_LOGD(TAG, "Processed output - Value: %.1f, Confidence: %.6f", 
             processed_output_.value, processed_output_.confidence);
    
    DURATION_END("ModelHandler::invoke_model");
    return true;
} */


#ifdef DEBUG_METER_READER_TFLITE


bool ModelHandler::validate_model_config() {
    auto* input = input_tensor();
    if (!input) return false;

    // Detailed model input information
    ESP_LOGI(TAG, "Model input tensor details:");
    ESP_LOGI(TAG, "  - Type: %s", input->type == kTfLiteUInt8 ? "uint8" : "float32");
    ESP_LOGI(TAG, "  - Bytes: %d", input->bytes);
    ESP_LOGI(TAG, "  - Dimensions: %d", input->dims->size);
    for (int i = 0; i < input->dims->size; i++) {
        ESP_LOGI(TAG, "    - dim[%d]: %d", i, input->dims->data[i]);
    }
    
    ESP_LOGI(TAG, "=== QUANTIZATION DIAGNOSTIC ===");
    ESP_LOGI(TAG, "INPUT: type=%d, scale=%.6f, zp=%d", 
             input->type, input->params.scale, input->params.zero_point);
    
    TfLiteTensor* output = output_tensor();
    if (output) {
        ESP_LOGI(TAG, "OUTPUT: type=%d, scale=%.6f, zp=%d", 
                 output->type, output->params.scale, output->params.zero_point);
        
        // Check if output needs dequantization
        if (output->type == kTfLiteUInt8 || output->type == kTfLiteInt8) {
            ESP_LOGI(TAG, "OUTPUT IS QUANTIZED - needs dequantization!");
        } else {
            ESP_LOGI(TAG, "OUTPUT IS FLOAT - already dequantized");
        }
    }
    
    // Calculate expected size
    size_t expected_size = 1;
    for (int i = 1; i < input->dims->size; i++) {
        expected_size *= input->dims->data[i];
    }
    ESP_LOGI(TAG, "  - Expected data size: %zu bytes", expected_size);

    // Update config with ACTUAL model dimensions
    if (input->dims->size >= 4) {
        config_.input_size = {input->dims->data[1], input->dims->data[2]};
        config_.input_channels = input->dims->data[3];
    }
        
    return true;
}

bool ModelHandler::verify_input_quantization(const uint8_t* input_data, size_t input_size) const {
    TfLiteTensor* input = input_tensor();
    if (!input || !input_data) return false;
    
    // Check if model expects quantized input
    if (input->type == kTfLiteUInt8) {
        // For quantized models, verify input data is in expected range
        uint8_t min_val = 255;
        uint8_t max_val = 0;
        
        for (size_t i = 0; i < input_size; i++) {
            min_val = std::min(min_val, input_data[i]);
            max_val = std::max(max_val, input_data[i]);
        }
        
        ESP_LOGD(TAG, "Quantized input range: [%u, %u]", min_val, max_val);
        
        // Check if values are within uint8 range
        if (max_val > 255) {
            ESP_LOGW(TAG, "Input values exceed uint8 range");
            return false;
        }
        
        // Check if zero point is reasonable
        int zero_point = input->params.zero_point;
        if (zero_point < 0 || zero_point > 255) {
            ESP_LOGW(TAG, "Suspicious zero point: %d", zero_point);
        }
        
        return true;
    }
    else if (input->type == kTfLiteFloat32) {
        // For float models, check if values are in reasonable range
        const float* float_data = reinterpret_cast<const float*>(input_data);
        size_t float_count = input_size / sizeof(float);
        
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();
        
        for (size_t i = 0; i < float_count; i++) {
            min_val = std::min(min_val, float_data[i]);
            max_val = std::max(max_val, float_data[i]);
        }
        
        ESP_LOGD(TAG, "Float input range: [%.3f, %.3f]", min_val, max_val);
        
        // Check if values are normalized (0-1) or in pixel range (0-255)
        if (max_val > 1.0f && max_val <= 255.0f) {
            ESP_LOGD(TAG, "Input appears to be in pixel range [0,255]");
        } else if (max_val <= 1.0f && min_val >= 0.0f) {
            ESP_LOGD(TAG, "Input appears to be normalized [0,1]");
        } else {
            ESP_LOGW(TAG, "Input range may be unexpected: [%.3f, %.3f]", min_val, max_val);
        }
        
        return true;
    }
    
    return true; // For other types, assume it's fine
}

void ModelHandler::debug_input_quantization_analysis(const uint8_t* input_data, 
                                                   size_t input_size,
                                                   const std::string& stage) const {
    TfLiteTensor* input = input_tensor();
    if (!input || !input_data || input_size == 0) return;
    
    ESP_LOGI(TAG, "=== INPUT QUANTIZATION ANALYSIS: %s ===", stage.c_str());
    // ESP_LOGI(TAG, "Input tensor type: %s", tflite_type_to_string(input->type));
    ESP_LOGI(TAG, "Input quantization: scale=%.6f, zp=%d", 
             input->params.scale, input->params.zero_point);
    ESP_LOGI(TAG, "Input size: %zu bytes (%zu elements)", 
             input_size, input_size / (input->type == kTfLiteFloat32 ? sizeof(float) : 1));
    
    // Analyze input data statistics
    if (input->type == kTfLiteUInt8) {
        uint8_t min_val = 255;
        uint8_t max_val = 0;
        uint32_t sum = 0;
        int zero_count = 0;
        
        for (size_t i = 0; i < input_size; i++) {
            min_val = std::min(min_val, input_data[i]);
            max_val = std::max(max_val, input_data[i]);
            sum += input_data[i];
            if (input_data[i] == 0) zero_count++;
        }
        
        float mean = static_cast<float>(sum) / input_size;
        ESP_LOGI(TAG, "Input data stats: min=%u, max=%u, mean=%.1f", min_val, max_val, mean);
        ESP_LOGI(TAG, "Zero values: %d/%zu (%.1f%%)", zero_count, input_size, 
                 (zero_count * 100.0f) / input_size);
        
        // Check against expected quantization
        const float expected_scale = 0.003922f; // 1/255
        const int expected_zp = 0;
        
        bool scale_match = fabs(input->params.scale - expected_scale) < 0.0001f;
        bool zp_match = (input->params.zero_point == expected_zp);
        
        if (scale_match && zp_match) {
            ESP_LOGI(TAG, "✓ Quantization matches expected (scale=1/255, zp=0)");
        } else {
            ESP_LOGW(TAG, "⚠️ Quantization differs from expected");
            ESP_LOGW(TAG, "  Expected: scale=%.6f, zp=%d", expected_scale, expected_zp);
            ESP_LOGW(TAG, "  Actual: scale=%.6f, zp=%d", input->params.scale, input->params.zero_point);
        }
        
    } else if (input->type == kTfLiteFloat32) {
        const float* float_data = reinterpret_cast<const float*>(input_data);
        size_t float_count = input_size / sizeof(float);
        
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();
        float sum = 0.0f;
        int zero_count = 0;
        
        for (size_t i = 0; i < float_count; i++) {
            min_val = std::min(min_val, float_data[i]);
            max_val = std::max(max_val, float_data[i]);
            sum += float_data[i];
            if (float_data[i] == 0.0f) zero_count++;
        }
        
        float mean = sum / float_count;
        ESP_LOGI(TAG, "Input data stats: min=%.3f, max=%.3f, mean=%.3f", min_val, max_val, mean);
        ESP_LOGI(TAG, "Zero values: %d/%zu (%.1f%%)", zero_count, float_count, 
                 (zero_count * 100.0f) / float_count);
    }
    
    ESP_LOGI(TAG, "=== END INPUT ANALYSIS ===");
}

// Add this helper function to model_handler.cpp
// const char* ModelHandler::tflite_type_to_string(TfLiteType type) const {
    // switch (type) {
        // case kTfLiteFloat32: return "FLOAT32";
        // case kTfLiteInt32: return "INT32";
        // case kTfLiteUInt8: return "UINT8";
        // case kTfLiteInt8: return "INT8";
        // case kTfLiteInt64: return "INT64";
        // case kTfLiteString: return "STRING";
        // case kTfLiteBool: return "BOOL";
        // case kTfLiteInt16: return "INT16";
        // case kTfLiteComplex64: return "COMPLEX64";
        // case kTfLiteComplex128: return "COMPLEX128";
        // case kTfLiteFloat16: return "FLOAT16";
        // case kTfLiteFloat64: return "FLOAT64";
        // default: return "UNKNOWN";
    // }
// }

void ModelHandler::debug_input_tensor_details() const {
    TfLiteTensor* input = input_tensor();
    if (!input) return;
    
    ESP_LOGI(TAG, "=== INPUT TENSOR DETAILS ===");
    ESP_LOGI(TAG, "Type: %s", tflite_type_to_string(input->type));
    ESP_LOGI(TAG, "Dimensions: %d", input->dims->size);
    for (int i = 0; i < input->dims->size; i++) {
        ESP_LOGI(TAG, "  dim[%d]: %d", i, input->dims->data[i]);
    }
    ESP_LOGI(TAG, "Bytes: %zu", input->bytes);
    ESP_LOGI(TAG, "Quantization: scale=%.6f, zero_point=%d", 
             input->params.scale, input->params.zero_point);
    ESP_LOGI(TAG, "=== END TENSOR DETAILS ===");
}


void ModelHandler::debug_tensor_types() const {
    TfLiteTensor* input = input_tensor();
    TfLiteTensor* output = output_tensor();
    
    ESP_LOGI(TAG, "=== TENSOR TYPE VERIFICATION (vs Python) ===");
    
    if (input) {
        ESP_LOGI(TAG, "INPUT: type=%s, scale=%.6f, zp=%d",
                 input->type == kTfLiteUInt8 ? "UINT8" : 
                 input->type == kTfLiteInt8 ? "INT8" : "FLOAT32",
                 input->params.scale, input->params.zero_point);
        
        // Compare with Python expectations
        if (input->type == kTfLiteUInt8 && std::abs(input->params.scale - 0.003922f) < 0.000001f && input->params.zero_point == 0) {
            ESP_LOGI(TAG, "✓ INPUT matches Python: UINT8, scale=0.003922, zp=0");
        } else if (input->type == kTfLiteUInt8) {
            ESP_LOGI(TAG, "⚠️ INPUT is UINT8 but scale/zp don't match Python exactly");
        }
    }
    
    if (output) {
        ESP_LOGI(TAG, "OUTPUT: type=%s, scale=%.6f, zp=%d",
                 output->type == kTfLiteUInt8 ? "UINT8" : 
                 output->type == kTfLiteInt8 ? "INT8" : "FLOAT32",
                 output->params.scale, output->params.zero_point);
        
        // Compare with Python expectations  
        if (output->type == kTfLiteUInt8 && std::abs(output->params.scale - 0.003906f) < 0.000001f && output->params.zero_point == 0) {
            ESP_LOGI(TAG, "✓ OUTPUT matches Python: UINT8, scale=0.003906, zp=0");
        } else if (output->type == kTfLiteUInt8) {
            ESP_LOGI(TAG, "⚠️ OUTPUT is UINT8 but scale/zp don't match Python exactly");
        }
    }
    
    // Check if this matches your model structure
    ESP_LOGI(TAG, "Expected from Python analysis:");
    ESP_LOGI(TAG, "  - Input: UINT8, scale=0.003922, zp=0");
    ESP_LOGI(TAG, "  - Output: UINT8, scale=0.003906, zp=0");
}

void ModelHandler::debug_input_data_stats(const uint8_t* input_data, size_t input_size) const {
    ESP_LOGI(TAG, "=== INPUT DATA STATISTICS ===");
    
    if (!input_data || input_size == 0) {
        ESP_LOGE(TAG, "Invalid input data for statistics");
        return;
    }
    
    // Calculate statistics
    uint32_t sum = 0;
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    int zero_count = 0;
    int low_count = 0;  // < 50
    int high_count = 0; // > 200
    
    for (size_t i = 0; i < input_size; i++) {
        uint8_t val = input_data[i];
        sum += val;
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
        if (val == 0) zero_count++;
        if (val < 50) low_count++;
        if (val > 200) high_count++;
    }
    
    float mean = static_cast<float>(sum) / input_size;
    
    ESP_LOGI(TAG, "Input data range: [%u, %u]", min_val, max_val);
    ESP_LOGI(TAG, "Input data mean: %.2f", mean);
    ESP_LOGI(TAG, "Counts - zeros: %d, low (<50): %d, high (>200): %d", 
             zero_count, low_count, high_count);
    ESP_LOGI(TAG, "Zero percentage: %.2f%%", (zero_count * 100.0f) / input_size);
    
    // Check for potential issues
    if (zero_count > input_size * 0.3f) {
        ESP_LOGW(TAG, "⚠️  High zero count - possible preprocessing issue");
    }
    if (max_val < 100) {
        ESP_LOGW(TAG, "⚠️  Low maximum value - image might be too dark");
    }
}

void ModelHandler::debug_quantized_input_details(TfLiteTensor* input, size_t input_size) const {
    ESP_LOGI(TAG, "=== QUANTIZED INPUT ANALYSIS ===");
    ESP_LOGI(TAG, "Input quantization - scale: %.6f, zero_point: %d",
             input->params.scale, input->params.zero_point);
    
    // Show first 10 values in both quantized and dequantized form
    ESP_LOGI(TAG, "First 10 input values (quantized -> dequantized):");
    for (int i = 0; i < 10 && i < input_size; i++) {
        uint8_t quantized = input->data.uint8[i];
        float dequantized = (quantized - input->params.zero_point) * input->params.scale;
        ESP_LOGI(TAG, "  [%d]: %u -> %.6f", i, quantized, dequantized);
    }
    
    // Check if this matches expected Python behavior
    if (input->params.scale == 0.003922f && input->params.zero_point == 0) {
        ESP_LOGI(TAG, "✅ Input quantization matches Python preprocessing");
    } else {
        ESP_LOGW(TAG, "⚠️  Input quantization differs from Python expectations");
    }
}

void ModelHandler::debug_int8_conversion_details(TfLiteTensor* input, const uint8_t* input_data, size_t input_size) const {
    ESP_LOGI(TAG, "=== INT8 CONVERSION ANALYSIS ===");
    ESP_LOGI(TAG, "Input quantization - scale: %.6f, zero_point: %d",
             input->params.scale, input->params.zero_point);
    
    // Show conversion examples
    ESP_LOGI(TAG, "First 10 conversion examples (uint8 -> int8 -> dequantized):");
    for (int i = 0; i < 10 && i < input_size; i++) {
        uint8_t original = input_data[i];
        int8_t converted = input->data.int8[i];
        float dequantized = (converted - input->params.zero_point) * input->params.scale;
        ESP_LOGI(TAG, "  [%d]: %u -> %d -> %.6f", i, original, converted, dequantized);
    }
}

void ModelHandler::debug_pre_inference_state() const {
    ESP_LOGI(TAG, "=== PRE-INFERENCE STATE ===");
    
    TfLiteTensor* input = input_tensor();
    if (input) {
        ESP_LOGI(TAG, "Input tensor ready - type: %s, bytes: %d",
                 input->type == kTfLiteUInt8 ? "UINT8" : 
                 input->type == kTfLiteInt8 ? "INT8" : "FLOAT32",
                 input->bytes);
    }
    
    // Check interpreter state
    if (interpreter_) {
        ESP_LOGI(TAG, "Interpreter arena used: %zu bytes", interpreter_->arena_used_bytes());
    }
}

void ModelHandler::debug_output_tensor_details(TfLiteTensor* output) const {
    ESP_LOGI(TAG, "=== OUTPUT TENSOR DETAILS ===");
    
    ESP_LOGI(TAG, "Output type: %s", 
             output->type == kTfLiteFloat32 ? "FLOAT32" :
             output->type == kTfLiteUInt8 ? "UINT8" :
             output->type == kTfLiteInt8 ? "INT8" : "OTHER");
    
    ESP_LOGI(TAG, "Output quantization - scale: %.6f, zero_point: %d",
             output->params.scale, output->params.zero_point);
    
    ESP_LOGI(TAG, "Output dimensions: %d", output->dims->size);
    for (int i = 0; i < output->dims->size; i++) {
        ESP_LOGI(TAG, "  dim[%d]: %d", i, output->dims->data[i]);
    }
    
    ESP_LOGI(TAG, "Output size (elements): %d", output_size_);
}

void ModelHandler::debug_raw_outputs(TfLiteTensor* output) const {
    ESP_LOGI(TAG, "=== RAW OUTPUTS BEFORE PROCESSING ===");
    
    if (output->type == kTfLiteUInt8 || output->type == kTfLiteInt8) {
        ESP_LOGI(TAG, "Quantized raw outputs (%s):", 
                 output->type == kTfLiteUInt8 ? "UINT8" : "INT8");
        
        for (int i = 0; i < output_size_ && i < 10; i++) {
            if (output->type == kTfLiteUInt8) {
                ESP_LOGI(TAG, "  [%d]: %u", i, output->data.uint8[i]);
            } else {
                ESP_LOGI(TAG, "  [%d]: %d", i, output->data.int8[i]);
            }
        }
    } else {
        ESP_LOGI(TAG, "Float raw outputs:");
        for (int i = 0; i < output_size_ && i < 10; i++) {
            ESP_LOGI(TAG, "  [%d]: %.6f", i, output->data.f[i]);
        }
    }
}

void ModelHandler::debug_qat_model_output() const {
    ESP_LOGI(TAG, "=== QAT MODEL DEBUG ===");
    ESP_LOGI(TAG, "Output processing method: %s", config_.output_processing.c_str());
    ESP_LOGI(TAG, "Scale factor: %.1f", config_.scale_factor);
    ESP_LOGI(TAG, "Number of classes: %d", output_size_);

    // Log all raw outputs
    std::string raw_outputs;
    for (int i = 0; i < output_size_; i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.6f ", model_output_[i]);
        raw_outputs += buf;
    }
    ESP_LOGI(TAG, "All outputs: [%s]", raw_outputs.c_str());

    // Find top 3 predictions
    std::vector<std::pair<float, int>> predictions;
    for (int i = 0; i < output_size_; i++) {
        predictions.push_back(std::make_pair(model_output_[i], i));
    }
    std::sort(predictions.rbegin(), predictions.rend());

    ESP_LOGI(TAG, "Top 3 predictions:");
    for (int i = 0; i < 3 && i < output_size_; i++) {
        ESP_LOGI(TAG, "  %d: value=%d, score=%.6f", 
                 i, predictions[i].second, predictions[i].first);
    }
    
    // Additional QAT-specific analysis
    float min_val = *std::min_element(model_output_, model_output_ + output_size_);
    float max_val = *std::max_element(model_output_, model_output_ + output_size_);
    ESP_LOGI(TAG, "Output range: min=%.6f, max=%.6f", min_val, max_val);
    
    // Check if outputs look like probabilities or logits
    bool looks_like_probabilities = (min_val >= 0.0f && max_val <= 1.0f);
    bool looks_like_logits = (max_val > 1.0f || min_val < 0.0f);
    
    ESP_LOGI(TAG, "Output characteristics:");
    ESP_LOGI(TAG, "  - Looks like probabilities: %s", looks_like_probabilities ? "YES" : "NO");
    ESP_LOGI(TAG, "  - Looks like logits: %s", looks_like_logits ? "YES" : "NO");
}

#endif

size_t ModelHandler::get_arena_peak_bytes() const {
  return interpreter_ ? interpreter_->arena_used_bytes() : 0;
}

TfLiteTensor* ModelHandler::input_tensor() const {
  return interpreter_ ? interpreter_->input(0) : nullptr;
}

TfLiteTensor* ModelHandler::output_tensor() const {
  return interpreter_ ? interpreter_->output(0) : nullptr;
}

void ModelHandler::debug_model_architecture() const {
    if (!tflite_model_ || !interpreter_) return;
    
    ESP_LOGD(TAG, "Model architecture analysis:");
    
    // Get the model structure
    auto* subgraph = tflite_model_->subgraphs()->Get(0);
    ESP_LOGD(TAG, "  - Number of operators: %d", subgraph->operators()->size());
    ESP_LOGD(TAG, "  - Number of tensors: %d", subgraph->tensors()->size());
    
    // Check input and output tensors
    TfLiteTensor* input = input_tensor();
    TfLiteTensor* output = output_tensor();
    
    if (input && output) {
        ESP_LOGD(TAG, "  - Input type: %s", 
                 input->type == kTfLiteFloat32 ? "float32" : 
                 input->type == kTfLiteUInt8 ? "uint8" : "other");
        ESP_LOGD(TAG, "  - Output type: %s", 
                 output->type == kTfLiteFloat32 ? "float32" : 
                 output->type == kTfLiteUInt8 ? "uint8" : "other");
    }
    
    // Check if this looks like a classification model
    if (output && output->dims->size >= 2) {
        int num_classes = output->dims->data[1];
        ESP_LOGD(TAG, "  - Number of classes: %d", num_classes);
        
        if (num_classes == 100) {
            ESP_LOGD(TAG, "  - Model type: 100-class classifier (scale10)");
        } else if (num_classes == 10) {
            ESP_LOGD(TAG, "  - Model type: 10-class classifier");
        }
    }
    
    // Log operator types for debugging
    ESP_LOGD(TAG, "  - Operator codes:");
    for (size_t i = 0; i < tflite_model_->operator_codes()->size(); ++i) {
        const auto *op_code = tflite_model_->operator_codes()->Get(i);
        ESP_LOGD(TAG, "    [%d]: %d (%s)", i, op_code->builtin_code(),
                 tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
    
    // Log tensor information
    // if (subgraph->tensors()) {
        // ESP_LOGD(TAG, "  - Tensor details:");
        // for (size_t i = 0; i < subgraph->tensors()->size(); ++i) {
            // const auto *tensor = subgraph->tensors()->Get(i);
            // ESP_LOGD(TAG, "    Tensor[%d]: %s, shape=[", i, tensor->name()->c_str());
            // if (tensor->shape()) {
                // for (size_t j = 0; j < tensor->shape()->size(); ++j) {
                    // ESP_LOGD(TAG, "%d", tensor->shape()->data()[j]);
                    // if (j < tensor->shape()->size() - 1) ESP_LOGD(TAG, ", ");
                // }
            // }
            // ESP_LOGD(TAG, "]");
        // }
    // }
}

#ifdef DEBUG_METER_READER_TFLITE

std::vector<ModelConfig> ModelHandler::generate_debug_configs() const {
    std::vector<ModelConfig> configs;
    
    // Base configurations to test
    std::vector<std::string> input_orders = {"BGR", "RGB"};
    std::vector<std::pair<int, int>> input_sizes = {{32, 20}, {20, 32}};
    std::vector<bool> normalize_options = {true, false};
    std::vector<std::string> input_types = {"float32", "uint8"};
    std::vector<std::string> output_processings = {
        "softmax", "direct_class", "logits", "experimental_scale"
    };
    
    // Generate all combinations
    for (const auto& input_order : input_orders) {
        for (const auto& input_size : input_sizes) {
            for (bool normalize : normalize_options) {
                for (const auto& input_type : input_types) {
                    for (const auto& output_processing : output_processings) {
                        ModelConfig config;
                        config.description = "debug_config";
                        config.input_order = input_order;
                        config.input_size = input_size;
                        config.normalize = normalize;
                        config.input_type = input_type;
                        config.output_processing = output_processing;
                        config.scale_factor = 10.0f;
                        config.input_channels = 3;
                        config.invert = false;
                        
                        configs.push_back(config);
                    }
                }
            }
        }
    }
    
    return configs;
}

void ModelHandler::test_configuration(const ModelConfig& config, 
                                    const std::vector<std::vector<uint8_t>>& zone_data,
                                    std::vector<ConfigTestResult>& results) {
    ESP_LOGI(TAG, "Testing configuration:");
    ESP_LOGI(TAG, "  Input order: %s", config.input_order.c_str());
    ESP_LOGI(TAG, "  Input size: %dx%d", config.input_size.first, config.input_size.second);
    ESP_LOGI(TAG, "  Normalize: %s", config.normalize ? "true" : "false");
    ESP_LOGI(TAG, "  Input type: %s", config.input_type.c_str());
    ESP_LOGI(TAG, "  Output processing: %s", config.output_processing.c_str());
    
    // Temporarily set this config
    ModelConfig original_config = config_;
    config_ = config;
    
    ConfigTestResult result;
    result.config = config;
    result.avg_confidence = 0.0f;
    
    float total_confidence = 0.0f;
    int successful_zones = 0;
    
    // Test this configuration on all zones
    for (size_t zone_idx = 0; zone_idx < zone_data.size(); zone_idx++) {
        const auto& zone_buffer = zone_data[zone_idx];
        
        // Reset watchdog every few zones to prevent timeout
        if (zone_idx % 3 == 0) {
            feed_watchdog();
        }
        
        if (invoke_model(zone_buffer.data(), zone_buffer.size())) {
            ProcessedOutput output = get_processed_output();
            result.zone_confidences.push_back(output.confidence);
            result.zone_values.push_back(output.value);
            total_confidence += output.confidence;
            successful_zones++;
            
            ESP_LOGI(TAG, "  Zone %d: Value=%.1f, Confidence=%.6f", 
                     zone_idx + 1, output.value, output.confidence);
        } else {
            ESP_LOGI(TAG, "  Zone %d: FAILED", zone_idx + 1);
            result.zone_confidences.push_back(0.0f);
            result.zone_values.push_back(0.0f);
        }
    }
    
    if (successful_zones > 0) {
        result.avg_confidence = total_confidence / successful_zones;
        ESP_LOGI(TAG, "  Average confidence: %.6f", result.avg_confidence);
    } else {
        result.avg_confidence = 0.0f;
        ESP_LOGI(TAG, "  All zones failed");
    }
    
    results.push_back(result);
    
    // Restore original config
    config_ = original_config;
}

void ModelHandler::debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data) {
    ESP_LOGI(TAG, "=== DEBUG PARAMETER TESTING ===");
    ESP_LOGI(TAG, "Testing %d zones with %zu configurations", 
             zone_data.size(), generate_debug_configs().size());
             
    // Reset watchdog at the start
    feed_watchdog();
    
    std::vector<ConfigTestResult> all_results;
    auto configs = generate_debug_configs();
    
    for (size_t i = 0; i < configs.size(); i++) {
        ESP_LOGI(TAG, "--- Test %zu/%zu ---", i + 1, configs.size());
        test_configuration(configs[i], zone_data, all_results);
        
        // Reset watchdog every few configurations
        if (i % 5 == 0) {
            feed_watchdog();
        }
    }
    
    // Sort results by average confidence (descending)
    std::sort(all_results.begin(), all_results.end(), 
        [](const ConfigTestResult& a, const ConfigTestResult& b) {
            return a.avg_confidence > b.avg_confidence;
        });
    
    // Display top 10 configurations
    ESP_LOGI(TAG, "=== TOP 10 CONFIGURATIONS ===");
    int display_count = std::min(10, static_cast<int>(all_results.size()));
    
    for (int i = 0; i < display_count; i++) {
        const auto& result = all_results[i];
        ESP_LOGI(TAG, "%d. Avg Confidence: %.6f", i + 1, result.avg_confidence);
        ESP_LOGI(TAG, "   Input order: %s", result.config.input_order.c_str());
        ESP_LOGI(TAG, "   Input size: %dx%d", result.config.input_size.first, result.config.input_size.second);
        ESP_LOGI(TAG, "   Normalize: %s", result.config.normalize ? "true" : "false");
        ESP_LOGI(TAG, "   Input type: %s", result.config.input_type.c_str());
        ESP_LOGI(TAG, "   Output processing: %s", result.config.output_processing.c_str());
        
        // Show zone-by-zone results for top configs
        for (size_t zone_idx = 0; zone_idx < result.zone_confidences.size(); zone_idx++) {
            ESP_LOGI(TAG, "   Zone %d: Value=%.1f, Confidence=%.6f", 
                     zone_idx + 1, result.zone_values[zone_idx], result.zone_confidences[zone_idx]);
        }
        ESP_LOGI(TAG, "   ---");
    }
    
    ESP_LOGI(TAG, "=== DEBUG TESTING COMPLETE ===");
}

void ModelHandler::feed_watchdog() {
    // ESP32-specific watchdog feed
    #ifdef ESP_PLATFORM
    esp_task_wdt_reset();
    #endif
    ESP_LOGD(TAG, "Watchdog fed");
}


void ModelHandler::verify_model_crc(const uint8_t* model_data, size_t model_size) {
    uint32_t crc = crc32_runtime(model_data, model_size);
    ESP_LOGI(TAG, "Model checksum (runtime): 0x%08" PRIX32, crc);
    ESP_LOGI(TAG, "Model checksum (compile): 0x%08X", MODEL_CRC32);

    if (crc == static_cast<uint32_t>(MODEL_CRC32)) {
        ESP_LOGI(TAG, "Model integrity verified");
    } else {
        ESP_LOGE(TAG, "Model integrity check failed!");
    }
}


uint32_t crc32_runtime(const uint8_t* data, size_t length) {
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
    return crc ^ 0xFFFFFFFF;
}

#endif

}  // namespace meter_reader_tflite
}  // namespace esphome
