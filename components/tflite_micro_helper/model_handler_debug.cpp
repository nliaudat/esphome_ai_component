#include "model_handler.h"
#include "esp_log.h"
#include "debug_utils.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

#ifdef ESP_PLATFORM
#include "esp_task_wdt.h"
#endif

namespace esphome {
namespace tflite_micro_helper {

static const char *const TAG = "ModelHandler";

// ... [Existing code] ...

// Helper to convert type to string for debug
static const char* tflite_type_to_string(TfLiteType type) {
    switch (type) {
        case kTfLiteFloat32: return "FLOAT32";
        case kTfLiteInt32: return "INT32";
        case kTfLiteUInt8: return "UINT8";
        case kTfLiteInt8: return "INT8";
        case kTfLiteInt64: return "INT64";
        case kTfLiteString: return "STRING";
        case kTfLiteBool: return "BOOL";
        case kTfLiteInt16: return "INT16";
        case kTfLiteComplex64: return "COMPLEX64";
        case kTfLiteComplex128: return "COMPLEX128";
        case kTfLiteFloat16: return "FLOAT16";
        case kTfLiteFloat64: return "FLOAT64";
        default: return "UNKNOWN";
    }
}

void ModelHandler::debug_input_quantization_analysis(const uint8_t* input_data, 
                                                   size_t input_size,
                                                   const std::string& stage) const {
    TfLiteTensor* input = input_tensor();
    if (!input || !input_data || input_size == 0) return;
    
    ESP_LOGI(TAG, "=== INPUT QUANTIZATION ANALYSIS: %s ===", stage.c_str());
    ESP_LOGI(TAG, "Input quantization: scale=%.6f, zp=%d", 
             input->params.scale, input->params.zero_point);
    ESP_LOGI(TAG, "Input size: %zu bytes", input_size);
    
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
    }
    ESP_LOGI(TAG, "=== END INPUT ANALYSIS ===");
}

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
    
    ESP_LOGI(TAG, "=== TENSOR TYPE VERIFICATION ===");
    if (input) {
         ESP_LOGI(TAG, "INPUT: type=%s, scale=%.6f, zp=%d",
                  tflite_type_to_string(input->type),
                  input->params.scale, input->params.zero_point);
    }
    if (output) {
         ESP_LOGI(TAG, "OUTPUT: type=%s, scale=%.6f, zp=%d",
                  tflite_type_to_string(output->type),
                  output->params.scale, output->params.zero_point);
    }
}

void ModelHandler::debug_input_data_stats(const uint8_t* input_data, size_t input_size) const {
    ESP_LOGI(TAG, "=== INPUT DATA STATISTICS ===");
    if (!input_data || input_size == 0) return;
    
    uint32_t sum = 0;
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    int zero_count = 0;
    
    for (size_t i = 0; i < input_size; i++) {
        uint8_t val = input_data[i];
        sum += val;
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
        if (val == 0) zero_count++;
    }
    
    ESP_LOGI(TAG, "Range: [%u, %u], Mean: %.2f, Zeros: %d (%.1f%%)", 
             min_val, max_val, static_cast<float>(sum)/input_size, 
             zero_count, (zero_count*100.0f)/input_size);
}

void ModelHandler::debug_quantized_input_details(TfLiteTensor* input, size_t input_size) const {
    if (!input || input->type != kTfLiteUInt8) return;
    ESP_LOGI(TAG, "First 10 input values (q -> deq):");
    for (size_t i = 0; i < 10 && i < input_size; i++) {
        uint8_t q = input->data.uint8[i];
        float deq = (q - input->params.zero_point) * input->params.scale;
        ESP_LOGI(TAG, "  [%d]: %u -> %.6f", i, q, deq);
    }
}

void ModelHandler::debug_int8_conversion_details(TfLiteTensor* input, const uint8_t* input_data, size_t input_size) const {
    if (!input || input->type != kTfLiteInt8) return;
    ESP_LOGI(TAG, "First 10 INT8 conversions:");
    for (size_t i = 0; i < 10 && i < input_size; i++) {
        uint8_t orig = input_data[i];
        int8_t conv = input->data.int8[i];
        float deq = (conv - input->params.zero_point) * input->params.scale;
        ESP_LOGI(TAG, "  [%d]: %u -> %d -> %.6f", i, orig, conv, deq);
    }
}

void ModelHandler::debug_pre_inference_state() const {
    TfLiteTensor* input = input_tensor();
    if (input) {
        ESP_LOGI(TAG, "Pre-inference: Input type %s, bytes %zu", 
                 tflite_type_to_string(input->type), input->bytes);
    }
}

void ModelHandler::debug_output_tensor_details(TfLiteTensor* output) const {
    if (!output) return;
    ESP_LOGI(TAG, "=== OUTPUT TENSOR DETAILS ===");
    ESP_LOGI(TAG, "Type: %s, Scale: %.6f, ZP: %d, Size: %d",
             tflite_type_to_string(output->type), 
             output->params.scale, output->params.zero_point, output_size_);
}

void ModelHandler::debug_raw_outputs(TfLiteTensor* output) const {
    ESP_LOGI(TAG, "=== RAW OUTPUTS ===");
    if (output->type == kTfLiteUInt8) {
        for (int i = 0; i < output_size_ && i < 10; i++) {
            ESP_LOGI(TAG, "  [%d]: %u", i, output->data.uint8[i]);
        }
    } else if (output->type == kTfLiteFloat32) {
        for (int i = 0; i < output_size_ && i < 10; i++) {
            ESP_LOGI(TAG, "  [%d]: %.6f", i, output->data.f[i]);
        }
    }
}

void ModelHandler::debug_qat_model_output() const {
    // Basic implementation for QAT debugging
    TfLiteTensor* output = output_tensor();
    if (!output) return;
    debug_raw_outputs(output);
}

std::vector<ModelConfig> ModelHandler::generate_debug_configs() const {
    std::vector<ModelConfig> configs;
    std::vector<std::string> input_orders = {"BGR", "RGB"};
    std::vector<std::pair<int, int>> input_sizes = {{32, 20}, {20, 32}};
    std::vector<bool> normalize_options = {true, false};
    std::vector<std::string> input_types = {"float32", "uint8"};
    std::vector<std::string> output_processings = {
        "softmax", "direct_class", "logits", "experimental_scale"
    };
    
    for (const auto& order : input_orders) {
        for (const auto& size : input_sizes) {
            for (bool norm : normalize_options) {
                for (const auto& type : input_types) {
                    for (const auto& proc : output_processings) {
                        ModelConfig config;
                        config.description = "auto_debug";
                        config.input_order = order;
                        config.input_size.assign({size.first, size.second});
                        config.normalize = norm;
                        config.input_type = type;
                        config.output_processing = proc;
                        config.scale_factor = 10.0f;
                        config.input_channels = 3;
                        configs.push_back(config);
                    }
                }
            }
        }
    }
    return configs;
}

bool ModelHandler::invoke_model(const uint8_t* data, size_t len) {
    if (!interpreter_) return false;
    TfLiteTensor* input = input_tensor();
    if (!input) return false;
    
    // Simple copy for now - assuming size matches
    // In real usage, this should duplicate the preprocessing logic (resize/norm)
    // but for debugging we assume 'data' is already a prepared buffer if possible,
    // OR we just copy raw bytes and let the chips fall where they may.
    // Given 'debug_test_parameters' passes raw zone buffers (likely RGB888),
    // and we are testing different interpretation configs...
    // We should copy MAX(len, input->bytes).
    
    size_t copy_len = std::min(len, input->bytes);
    if (input->type == kTfLiteFloat32) {
        // Naive float conversion for testing
        float* f = input->data.f;
        for (size_t i = 0; i < copy_len && i < input->bytes/4; i++) {
             f[i] = (data[i] / 255.0f); // Blind normalization
        }
    } else {
        memcpy(input->data.uint8, data, copy_len);
    }
    
    return (interpreter_->Invoke() == kTfLiteOk);
}

void ModelHandler::test_configuration(const ModelConfig& config, 
                                    const std::vector<std::vector<uint8_t>>& zone_data,
                                    std::vector<ConfigTestResult>& results) {
    ESP_LOGI(TAG, "Testing Config: Order=%s Size=%dx%d Norm=%d Type=%s Proc=%s",
             config.input_order.c_str(), config.input_size[0], config.input_size[1],
             config.normalize, config.input_type.c_str(), config.output_processing.c_str());
             
    ModelConfig original = config_;
    // Hack: we modify internal config to affect how output is processed
    // But we cannot easily re-allocate the model input tensor size without Init.
    // So 'input_size' changes here might be ignored by the actual interpreter 
    // unless we re-allocate. 
    // For now, valid tests are only those that match the CURRENT model input size.
    // If input size differs, invoke might fail or be skipped.
    
    ConfigTestResult res;
    res.config = config;
    
    // Only test if input size matches model expectation
    if (get_input_width() != config.input_size[0] || get_input_height() != config.input_size[1]) {
        ESP_LOGD(TAG, "Skipping config due to size mismatch with loaded model");
        return; 
    }
    
    // config_ = config; // Dangerous if not thread safe or if methods rely on it
    // Actually, process_output relies on config_.
    const_cast<ModelHandler*>(this)->config_ = config;
    
    float total_conf = 0;
    int success = 0;
    
    for (const auto& zone : zone_data) {
        if (invoke_model(zone.data(), zone.size())) {
            // Get output tensor manually
            TfLiteTensor* output = output_tensor();
            if (output->type == kTfLiteFloat32) {
                ProcessedOutput out = process_output(output->data.f);
                res.zone_confidences.push_back(out.confidence);
                res.zone_values.push_back(out.value);
                total_conf += out.confidence;
                success++;
            }
        }
    }
    
    if (success > 0) res.avg_confidence = total_conf / success;
    results.push_back(res);
    
    const_cast<ModelHandler*>(this)->config_ = original;
}

void ModelHandler::debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data) {
    ESP_LOGI(TAG, "=== DEBUG PARAMETER TESTING ===");
    feed_watchdog();
    
    std::vector<ConfigTestResult> results;
    auto configs = generate_debug_configs();
    
    int i = 0;
    for (const auto& cfg : configs) {
        test_configuration(cfg, zone_data, results);
        if (++i % 5 == 0) feed_watchdog();
    }
    
    std::sort(results.begin(), results.end(), 
        [](const ConfigTestResult& a, const ConfigTestResult& b) {
            return a.avg_confidence > b.avg_confidence;
        });
        
    ESP_LOGI(TAG, "=== TOP CONFIGURATIONS ===");
    for (int j = 0; j < std::min((size_t)10, results.size()); j++) {
        const auto& r = results[j];
        ESP_LOGI(TAG, "#%d: Conf=%.4f [%s %dx%d %s %s]", j+1, r.avg_confidence,
                 r.config.input_order.c_str(), r.config.input_size[0], r.config.input_size[1],
                 r.config.input_type.c_str(), r.config.output_processing.c_str());
    }
}

void ModelHandler::feed_watchdog() {
#ifdef ESP_PLATFORM
    esp_task_wdt_reset();
#endif
    ESP_LOGV(TAG, "Watchdog fed");
}

}  // namespace tflite_micro_helper
}  // namespace esphome
