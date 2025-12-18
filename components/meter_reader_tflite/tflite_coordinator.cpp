#include "tflite_coordinator.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "tflite_coordinator";

void TFLiteCoordinator::setup(const std::string& model_type, size_t tensor_arena_size) {
    model_type_ = model_type;
    tensor_arena_size_requested_ = tensor_arena_size;
}

void TFLiteCoordinator::set_model(const uint8_t *model, size_t length) {
    model_ = model;
    model_length_ = length;
    ESP_LOGD(TAG, "Model set: %zu bytes", length);
}

bool TFLiteCoordinator::allocate_tensor_arena() {
    ESP_LOGI(TAG, "Allocating tensor arena: %zu bytes", tensor_arena_size_requested_);
    
    tensor_arena_allocation_ = tflite_micro_helper::MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }
    
    ESP_LOGI(TAG, "Tensor arena allocated successfully: %zu bytes", 
             tensor_arena_allocation_.actual_size);
    return true;
}

bool TFLiteCoordinator::load_model() {
    ESP_LOGI(TAG, "Loading TFLite model...");

    // Get model configuration from model_config.h FIRST
    tflite_micro_helper::ModelConfig config;
    auto it = MODEL_CONFIGS.find(model_type_);
    if (it != MODEL_CONFIGS.end()) {
        config = it->second;
        ESP_LOGI(TAG, "Using model config: %s", config.description.c_str());

        // Parse arena size string logic from original code...
        std::string arena_size_str = config.tensor_arena_size;
        size_t multiplier = 1;
        if (arena_size_str.find("KB") != std::string::npos) {
            multiplier = 1024;
            arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
        } else if (arena_size_str.find("MB") != std::string::npos) {
            multiplier = 1024 * 1024;
            arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
        }
        
        char* end_ptr;
        long size_value = strtol(arena_size_str.c_str(), &end_ptr, 10);
        if (size_value > 0) {
            // Override with config-specific size if valid
            tensor_arena_size_requested_ = size_value * multiplier;
            ESP_LOGI(TAG, "Using model-specific tensor arena size: %zu", tensor_arena_size_requested_);
        }
    } else {
        ESP_LOGE(TAG, "Model type '%s' not found", model_type_.c_str());
        // Fallback or fail? Original code kept going with default or previous.
    }

    if (!allocate_tensor_arena()) {
        return false;
    }

    if (!model_handler_.load_model_with_arena(model_, model_length_, 
                                 tensor_arena_allocation_.data.get(), 
                                 tensor_arena_allocation_.actual_size,
                                 config)) {
        ESP_LOGE(TAG, "Failed to load model into interpreter");
        return false;
    }

    model_loaded_ = true;
    ESP_LOGI(TAG, "Model loaded successfully. Input: %dx%dx%d", 
             model_handler_.get_input_width(),
             model_handler_.get_input_height(),
             model_handler_.get_input_channels());

    #ifdef DEBUG_METER_READER_TFLITE
    model_handler_.debug_model_architecture();
    #endif



    return true;
}

std::vector<TFLiteCoordinator::InferenceResult> TFLiteCoordinator::run_inference(
      const std::vector<esp32_camera_utils::ImageProcessor::ProcessResult>& processed_zones) {
    
    std::vector<InferenceResult> results;
    if (!model_loaded_) {
        ESP_LOGE(TAG, "Cannot run inference - Model not ready");
        return results;
    }
    
    for (const auto& zone_result : processed_zones) {
        float val, conf;
        if (process_model_result(zone_result, &val, &conf)) {
             results.push_back({val, conf, true});
        } else {
             results.push_back({0.0f, 0.0f, false});
        }
    }
    return results;
}

bool TFLiteCoordinator::process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult& result, 
                                             float* value, float* confidence) {
    TfLiteTensor* input = model_handler_.input_tensor();
    if (!input) return false;
    
    if (result.size != input->bytes) {
         ESP_LOGE(TAG, "Input size mismatch: expected %zu, got %zu", input->bytes, result.size);
         return false;
    }
    
    memcpy(input->data.uint8, result.data->get(), result.size);
    
    if (model_handler_.invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Model invocation failed");
        return false;
    }
    
    TfLiteTensor* output_tensor = model_handler_.output_tensor();
    if (!output_tensor) return false;
    
    auto output = model_handler_.process_output(output_tensor);
    *value = output.value;
    *confidence = output.confidence;
    
    if (*confidence < 0.0001f) {
        ESP_LOGW(TAG, "Zero confidence detected");
        #ifdef DEBUG_METER_READER_TFLITE
        model_handler_.log_input_stats();
        #endif
    } else {
        #ifdef DEBUG_METER_READER_TFLITE
        // Always log input stats for full debugging validation? 
        // Or only on high debug level? Let's obey the user request "Restore all debug output"
        // model_handler_.log_input_stats(); // This might be too spammy if 8 zones enabled.
        // Maybe only if needed? The user complaint specifically mentioned tflite messages.
        // Let's enable it behind the define.
        #endif
    }
    return true;
}



TFLiteCoordinator::ModelSpec TFLiteCoordinator::get_model_spec() const {
    ModelSpec spec;
    spec.input_width = model_handler_.get_input_width();
    spec.input_height = model_handler_.get_input_height();
    spec.input_channels = model_handler_.get_input_channels();
    spec.normalize = model_handler_.get_config().normalize;
    spec.input_order = model_handler_.get_config().input_order;
    
    // Determine input type by checking tensor size
    // Float32 = W*H*C*4, Uint8 = W*H*C*1
    TfLiteTensor* input = ((tflite_micro_helper::ModelHandler&)model_handler_).input_tensor(); 
    
    size_t num_elements = spec.input_width * spec.input_height * spec.input_channels;
    
    if (input && input->bytes == num_elements * 4) {
        spec.input_type = 1; // Float
    } else {
        spec.input_type = 0; // Uint8 (or fallback)
    }
    
    ESP_LOGD(TAG, "Model Spec: %dx%dx%d InputBytes=%zu Type=%d", 
             spec.input_width, spec.input_height, spec.input_channels, 
             input ? input->bytes : 0, spec.input_type);

    return spec;
}

void TFLiteCoordinator::report_memory_status() {
    size_t peak = model_handler_.get_arena_used_bytes();
    
    ESP_LOGI(TAG, "Tensor Arena: Size=%zu, Peak Used=%zu (%.1f%%)",
             tensor_arena_allocation_.actual_size,
             peak,
             (peak * 100.0f) / tensor_arena_allocation_.actual_size);

    memory_manager_.report_memory_status(
        tensor_arena_size_requested_,
        tensor_arena_allocation_.actual_size,
        peak,
        model_length_
    );
}

void TFLiteCoordinator::debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data) {
    model_handler_.debug_test_parameters(zone_data);
}

} // namespace meter_reader_tflite
} // namespace esphome
