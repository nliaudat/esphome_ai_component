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

void TFLiteCoordinator::unload_model() {
    ESP_LOGI(TAG, "Unloading TFLite model and freeing arena");
    {
        std::lock_guard<std::mutex> lock(model_mutex_);
        model_handler_.unload();
        model_loaded_ = false;
    }
    // Clear arena stats outside model_mutex_ to avoid ABBA deadlock
    // (get_arena_stats holds arena_stats_mutex_ then may wait for model_mutex_ via run_inference)
    {
        std::lock_guard<std::mutex> stats_lock(arena_stats_mutex_);
        cached_arena_stats_ = ArenaStats{};
    }
}

bool TFLiteCoordinator::load_model() {
    std::lock_guard<std::mutex> lock(model_mutex_);
    ESP_LOGI(TAG, "Loading TFLite model...");

    // Build ModelConfig from dynamically-set member variables
    // (set from __init__.py at build time via individual setters)
    tflite_micro_helper::ModelConfig config;
    config.description = model_type_;
    config.input_type = input_type_;
    config.input_channels = input_channels_;
    config.input_size = {input_width_, input_height_};
    config.output_processing = output_processing_;
    config.scale_factor = scale_factor_;
    config.input_order = input_order_;
    config.normalize = normalize_;
    config.invert = invert_;
    // tensor_arena_size is handled directly via tensor_arena_size_requested_
    config.tensor_arena_size = "";  // Not used - arena size is set directly

    ESP_LOGI(TAG, "Using model config: %s", config.description.c_str());
    ESP_LOGI(TAG, "  Input: %s %dx%dx%d", input_type_.c_str(), input_width_, input_height_, input_channels_);
    ESP_LOGI(TAG, "  Output: %s (scale=%.1f)", output_processing_.c_str(), scale_factor_);
    ESP_LOGI(TAG, "  Order: %s, normalize=%d, invert=%d", input_order_.c_str(), normalize_, invert_);
    ESP_LOGI(TAG, "  Tensor arena: %zu bytes", tensor_arena_size_requested_);

    if (!allocate_tensor_arena()) {
        return false;
    }

    if (!model_handler_.load_model_with_arena(model_, model_length_, 
                                 tensor_arena_allocation_.data.get(), 
                                 tensor_arena_allocation_.actual_size,
                                 config)) {
        ESP_LOGE(TAG, "Failed to load model into interpreter");
        ESP_LOGE(TAG, "  Model type: %s", model_type_.c_str());
        ESP_LOGE(TAG, "  Model config: %s", config.description.c_str());
        ESP_LOGE(TAG, "  Model size: %zu bytes", model_length_);
        ESP_LOGE(TAG, "  Tensor arena: %zu bytes (requested: %zu)", 
                 tensor_arena_allocation_.actual_size, tensor_arena_size_requested_);
        ESP_LOGE(TAG, "  Input type from config: %s", config.input_type.c_str());
        ESP_LOGE(TAG, "  Input channels: %d", config.input_channels);
        ESP_LOGE(TAG, "  Input size: %dx%d", 
                 config.input_size.size() >= 1 ? config.input_size[0] : 0,
                 config.input_size.size() >= 2 ? config.input_size[1] : 0);
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
      std::span<const esp32_camera_utils::ImageProcessor::ProcessResult> processed_zones) {
    
    std::lock_guard<std::mutex> lock(model_mutex_);
    
    std::vector<InferenceResult> results;
    if (!model_loaded_) {
        ESP_LOGE(TAG, "Cannot run inference - Model not ready");
        return results;
    }
    
    if (debug_) {
        ESP_LOGD(TAG, "Running inference on %d zones", static_cast<int>(processed_zones.size()));
    }
    
    for (const auto& zone_result : processed_zones) {
        float val, conf;
        if (process_model_result(zone_result, &val, &conf)) {
             if (debug_) {
                 ESP_LOGD(TAG, "Inference result: val=%.2f, conf=%.2f", val, conf);
             }
             results.push_back({val, conf, true});
        } else {
             if (debug_) {
                 ESP_LOGW(TAG, "Inference failed for a zone");
             }
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
    
    // Verify input tensor type avoids implicit reinterpretation risks if model changes
    if (input->type != kTfLiteUInt8 && input->type != kTfLiteInt8 && input->type != kTfLiteFloat32) {
         ESP_LOGE(TAG, "Unsupported input tensor type: %d", input->type);
         return false;
    }
    
    // Validate result data pointer before memcpy
    if (!result.data) {
         ESP_LOGE(TAG, "Null result data pointer");
         return false;
    }
    const uint8_t* src_data = result.data->get();
    if (!src_data) {
         ESP_LOGE(TAG, "Null result data buffer (shared_ptr is empty)");
         return false;
    }
    
    memcpy(input->data.uint8, src_data, result.size);
    
    if (model_handler_.invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Model invocation failed");
        return false;
    }
    
    // Update arena stats cache after inference (for thread-safe Core 1 access)
    update_arena_stats_cache();
    
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
        // Logging enabled if DEBUG_METER_READER_TFLITE is defined.
        // Useful for debugging specific input patterns when confidence is non-zero but incorrect.
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
    // Use const_cast explicitly (documented) since input_tensor() lacks a const overload
    TfLiteTensor* input = const_cast<tflite_micro_helper::ModelHandler&>(model_handler_).input_tensor();
    
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

// Thread-safe: Returns cached arena stats (safe for Core 1 to call while Core 0 runs inference)
TFLiteCoordinator::ArenaStats TFLiteCoordinator::get_arena_stats() const {
    std::lock_guard<std::mutex> lock(arena_stats_mutex_);
    return cached_arena_stats_;
}

// Call this after inference completes (from Core 0 inference task)
void TFLiteCoordinator::update_arena_stats_cache() {
    ArenaStats stats;
    stats.total_size = tensor_arena_allocation_.actual_size;
    stats.used_bytes = model_handler_.get_arena_used_bytes();
    stats.wasted_bytes = (stats.total_size > stats.used_bytes) ? (stats.total_size - stats.used_bytes) : 0;
    stats.efficiency = (stats.total_size > 0) ? (100.0f * stats.used_bytes / stats.total_size) : 0.0f;
    
    std::lock_guard<std::mutex> lock(arena_stats_mutex_);
    cached_arena_stats_ = stats;
}

void TFLiteCoordinator::debug_test_parameters(const std::vector<std::vector<uint8_t>>& zone_data) {
    model_handler_.debug_test_parameters(zone_data);
}

} // namespace meter_reader_tflite
} // namespace esphome
