#include "tflite_micro_helper.h"
#include <cstdlib>

namespace esphome {
namespace tflite_micro_helper {

static const char *const TAG = "tflite_micro_helper";

bool TFLiteMicroHelper::load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config) {
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
        // Fallback: Use default 100KB if parsing fails and no valid size was previously set.
        if (tensor_arena_size_requested_ == 0) {
             tensor_arena_size_requested_ = 100 * 1024; // Default 100KB if parsing fails and no previous size
             ESP_LOGW(TAG, "Using default tensor arena size: 100KB");
        }
    }

    // Allocate tensor arena
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }

    // Load the model
    if (!model_handler_.load_model_with_arena(model_data, model_size, 
                                 tensor_arena_allocation_.data.get(), 
                                 tensor_arena_allocation_.actual_size,
                                 config)) {
        ESP_LOGE(TAG, "Failed to load model into interpreter");
        return false;
    }

    if (debug_) {
        ESP_LOGD(TAG, "Model loaded successfully. Arena used: %zu / %zu bytes", 
                 model_handler_.get_arena_used_bytes(), tensor_arena_allocation_.actual_size);
    }
    return true;
}

}  // namespace tflite_micro_helper
}  // namespace esphome
