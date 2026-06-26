#include "tflite_micro_helper.h"
#include <cstdlib>

namespace esphome {
namespace tflite_micro_helper {

static const char *const TAG = "tflite_micro_helper";

bool TFLiteMicroHelper::load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config) {
    this->model_length_ = model_size;
    
    // Determine tensor arena size from config (uses shared parser in MemoryManager)
    if (!config.tensor_arena_size.empty()) {
        size_t parsed = MemoryManager::parse_size_string(config.tensor_arena_size);
        if (parsed > 0) {
            this->tensor_arena_size_requested_ = parsed;
            ESP_LOGI(TAG, "Using model-specific tensor arena size: %s (%zu bytes)", 
                    config.tensor_arena_size.c_str(), this->tensor_arena_size_requested_);
        } else {
            ESP_LOGW(TAG, "Failed to parse tensor arena size from config: %s", 
                    config.tensor_arena_size.c_str());
        }
    }
    
    if (this->tensor_arena_size_requested_ == 0) {
        this->tensor_arena_size_requested_ = 100 * 1024;
        ESP_LOGW(TAG, "Using default tensor arena size: 100KB");
    }

    // Defensive check: validate model data before CRC verification
    if (model_data == nullptr || model_size == 0) {
        ESP_LOGE(TAG, "Model data is NULL or empty");
        return false;
    }

    // Verify CRC32 checksum BEFORE allocating tensor arena or parsing model data.
    // This prevents wasted memory allocation and potential UB from parsing corrupt data.
    if (!this->model_handler_.verify_model_crc(model_data, model_size)) {
        ESP_LOGE(TAG, "Model CRC32 verification failed");
        return false;
    }

    // Allocate tensor arena
    this->tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(this->tensor_arena_size_requested_);
    if (!this->tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }

    // Load the model
    if (!this->model_handler_.load_model_with_arena(model_data, model_size, 
                                 this->tensor_arena_allocation_.data.get(), 
                                 this->tensor_arena_allocation_.actual_size,
                                 config)) {
        ESP_LOGE(TAG, "Failed to load model into interpreter");
        return false;
    }

    if (this->debug_) {
        ESP_LOGD(TAG, "Model loaded successfully. Arena used: %zu / %zu bytes", 
                 this->model_handler_.get_arena_used_bytes(), this->tensor_arena_allocation_.actual_size);
    }
    return true;
}

}  // namespace tflite_micro_helper
}  // namespace esphome
