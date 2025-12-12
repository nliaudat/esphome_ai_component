#include "memory_manager.h"
#include "esp_log.h"
#include "debug_utils.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "MemoryManager";

MemoryManager::AllocationResult MemoryManager::allocate_tensor_arena(size_t requested_size) {
  ESP_LOGD(TAG, "Attempting to allocate %zu bytes for tensor arena", requested_size);
  
  AllocationResult result;
  result.actual_size = requested_size;

  // First try PSRAM
  uint8_t *arena_ptr = static_cast<uint8_t*>(
      heap_caps_malloc(requested_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  
  if (!arena_ptr) {
    ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM");
    arena_ptr = static_cast<uint8_t*>(
        heap_caps_malloc(requested_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }

  if (arena_ptr) {
    // Use the HeapCapsDeleter from the AllocationResult struct
    result.data = std::unique_ptr<uint8_t[], AllocationResult::HeapCapsDeleter>(
        arena_ptr, 
        AllocationResult::HeapCapsDeleter());
    
    ESP_LOGD(TAG, "Successfully allocated tensor arena at %p", arena_ptr);
  } else {
    ESP_LOGE(TAG, "Failed to allocate tensor arena");
    result.actual_size = 0;
  }

  return result;
}

void MemoryManager::report_memory_status(size_t requested_size, 
                                       size_t allocated_size,
                                       size_t peak_usage,
                                       size_t model_size) {
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t free_psram = 0;
  
  if (total_psram > 0) {
    free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM: %zuB free of %zuB total (%.1fKB / %.1fKB)", 
             free_psram, total_psram, free_psram / 1024.0f, total_psram / 1024.0f);
  }

  // Calculate total memory usage
  size_t total_memory_usage = model_size + peak_usage;
  
  ESP_LOGI(TAG, "Memory Status (Complete Picture):");
  ESP_LOGI(TAG, "  Model Size: %zuB (%.1fKB)", model_size, model_size / 1024.0f);
  ESP_LOGI(TAG, "  Tensor Arena (Requested): %zuB (%.1fKB)", requested_size, requested_size / 1024.0f);
  ESP_LOGI(TAG, "  Tensor Arena (Allocated): %zuB (%.1fKB)", allocated_size, allocated_size / 1024.0f);
  ESP_LOGI(TAG, "  Arena Peak Usage: %zuB (%.1fKB)", peak_usage, peak_usage / 1024.0f);
  ESP_LOGI(TAG, "  TOTAL Memory Usage: %zuB (%.1fKB)", total_memory_usage, total_memory_usage / 1024.0f);
  ESP_LOGI(TAG, "  Free Internal Heap: %zuB (%.1fKB)", free_internal, free_internal / 1024.0f);

  if (model_size > 0) {
    float arena_model_ratio = static_cast<float>(allocated_size) / model_size;
    float total_model_ratio = static_cast<float>(total_memory_usage) / model_size;
    ESP_LOGI(TAG, "  Arena/Model Ratio: %.1fx", arena_model_ratio);
    ESP_LOGI(TAG, "  Total/Model Ratio: %.1fx", total_model_ratio);
  }
}

static void log_current_memory_usage(const char* context) {
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGD(TAG, "[%s] Free: Internal=%zuB, PSRAM=%zuB", 
             context, free_internal, free_psram);
}

}  // namespace meter_reader_tflite
}  // namespace esphome
