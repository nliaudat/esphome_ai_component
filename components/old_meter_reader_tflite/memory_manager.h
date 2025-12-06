// memory_manager.h
#pragma once

#include <memory>
#include <cstdint>
#include "esp_heap_caps.h"

namespace esphome {
namespace meter_reader_tflite {

class MemoryManager {
 public:
  struct AllocationResult {
    struct HeapCapsDeleter {
      void operator()(uint8_t* p) const {
        if (p) heap_caps_free(p);
      }
    };
    
    std::unique_ptr<uint8_t[], HeapCapsDeleter> data;
    size_t actual_size;
    
    operator bool() const { return static_cast<bool>(data); }
  };

  static AllocationResult allocate_tensor_arena(size_t requested_size);
  static void report_memory_status(size_t requested_size, 
                                 size_t allocated_size,
                                 size_t peak_usage,
                                 size_t model_size);
};


}  // namespace meter_reader_tflite
}  // namespace esphome