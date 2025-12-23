#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <atomic>
#include "esp_heap_caps.h"

namespace esphome {
namespace esp32_camera_utils {

/**
 * @brief Thread-safe buffer pool for frequently allocated fixed-size buffers.
 * 
 * Reduces heap allocation churn by reusing buffers for model inputs.
 * Automatically expands up to MAX_POOL_ENTRIES as needed.
 */
class BufferPool {
 public:
  struct Buffer {
    uint8_t* data{nullptr};
    size_t size{0};
    bool from_pool{false};  // Track origin for statistics
  };

  /**
   * @brief Acquire a buffer from the pool or allocate a new one.
   * @param size Required buffer size in bytes
   * @return Buffer structure (check data != nullptr for success)
   */
  [[nodiscard]] Buffer acquire(size_t size);
  
  /**
   * @brief Return a buffer to the pool or free it.
   * @param buffer Buffer to release (will be reset to nullptr)
   */
  void release(Buffer& buffer);
  
  /**
   * @brief Get pool hit rate as percentage.
   * @return Hit rate 0-100%
   */
  size_t get_hit_rate() const;
  
  /**
   * @brief Get total number of allocations requested.
   * @return Total allocations (hits + misses)
   */
  size_t get_total_allocations() const;
  
  /**
   * @brief Get current pool size.
   * @return Number of buffers currently in pool
   */
  size_t get_pool_size() const;

 private:
  struct PoolSlot {
    uint8_t* data{nullptr};
    size_t size{0};
    bool in_use{false};
  };
  
  static constexpr size_t MAX_POOL_ENTRIES = 8;
  std::vector<PoolSlot> pool_;
  mutable std::mutex mutex_;
  
  std::atomic<size_t> hits_{0};
  std::atomic<size_t> misses_{0};
};

}  // namespace esp32_camera_utils
}  // namespace esphome
