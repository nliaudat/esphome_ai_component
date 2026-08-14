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
    uint8_t *data{nullptr};
    size_t size{0};
    bool from_pool{false};  // Track origin for statistics
  };

  /// @brief Free all pooled slot buffers (RAII cleanup).
  ~BufferPool();

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
  void release(Buffer &buffer);

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
   * @brief Get number of heap allocations caused by pool saturation.
   * @return Number of misses due to MAX_POOL_ENTRIES reached
   */
  size_t get_saturation_misses() const;

  /**
   * @brief Get current pool size.
   * @return Number of buffers currently in pool
   */
  size_t get_pool_size() const;

  /**
   * @brief Get number of allocations satisfied from PSRAM.
   * @return Count of PSRAM allocations (misses)
   */
  size_t get_psram_allocations() const;

  /**
   * @brief Get number of allocations satisfied from internal SRAM.
   * @return Count of SRAM allocations (misses)
   */
  size_t get_sram_allocations() const;

#ifdef DEBUG_ESP32_CAMERA_UTILS
  /**
   * @brief Log buffer pool statistics (hit rate, pool size, allocation sources).
   * Debug-gated: compiled out of release builds to save flash.
   */
  void report_statistics() const;
#endif

 private:
  struct PoolSlot {
    uint8_t *data{nullptr};
    size_t size{0};
    bool in_use{false};
  };

  static constexpr size_t MAX_POOL_ENTRIES = 16;
  std::vector<PoolSlot> pool_;
  mutable std::mutex mutex_;

  std::atomic<size_t> hits_{0};
  std::atomic<size_t> misses_{0};
  std::atomic<size_t> saturation_misses_{0};
  std::atomic<size_t> psram_allocations_{0};
  std::atomic<size_t> sram_allocations_{0};
};

}  // namespace esp32_camera_utils
}  // namespace esphome
