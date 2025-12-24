#include "buffer_pool.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *TAG = "BufferPool";
static constexpr float OVERSIZE_THRESHOLD_FACTOR = 1.2f;

BufferPool::Buffer BufferPool::acquire(size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Strategy 1: Search for exact size match
  for (auto& slot : pool_) {
    if (!slot.in_use && slot.size == size) {
      slot.in_use = true;
      hits_++;
      ESP_LOGV(TAG, "Pool hit: exact match %zu bytes", size);
      return {slot.data, size, true};
    }
  }
  
  // Strategy 2: Try to reuse oversized slot (within 20% overhead)
  for (auto& slot : pool_) {
    if (!slot.in_use && slot.size >= size && slot.size <= size * OVERSIZE_THRESHOLD_FACTOR) {
      slot.in_use = true;
      hits_++;
      ESP_LOGV(TAG, "Pool hit: reusing %zu bytes for %zu bytes", slot.size, size);
      return {slot.data, slot.size, true};
    }
  }
  
  // Strategy 3: Allocate new buffer
  misses_++;
  
  // Prefer SPIRAM for buffers >1KB
  uint8_t* data = nullptr;
  if (size > 1024) {
    data = (uint8_t*)heap_caps_aligned_alloc(64, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  
  // Fallback to internal RAM
  if (!data) {
    data = (uint8_t*)heap_caps_aligned_alloc(64, size, MALLOC_CAP_8BIT);
  }
  
  if (!data) {
    ESP_LOGE(TAG, "Failed to allocate %zu bytes", size);
    return {nullptr, 0, false};
  }
  
  // Try to add to pool if space available
  if (pool_.size() < MAX_POOL_ENTRIES) {
    pool_.push_back({data, size, true});
    ESP_LOGD(TAG, "Pool miss: allocated %zu bytes, added to pool (size: %zu)", size, pool_.size());
    return {data, size, true};
  }
  
  // Pool full, return non-pooled buffer
  saturation_misses_++;
  ESP_LOGW(TAG, "Pool miss: allocated %zu bytes (pool full)", size);
  return {data, size, false};
}

void BufferPool::release(Buffer& buffer) {
  if (!buffer.data) return;
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Find in pool and mark available
  for (auto& slot : pool_) {
    if (slot.data == buffer.data) {
      slot.in_use = false;
      buffer.data = nullptr;
      ESP_LOGV(TAG, "Buffer returned to pool (%zu bytes)", buffer.size);
      return;
    }
  }
  
  // Not in pool, free it
  heap_caps_free(buffer.data);
  buffer.data = nullptr;
  ESP_LOGV(TAG, "Non-pooled buffer freed (%zu bytes)", buffer.size);
}

size_t BufferPool::get_hit_rate() const {
  size_t h = hits_.load();
  size_t total = h + misses_.load();
  return total > 0 ? (100 * h / total) : 0;
}

size_t BufferPool::get_total_allocations() const {
  return hits_.load() + misses_.load();
}

size_t BufferPool::get_pool_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pool_.size();
}

size_t BufferPool::get_saturation_misses() const {
  return saturation_misses_.load();
}

}  // namespace esp32_camera_utils
}  // namespace esphome
