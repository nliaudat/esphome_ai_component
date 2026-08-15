#include "buffer_pool.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *TAG = "BufferPool";
static constexpr float OVERSIZE_THRESHOLD_FACTOR = 1.2f;

BufferPool::Buffer BufferPool::acquire(size_t size) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  // Strategy 1: Search for exact size match
  for (auto &slot : this->pool_) {
    if (!slot.in_use && slot.size == size) {
      slot.in_use = true;
      this->hits_++;
      ESP_LOGV(TAG, "Pool hit: exact match %zu bytes", size);
      return {slot.data, size, true};
    }
  }

  // Strategy 2: Try to reuse oversized slot (within 20% overhead)
  for (auto &slot : this->pool_) {
    if (!slot.in_use && slot.size >= size && slot.size <= size * OVERSIZE_THRESHOLD_FACTOR) {
      slot.in_use = true;
      this->hits_++;
      ESP_LOGV(TAG, "Pool hit: reusing %zu bytes for %zu bytes", slot.size, size);
      return {slot.data, slot.size, true};
    }
  }

  // Strategy 3: Allocate new buffer
  this->misses_++;

  // Prefer SPIRAM for buffers >1KB
  uint8_t *data = nullptr;
  if (size > 1024) {
    data = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }

  // Fallback to internal RAM
  if (!data) {
    data = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, size, MALLOC_CAP_8BIT));
  }

  if (!data) {
    ESP_LOGE(TAG, "Failed to allocate %zu bytes", size);
    return {nullptr, 0, false};
  }

  // Track which allocator path actually succeeded (debug aid)
  if (size > 1024) {
    this->psram_allocations_.fetch_add(1);
  } else {
    this->sram_allocations_.fetch_add(1);
  }

  // Try to add to pool if space available
  if (this->pool_.size() < MAX_POOL_ENTRIES) {
    this->pool_.push_back({data, size, true});
    ESP_LOGD(TAG, "Pool miss: allocated %zu bytes, added to pool (size: %zu)", size, this->pool_.size());
    return {data, size, true};
  }

  // Pool full, return non-pooled buffer
  this->saturation_misses_++;
  ESP_LOGW(TAG, "Pool miss: allocated %zu bytes (pool full)", size);
  return {data, size, false};
}

BufferPool::~BufferPool() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  for (auto &slot : this->pool_) {
    if (slot.data) {
      heap_caps_free(slot.data);
      slot.data = nullptr;
      slot.size = 0;
      slot.in_use = false;
    }
  }
  this->pool_.clear();
}

void BufferPool::release(Buffer &buffer) {
  if (!buffer.data)
    return;

  std::lock_guard<std::mutex> lock(this->mutex_);

  const size_t buffer_size = buffer.size;

  // Find in pool and mark available
  for (auto &slot : this->pool_) {
    if (slot.data == buffer.data) {
      slot.in_use = false;
      buffer.data = nullptr;
      buffer.size = 0;
      buffer.from_pool = false;
      ESP_LOGV(TAG, "Buffer returned to pool (%zu bytes)", buffer_size);
      return;
    }
  }

  // Not in pool, free it
  heap_caps_free(buffer.data);
  buffer.data = nullptr;
  buffer.size = 0;
  buffer.from_pool = false;
  ESP_LOGV(TAG, "Non-pooled buffer freed (%zu bytes)", buffer_size);
}

size_t BufferPool::get_hit_rate() const {
  size_t h = this->hits_.load();
  size_t total = h + this->misses_.load();
  return total > 0 ? (100 * h / total) : 0;
}

size_t BufferPool::get_total_allocations() const { return this->hits_.load() + this->misses_.load(); }

size_t BufferPool::get_pool_size() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->pool_.size();
}

size_t BufferPool::get_saturation_misses() const { return this->saturation_misses_.load(); }

size_t BufferPool::get_psram_allocations() const { return this->psram_allocations_.load(); }

size_t BufferPool::get_sram_allocations() const { return this->sram_allocations_.load(); }

#ifdef DEBUG_ESP32_CAMERA_UTILS
void BufferPool::report_statistics() const {
  size_t h = this->hits_.load();
  size_t m = this->misses_.load();
  size_t total = h + m;
  float hit_rate = total > 0 ? (100.0f * h / total) : 0.0f;
  ESP_LOGI(TAG, "BufferPool stats - Hits: %zu, Misses: %zu, Hit rate: %.1f%%", h, m, hit_rate);
  ESP_LOGI(TAG, "Pool size: %zu, Saturation misses: %zu, PSRAM allocs: %zu, SRAM allocs: %zu", this->get_pool_size(),
           this->saturation_misses_.load(), this->psram_allocations_.load(), this->sram_allocations_.load());
}
#endif  // DEBUG_ESP32_CAMERA_UTILS

}  // namespace esp32_camera_utils
}  // namespace esphome
