# ESP32 Camera Utils - Implementation Plan

**Date:** 2026-08-14 (v2 -- reviewed against current source)

**Component Status:** Active

## 1. EXECUTIVE SUMMARY

The `esp32_camera_utils` provides core image processing functionality including buffer management,
image cropping, rotation, and windowing controls.

**Review note (2026-08-14):** v1 of this plan described an older code state. The four "manual `new`
memory-leak critical" claims (§2.2, §3.1, §3.2, §3.3) are FALSE in the current source -- there are
no raw `new` calls in the buffer pool or crop zone handler, and the only `new` calls in
`image_processor.cpp` are immediately RAII-wrapped. This revision removes those, keeps the one real
(but low-severity) gap -- the missing `BufferPool` destructor -- plus minor hygiene/debug items.

## 2. CURRENT STATE ANALYSIS

### 2.1 Strengths

- Implements `TrackedBuffer` pattern for JPEG memory management
- Supports both JPEG and RAW (RGB565/GRAYSCALE) formats
- Thread-safe buffer pool implementation with an atomic-counter statistics layer
- `BufferPool::acquire()` uses `heap_caps_aligned_alloc()` with SPIRAM-first then internal-RAM
  fallback (`buffer_pool.cpp:37-45`) -- no raw `new`
- `TrackedBuffer` RAII destructor returns pooled buffers or frees via `jpeg_free_align` /
  `heap_caps_free` (`image_processor.h:109-122`)

### 2.2 Issues Found (still open)

| Issue | Location | Severity |
|-------|----------|----------|
| `BufferPool` has no destructor -- pooled slot buffers (`PoolSlot::data`) are never freed en-masse | `buffer_pool.h` (no `~BufferPool`) | Low-Medium (long-lived static component, but a real RAII gap) |
| `release()` does not reset `buffer.size` / `buffer.from_pool` after handing the buffer back | `buffer_pool.cpp:65-85` | Low (hygiene) |
| No `report_statistics()` logging helper | N/A | Low (debug aid) |
| No PSRAM-vs-SRAM allocation counters | `buffer_pool.cpp` | Low (debug aid) |

### 2.3 Previously-Reported Items -- RESOLVED / FALSE

| v1 Item | Status | Evidence |
|---------|--------|----------|
| "Manual `new` in Buffer struct" (`buffer_pool.h:27`) | **FALSE** | `Buffer` is `{ uint8_t *data; size_t size; bool from_pool; }` -- a plain holder, no allocation (`buffer_pool.h:20-24`) |
| "Manual `new` in acquire()" (`buffer_pool.cpp:33`) | **FALSE** | `acquire()` uses `heap_caps_aligned_alloc()` with SPIRAM-first fallback (`buffer_pool.cpp:37-45`) |
| "Manual `new` in constructor" (`crop_zone_handler.h:44`) | **FALSE** | `CropZoneHandler` has no constructor and no `new`; `zones_` is a value `std::vector<CropZone>` (`crop_zone_handler.h:100`) |
| "7 manual `new` calls in image_processor.cpp (lines 342, 375, 502, 534, 804, 1135, 1176)" | **FALSE** | Only 3 real `new`s exist: `new TrackedBuffer(...)` and `new RotatedPreviewImage(...)`, each immediately wrapped in `UniqueBufferPtr` / `std::shared_ptr` (RAII). The cited line numbers do not match |
| "Arena stats / pool stats not implemented" | **PARTIALLY FALSE** | `get_hit_rate()`, `get_total_allocations()`, `get_saturation_misses()`, `get_pool_size()` already exist (`buffer_pool.cpp:87-100`). Only `report_statistics()` and PSRAM/SRAM counters are missing |

**Note:** v1's §3.1 proposed snippet is not a valid drop-in (it mixes `unique_ptr` data with raw
`PoolSlot.data`, discards the size-match search, and wouldn't compile as written). v1's §3.2 proposal
to move `std::vector<CropZone> zones_` behind a `std::unique_ptr` would be a REGRESSION (adds a heap
indirection with zero benefit). v1's §5.1 sketch references a non-existent `slot.last_used` field.

## 3. BUG FIXES

### 3.1 Add `BufferPool` Destructor

**Problem:** `BufferPool::pool_` holds raw `uint8_t *data` pointers that are only freed when a
buffer is released as non-pooled (saturation path). All buffers cached in the pool are never
freed unless the pool object is destroyed -- and there is no destructor.

**Solution:** declare `~BufferPool();` and implement it to free every pooled slot:

```cpp
// buffer_pool.h (public section)
~BufferPool();

// buffer_pool.cpp
BufferPool::~BufferPool() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  for (auto &slot : this->pool_) {
    if (slot.data) {
      heap_caps_free(slot.data);
      slot.data = nullptr;
    }
  }
  this->pool_.clear();
}
```

Design note: `BufferPool` is the static `ImageProcessor::buffer_pool_`. TrackedBuffer instances
return pooled buffers to the pool on destruction (`image_processor.h:109-122`); the destructor
above is the pool's own RAII cleanup and must only run at teardown once no `TrackedBuffer` holds a
pooled pointer (true in ESPHome shutdown order). The pool is never copied or moved, so adding a
user-declared destructor has no lifetime impact.

### 3.2 `release()` Buffer Hygiene

**Problem:** `release()` nulls `buffer.data` but leaves `buffer.size` and `buffer.from_pool`
stale.

**Solution:** reset all three fields:

```cpp
void BufferPool::release(Buffer &buffer) {
  if (!buffer.data)
    return;

  std::lock_guard<std::mutex> lock(this->mutex_);

  for (auto &slot : this->pool_) {
    if (slot.data == buffer.data) {
      slot.in_use = false;
      buffer.data = nullptr;
      buffer.size = 0;
      buffer.from_pool = false;
      ESP_LOGV(TAG, "Buffer returned to pool (%zu bytes)", buffer.size);
      return;
    }
  }

  heap_caps_free(buffer.data);
  buffer.data = nullptr;
  buffer.size = 0;
  buffer.from_pool = false;
  ESP_LOGV(TAG, "Non-pooled buffer freed (%zu bytes)", buffer.size);
}
```

## 4. MEMORY LEAK PREVENTION

### 4.1 Buffer Pool Statistics and Monitoring (complete the partially-implemented layer)

`get_hit_rate()`, `get_total_allocations()`, `get_saturation_misses()`, and `get_pool_size()`
already exist. Add the two missing pieces:

```cpp
// Accessors
size_t get_psram_allocations() const;
size_t get_sram_allocations() const;

// Logging helper
void report_statistics() const;

// buffer_pool.cpp
void BufferPool::report_statistics() const {
  size_t h = this->hits_.load();
  size_t m = this->misses_.load();
  size_t total = h + m;
  float hit_rate = total > 0 ? (100.0f * h / total) : 0.0f;
  ESP_LOGI(TAG, "BufferPool stats - Hits: %zu, Misses: %zu, Hit rate: %.1f%%", h, m, hit_rate);
  ESP_LOGI(TAG, "Pool size: %zu, Saturation misses: %zu, PSRAM allocs: %zu, SRAM allocs: %zu",
           this->get_pool_size(), this->saturation_misses_.load(), this->psram_allocations_.load(),
           this->sram_allocations_.load());
}
```

`acquire()` is instrumented to count which allocator path succeeded (SPIRAM vs internal RAM)
as atomic counters.

## 5. ENHANCEMENTS

None pending beyond §4. v1's §5.1 "Automatic Buffer Recycling" and §5.2 "PSRAM Usage Statistics"
are superseded: recycling is not needed for the short-lived model-input buffers this pool serves,
and the statistics are covered by §4.1. Any future recycling work must use a proper `last_used`
timestamp field (not the non-existent one from v1).

## 6. TESTING REQUIREMENTS

### 6.1 Unit Tests

- [ ] Test `acquire()` returns pooled buffer when available
- [ ] Test `acquire()` allocates new buffer when pool exhausted
- [ ] Test `release()` properly frees non-pooled buffer and resets `size`/`from_pool`
- [ ] Test destructor frees all pooled slots
- [ ] Test thread-safety under concurrent access

### 6.2 Memory Tests

- [ ] Verify no memory leaks with Valgrind/ESP-MD5
- [ ] Test buffer pool saturation behavior
- [ ] Test PSRAM vs SRAM allocation paths (and counters)
- [ ] Stress test: rapid acquire/release cycles

## 7. PRIORITY MATRIX

| Priority | Item | Reason |
|----------|------|--------|
| **P1** | Add `BufferPool` destructor (§3.1) | Fixes the only real leak path (pooled slots never freed en-masse) |
| **P2** | `release()` resets all fields (§3.2) | Hygiene, prevents stale-state bugs |
| **P2** | `report_statistics()` + PSRAM/SRAM counters (§4.1) | Debugging aid |

(Removed: v1's P0/P1 "manual new" items -- false, see 2.3.)

## 8. FILES TO MODIFY

| File | Changes |
|------|---------|
| `buffer_pool.h` | Declare `~BufferPool()`, `report_statistics()`, PSRAM/SRAM accessors and atomic counters |
| `buffer_pool.cpp` | Implement destructor, `release()` hygiene, allocation counters in `acquire()`, `report_statistics()` |