# TFLite Micro Helper - Implementation Plan

**Date:** 2026-08-14 (v2 -- reviewed against current source)

**Component Status:** Active (v2.0+)

## 1. EXECUTIVE SUMMARY

The `tflite_micro_helper` is the central TFLite runtime wrapper for ESPHome AI components.
It provides model loading, tensor arena management, and inference invocation.

**Review note (2026-08-14):** v1 of this plan contained items that are false or already
corrected in the current source (CRC32 verification, arena-stats thread-safety, default
arena size). This revision removes those, keeps the two still-open enhancements, and adds
a newly-discovered binary-search logic bug in `probe_arena_size()`.

## 2. CURRENT STATE ANALYSIS

### 2.1 Strengths

- Uses RAII pattern with `unique_ptr` and custom `HeapCapsDeleter`
- PSRAM-aware allocation via `heap_caps_aligned_alloc(16, ...)`
- Supports build-time override flags (`TFLITE_FORCE_SRAM`, `TFLITE_FORCE_PSRAM`)
- **CRC32 verification implemented** -- `ModelHandler::calculate_crc32()` /
  `verify_model_crc()` (`model_handler.cpp:584-612`), invoked during
  `load_model()` (`tflite_micro_helper.cpp:137`); expected CRC computed per
  instance from `zlib.crc32` (`__init__.py:388-389`)
- Thread-safe arena stats with mutex protection -- `scoped_lock` held on BOTH
  read (`get_arena_stats()`) and write (`update_arena_stats_cache()`)
- State machine for load state (UNLOADED/LOADING/READY) via a single atomic
- **Arena size probing** -- `probe_arena_size()` (`model_handler.cpp:644`)
  measures the real required arena at load time and adjusts
  `tensor_arena_size_requested_` both up and down
  (`tflite_micro_helper.cpp:143-158`)

### 2.2 Issues Found (still open)

| Issue | Location | Severity |
|-------|----------|----------|
| `probe_arena_size()` binary search does not converge to the true minimum (and can construct an interpreter with arena size > allocated buffer) | `model_handler.cpp:687-717` | Medium |
| No heap fragmentation monitoring | `TFLiteMicroHelper::report_memory_status()` | Medium |
| No model loading statistics | `TFLiteMicroHelper::load_model()` | Low |

### 2.3 Previously-Reported Items -- RESOLVED / FALSE

| v1 Item | Status | Evidence |
|---------|--------|----------|
| "No CRC32 verification in current code" | **FALSE** | Implemented (`model_handler.cpp:584-612`), invoked (`tflite_micro_helper.cpp:137`), expected CRC from `zlib.crc32` (`__init__.py:388-389`) |
| "Arena stats cache not thread-safe on write" | **ALREADY FIXED** | `scoped_lock` held before write (`tflite_micro_helper.cpp:279-280`); `get_arena_stats()` and `unload_model()` also lock |
| "Default tensor arena size (100KB) too small" | **MITIGATED** | `probe_arena_size()` measures real requirement at load and adjusts both ways; `.txt` "Recommended tensor_arena_size" auto-config (`__init__.py:112-114`, `486-489`). If a default change is ever pursued, all THREE hard-coded 100KB sites must change: header member init (`tflite_micro_helper.h:161`), `reset_config()` (`tflite_micro_helper.cpp:230`), `ModelHandler::load_model()` (`model_handler.cpp:35`) |

## 3. BUG FIXES

### 3.1 Fix `probe_arena_size()` Binary-Search Logic

**Problem:** The shrink branch does:

```cpp
upper = lower;                  // upper set to lower BEFORE midpoint is computed
lower = ((lower + upper) / 2 + 15) & ~15;   // == lower, never decreases
```

so `lower` never shrinks -- the loop tests the initial bound at most once and the
function returns a *working* size, not the minimum it claims. Additionally, `lower`
(`arena_used_bytes + 16`, aligned) can exceed `attempt_size`; the original code then
constructs an interpreter with `arena_size = lower` on a buffer allocated for only
`attempt_size` bytes -- a potential buffer over-read/write.

**Solution:** standard binary search with a proven-good `upper` bound, plus a clamp
when `lower >= upper`:

```cpp
size_t lower = (probe_interpreter->arena_used_bytes() + 16 + 15) & ~15;
probe_interpreter.reset();
size_t upper = attempt_size;  // proven-good upper bound

if (lower >= upper) {
  // Arena usage already at/above the attempt size; the attempt size is the
  // practical minimum for this bounded probe.
  ESP_LOGI(TAG, "probe_arena_size: arena usage at attempt size; using %zu bytes", upper);
  return upper;
}

// Binary-search down to the smallest proven-working size.
// Invariant: upper is always proven-good.
while (lower < upper) {
  size_t mid = lower + (upper - lower) / 2;
  auto test_interpreter = std::make_unique<tflite::MicroInterpreter>(model, *local_resolver, probe_arena, mid);
  bool ok = test_interpreter->AllocateTensors() == kTfLiteOk;
  test_interpreter.reset();
  if (ok) {
    upper = mid;       // mid works -> shrink upper bound
  } else {
    lower = mid + 1;   // mid too small -> raise lower bound
  }
}

// lower == upper == minimum proven-good size. Re-test to confirm, then round
// up to 16-byte alignment as the docstring promises.
auto final_interpreter = std::make_unique<tflite::MicroInterpreter>(model, *local_resolver, probe_arena, lower);
bool final_ok = final_interpreter->AllocateTensors() == kTfLiteOk;
final_interpreter.reset();
probe_allocation.data.reset();

if (final_ok) {
  size_t aligned_lower = (lower + 15) & ~15;
  ESP_LOGI(TAG, "probe_arena_size: minimum required arena: %zu bytes", aligned_lower);
  return aligned_lower;
}
// upper was proven to work, so fall back to it.
ESP_LOGI(TAG, "probe_arena_size: using proven size: %zu bytes", upper);
return upper;
```

## 4. MEMORY LEAK ANALYSIS

### 4.1 Current Memory Management (RAII - GOOD)

The component correctly uses:
```cpp
std::unique_ptr<uint8_t[], HeapCapsDeleter> data;
```

This ensures automatic cleanup via `heap_caps_free()` on destruction.

### 4.2 Potential Memory Issues

| Issue | Location | Impact |
|-------|----------|--------|
| No arena fragmentation monitoring | `TFLiteMicroHelper::report_memory_status()` | Medium - long-term heap degradation (see 5.1) |

(Removed: v1's "Arena stats mutex not held during write" row -- already fixed, see 2.3.)

## 5. ENHANCEMENTS

### 5.1 Add Heap Fragmentation Monitoring

**Purpose:** Detect heap fragmentation before it causes allocation failures.

**Implementation (in `TFLiteMicroHelper::report_memory_status()`):**
```cpp
void TFLiteMicroHelper::report_memory_status() {
  MemoryManager::report_memory_status(
      this->tensor_arena_size_requested_,
      this->tensor_arena_allocation_.actual_size,
      this->model_handler_.get_arena_used_bytes(),
      this->model_length_
  );

  // NEW: Monitor fragmentation
  size_t total_free = heap_caps_get_total_free(MALLOC_CAP_INTERNAL);
  size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

  ESP_LOGD(TAG, "Heap status - Total free: %zu bytes, Largest block: %zu bytes",
           total_free, max_block);

  if (max_block < this->tensor_arena_size_requested_) {
    ESP_LOGW(TAG, "WARNING: Largest free block (%zu) < required arena size (%zu)",
             max_block, this->tensor_arena_size_requested_);
  }
}
```

### 5.2 Add Model Loading Statistics

**Purpose:** Track model load performance for debugging and optimization.

**Implementation:**
```cpp
struct LoadStats {
  uint32_t load_start_ms = 0;
  uint32_t parse_time_ms = 0;   // CRC verify + arena probe
  uint32_t arena_alloc_time_ms = 0;
  uint32_t total_load_time_ms = 0;
  bool success = false;
};

// Add to TFLiteMicroHelper
LoadStats last_load_stats_{};

// Public accessor
const LoadStats &get_last_load_stats() const { return this->last_load_stats_; }
```

`load_model()` is instrumented: `load_start_ms` at entry, `parse_time_ms`
around CRC/probe, `arena_alloc_time_ms` around `allocate_tensor_arena_()`,
`total_load_time_ms` at every exit path, and `success` set on READY. A summary
log line is emitted after a successful load.

## 6. TESTING REQUIREMENTS

### 6.1 Unit Tests Needed

- [ ] Test `allocate_tensor_arena()` with PSRAM and SRAM
- [ ] Test arena stats thread-safety under concurrent access (regression)
- [ ] Test `probe_arena_size()` returns the TRUE minimum (not just a working size)
  - [ ] Model that needs ~attempt_size (lower would exceed upper) does not over-run the probe buffer
- [ ] Test heap fragmentation detection
- [ ] Test `reset_config()` clears all fields properly

### 6.2 Integration Tests Needed

- [ ] ESP32-S3: Load 512KB model, verify arena allocation from PSRAM
- [ ] ESP32: Load 200KB model, verify fallback to internal SRAM
- [ ] Stress test: rapid load/unload cycles

## 7. FILES TO MODIFY

| File | Change |
|------|--------|
| `tflite_micro_helper.h` | Add `LoadStats` struct + `last_load_stats_` member + `get_last_load_stats()` accessor |
| `tflite_micro_helper.cpp` | Instrument `load_model()` with load stats (5.2); add heap fragmentation monitoring to `report_memory_status()` (5.1) |
| `model_handler.cpp` | Fix `probe_arena_size()` binary-search logic (3.1) |