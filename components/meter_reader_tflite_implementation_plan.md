# Meter Reader TFLite - Implementation Plan

**Date:** 2026-08-14 (v2 -- reviewed against current source)

**Component Status:** Active (v2.0+)

## 1. EXECUTIVE SUMMARY

`meter_reader_tflite` is the primary AI component for digit recognition using TensorFlow Lite Micro.
It integrates camera capture, TFLite inference, and value validation.

**Review note (2026-08-14):** v1 of this plan described an older code state. The "CRITICAL MEMORY
ISSUES" claims (manual `new`/`delete` leaks, `delay()` in `loop()`) are FALSE in the current source --
the component already uses RAII job/result pointers with custom deleters and a fully non-blocking
worker-task design. The enhancement items (§5.1 camera recovery, §5.2 inference statistics,
§5.3 grayscale pipeline) are largely implemented. This revision removes the false claims and keeps
the two genuinely additive pieces: consecutive-timeout auto-recovery and consolidated inference
statistics.

## 2. CURRENT STATE ANALYSIS

### 2.1 Strengths

- Well-integrated with `tflite_micro_helper`
- Camera coordination and flashlight control
- Value validation coordinator for reading verification
- Pool-based inference job management with RAII smart-pointer wrappers
- Non-blocking architecture: inference worker task + `millis()`-based frame timeouts (no `delay()`)

### 2.2 Previously-Reported Items -- RESOLVED / FALSE

| v1 Item | Status | Evidence |
|---------|--------|----------|
| "Manual `new` in camera coordinator" (lines 124-125) | **FALSE** | No `new` at those lines; the only `new`s are pool-exhaustion fallbacks immediately RAII-wrapped |
| "Manual `delete` (should use RAII)" (lines 143, 184) | **FALSE** | `delete` only exists inside the custom deleters `free_inference_job()` / `free_inference_result()`, gated by `!from_pool` |
| "Manual `new`" (lines 170, 250, 711) | **FALSE** | `InferenceJobPtr(new InferenceJob(), free_inference_job)` / `InferenceResultPtr(...)` -- RAII with `unique_ptr` + custom deleters |
| "`delay()` calls in loop()" (lines 843-897) | **FALSE** | Zero `delay(` matches in any `.cpp`/`.h` in the component; timeout handling is `millis()`-based (`frame_request_timeout_ms_`, default 15s) |
| "Camera recovery needed (§5.1)" | **PARTIALLY IMPLEMENTED** | `CameraCoordinator::basic_recovery()` exists (`camera_coordinator.h/.cpp`); timeout path in `meter_reader_tflite.cpp`. Missing: consecutive-timeout counter + escalation |
| "Inference statistics needed (§5.2)" | **PARTIALLY IMPLEMENTED** | `ProcessingStats` exists in `image_processor.h`; timing sensors exist. Missing: a consolidated `InferenceStats` struct + report helper |
| "Grayscale pipeline needed (§5.3)" | **ALREADY IMPLEMENTED** | `spec.input_channels == 1` selects `pixel_format = "GRAYSCALE"`; camera coordinator handles `model_channels == 1` luminance |

## 3. CURRENT STATE ANALYSIS (post-v1 de-risking)

### 3.1 Memory Management -- ALREADY RAII (no action needed)

```cpp
using InferenceJobPtr   = std::unique_ptr<InferenceJob,   decltype(&free_inference_job)>;
using InferenceResultPtr = std::unique_ptr<InferenceResult, decltype(&free_inference_result)>;
```

Pool-exhaustion fallbacks (`new InferenceJob()` / `new InferenceResult()`) are immediately wrapped
in the smart pointers; the custom deleters return pool slots or `delete` heap objects. This is the
correct RAII pattern -- v1's proposed `unique_ptr` conversion is already in place.

### 3.2 Blocking Calls -- ALREADY NON-BLOCKING (no action needed)

No `delay()` exists. Frame timeout detection uses `millis() - last_request_time_ >
frame_request_timeout_ms_`. Inference runs on a dedicated FreeRTOS task with poisoned-pill
shutdown that lets RAII destructors run (`free_inference_job` on leaked jobs at teardown).

## 4. ENHANCEMENTS (still open)

### 4.1 Consecutive-Timeout Camera Auto-Recovery (P2)

**Purpose:** Escalate beyond `basic_recovery()` when the camera repeatedly times out.

```cpp
// In meter_reader_tflite.h (or camera_coordinator.h)
struct CameraState {
  uint32_t consecutive_timeouts = 0;
  uint32_t last_timeout_ms = 0;
};

CameraState camera_state_{};
static constexpr uint32_t MAX_CONSECUTIVE_TIMEOUTS = 3;
static constexpr uint32_t RECOVERY_COOLDOWN_MS = 15000;

// On each frame timeout in meter_reader_tflite.cpp:
if (camera_state_.last_timeout_ms == 0 ||
    millis() - camera_state_.last_timeout_ms > RECOVERY_COOLDOWN_MS) {
  camera_state_.consecutive_timeouts = 1;
} else {
  camera_state_.consecutive_timeouts++;
}
camera_state_.last_timeout_ms = millis();

if (camera_state_.consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
  ESP_LOGW(TAG, "Camera unresponsive -- escalating recovery");
  this->camera_coord_.basic_recovery();
  camera_state_.consecutive_timeouts = 0;  // reset after recovery attempt
}
```

### 4.2 Consolidated Inference Statistics (P2)

**Purpose:** Single struct for success/failure + timing, with a report helper for the log.

```cpp
struct InferenceStats {
  uint32_t total_inferences = 0;
  uint32_t successful_inferences = 0;
  uint32_t failed_inferences = 0;
  uint32_t total_capture_time_ms = 0;
  uint32_t total_inference_time_ms = 0;
};

InferenceStats inference_stats_{};

void record_inference_result(bool success, uint32_t capture_time_ms, uint32_t inference_time_ms) {
  inference_stats_.total_inferences++;
  success ? inference_stats_.successful_inferences++ : inference_stats_.failed_inferences++;
  inference_stats_.total_capture_time_ms += capture_time_ms;
  inference_stats_.total_inference_time_ms += inference_time_ms;
}

void report_inference_statistics() {
  ESP_LOGI(TAG, "Inference stats - Total: %u, Success: %u, Failed: %u",
           inference_stats_.total_inferences, inference_stats_.successful_inferences,
           inference_stats_.failed_inferences);
  if (inference_stats_.total_inferences > 0) {
    ESP_LOGI(TAG, "Success rate: %.1f%%",
             100.0f * inference_stats_.successful_inferences / inference_stats_.total_inferences);
  }
}
```

## 5. TESTING REQUIREMENTS

### 5.1 Unit Tests

- [ ] Test pool-exhaustion fallback: heap `InferenceJob` paths are freed correctly
- [ ] Test consecutive-timeout counter resets after recovery or cooldown
- [ ] Test inference statistics accumulation

### 5.2 Integration Tests

- [ ] Full pipeline test with real meter images
- [ ] Simulate repeated camera timeouts → verify `basic_recovery()` is escalated
- [ ] Memory leak test with AddressSanitizer/ESP-MD5

## 6. PRIORITY MATRIX

| Priority | Item | Reason |
|----------|------|--------|
| **P2** | Consecutive-timeout camera auto-recovery (§4.1) | Escalates diagnostics beyond basic recovery |
| **P2** | Consolidated inference statistics (§4.2) | Debugging and optimization aid |

(Removed: v1's P0/P1 "manual new/delete RAII" and "remove delay()" -- false/done, see 2.2.)

## 7. FILES TO MODIFY

| File | Changes |
|------|---------|
| `camera_coordinator.h` | (optional) add `CameraState` if owned there |
| `meter_reader_tflite.h` | Add `InferenceStats` struct + `record/report` methods; add `CameraState` counter members if not in coordinator |
| `meter_reader_tflite.cpp` | Call `record_inference_result()` at job completion; escalate recovery on consecutive timeouts |