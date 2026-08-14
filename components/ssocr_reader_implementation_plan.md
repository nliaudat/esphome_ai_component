# SSOCR Reader - Implementation Plan

**Date:** 2026-08-14 (v2 -- reviewed against current source)

**Component Status:** Alpha (Experimental)

## 1. EXECUTIVE SUMMARY

`ssocr_reader` implements seven-segment OCR without AI, using traditional image processing.
It's an alpha component for experimental features.

**Review note (2026-08-14):** v1 of this plan overstated issues that are either documented
exceptions, already implemented, or false. The `delay()` calls are intentional, documented
exceptions (service context, not `loop()`). Non-blocking camera acquisition, confidence sensor,
and flashlight synchronization already exist. This revision removes the false "P0" claims and
keeps the three genuinely open items: real per-digit confidence scoring (the sensor currently
publishes a constant 100), a functional `basic_recovery()` (currently a no-op stub), and
multi-algorithm support (a real feature that does not exist).

## 2. CURRENT STATE ANALYSIS

### 2.1 Strengths

- No AI model dependency (lighter footprint)
- Seven-segment display specific optimization
- Value validation integration
- Non-blocking camera callback architecture (`CameraListener::on_camera_image`)
- Atomic frame state machine (IDLE/REQUESTED/AVAILABLE/PROCESSING/TIMEOUT)

### 2.2 Previously-Reported Items -- RESOLVED / FALSE

| v1 Item | Status | Evidence |
|---------|--------|----------|
| "Multiple `delay()` calls" (lines 7-16, 18, 61, 84, 113) -- P0 ESP-IDF violation | **DOCUMENTED EXCEPTION** | `camera_coordinator.cpp:14-18` explicitly states delay() is intentional; callers (`set_window`/`reset_window`) run in service context and expect immediate synchronous results, not `loop()`. Only 3 calls exist (not 5): `delay(100)`/`delay(200)` stabilization + `delay(100)` inside a bounded `feed_wdt()` poll loop |
| "Add non-blocking camera acquisition (§3.2)" | **ALREADY IMPLEMENTED** | `SSOCRReader` is a `camera::CameraListener`; frames arrive via `on_camera_image()` callback. No blocking capture exists |
| "Audit all `new`/`delete` in camera_coordinator.cpp (§4.1)" | **FALSE** | Zero `new`/`delete`; `ImageProcessor` is created via `std::make_unique` (`camera_coordinator.cpp:135`) |
| "Add confidence scoring (§5.2)" | **PARTIALLY IMPLEMENTED** | `confidence_sensor_` exists and is wired (`ssocr_reader.h:30,67`), but publishes a hardcoded `100.0f` (`ssocr_reader.cpp:204`) -- see 3.1 |
| "Add flashlight synchronization (§5.3)" | **ALREADY IMPLEMENTED** | `FlashlightCoordinator` with `set_timing(pre, post)`, `enable/disable_flash`, `force_inference`, `update_scheduling` (`flashlight_coordinator.h`) |

## 3. BUG FIXES

### 3.1 Real Per-Digit Confidence (replaces hardcoded 100.0f)

**Problem:** `confidence_sensor_->publish_state(100.0f)` is a meaningless constant.

**Solution:** compute a real confidence from the per-digit recognition margin. For each of the
7 segments the on-ratio (fraction of lit pixels in the 3x3 probe window) is compared to the
0.5 decision threshold. The confidence of a digit is the average normalized margin:

```cpp
// inside recognize_digit(), per segment:
//   margin_i = ratio > 0.5 ? (ratio - 0.5) : (0.5 - ratio);   // 0..0.5
//   confidence = average(margin_i / 0.5);                     // 0..1
```

A confident read (all segments strongly on/off) approaches 1.0; a marginal read (segments
near the decision boundary) drops toward 0. The per-digit confidences are averaged across the
recognized digits and published to `confidence_sensor_`.

**API:** `int recognize_digit(..., float *confidence_out = nullptr)` -- out-param keeps the
existing return convention (digit index or -1 for unknown).

### 3.2 Make `basic_recovery()` Functional

**Problem:** `CameraCoordinator::basic_recovery()` only logs
(`camera_coordinator.cpp:96-100`).

**Solution:** mirror the functional recovery already used in `reset_window()` -- attempt a
hard camera reset, fall back to soft reset, then restore full-frame dimensions:

```cpp
void CameraCoordinator::basic_recovery() {
  ESP_LOGW(TAG, "Executing basic camera recovery (state reset)");
  if (this->window_control_.hard_reset_camera(this->camera_)) {
    ESP_LOGI(TAG, "Camera hard-reset OK");
    this->window_control_.reset_to_full_frame_with_dimensions(
        this->camera_, this->orig_width_, this->orig_height_, this->current_width_, this->current_height_);
    this->current_format_ = this->orig_format_;
  } else {
    ESP_LOGW(TAG, "Hard reset failed, trying soft reset");
    this->window_control_.soft_reset_camera(this->camera_);
  }
}
```

## 4. MEMORY LEAK ANALYSIS

No action needed -- `camera_coordinator.cpp` uses no raw `new`/`delete`; the
`ImageProcessor` is RAII-owned via `std::unique_ptr` (`camera_coordinator.cpp:135`).

## 5. ENHANCEMENTS

### 5.1 Multiple OCR Algorithms (not implemented -- real feature)

`OcrAlgorithm` / `OcrConfig` do not exist; the reader uses a single heuristic
`recognize_digit()`. This is a legitimate P2 feature:

```cpp
enum class OcrAlgorithm {
  SEGMENT_PROJECTION,   // current implementation
  TEMPLATE_MATCHING,
  CONTOUR_ANALYSIS,
};

struct OcrConfig {
  OcrAlgorithm algorithm = OcrAlgorithm::SEGMENT_PROJECTION;
  float confidence_threshold = 0.7f;
  bool use_validation = true;
};
```

(Template/contour algorithms require additional infrastructure -- out of scope for this
revision.)

## 6. TESTING REQUIREMENTS

### 6.1 Unit Tests

- [ ] Test confidence scoring: strong vs marginal segment reads
- [ ] Test `basic_recovery()` hard-reset + soft-reset fallback paths

### 6.2 Integration Tests

- [ ] Seven-segment display testing with various fonts/sizes
- [ ] Verify confidence sensor reports < 100% for marginal reads
- [ ] Memory leak test under continuous operation

## 7. PRIORITY MATRIX

| Priority | Item | Reason |
|----------|------|--------|
| **P1** | Real per-digit confidence (§3.1) | Confidence sensor currently publishes meaningless constant |
| **P2** | Functional `basic_recovery()` (§3.2) | Stub only logs; no recovery action |
| **P2** | Multiple algorithm support (§5.1) | Feature enhancement |

(Removed: v1's P0 "remove delay()" and P1 "add RAII" -- false/documented-exception, see 2.2.)

## 8. FILES TO MODIFY

| File | Changes |
|------|---------|
| `ssocr_reader.h` | Add `confidence_out` parameter to `recognize_digit()` |
| `ssocr_reader.cpp` | Compute per-digit confidence; publish average; functional `basic_recovery()` |