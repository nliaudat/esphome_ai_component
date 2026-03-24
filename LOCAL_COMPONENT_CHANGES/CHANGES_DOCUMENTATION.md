# Local Component Changes Documentation

## Overview

This document tracks all modifications made to the local components compared to the upstream `nliaudat/esphome_ai_component` repository.

**Upstream URL:** https://github.com/nliaudat/esphome_ai_component  
**Local Path:** `components/esphome_ai_component/components/`

## Summary of Changes

### 1. Arduino Macro Conflict Fix
**Files:** `esp32_camera_utils/rotator.h`

**Change:**
- Renamed `PI` → `ROTATOR_PI`
- Renamed `DEG_TO_RAD` → `ROTATOR_DEG_TO_RAD`

**Why:**
Arduino.h already defines `PI` and `DEG_TO_RAD` as macros. When rotator.h included Arduino.h indirectly, the macro expansion broke the C++ syntax for `static constexpr float PI = ...`.

**Error seen:**
```
error: expected unqualified-id before numeric constant
#define PI 3.1415926535897932384626433832795
```

---

### 2. Flash Synchronization Frame Discarding
**Files:** `meter_reader_tflite/meter_reader_tflite.h`, `meter_reader_tflite/meter_reader_tflite.cpp`

**Changes:**
- Added `frames_to_discard_` member variable
- Added `capture_state_` state machine enum
- Added `handle_capture_state()` method
- Modified `update()` to use state machine
- Added frame buffering logic in `on_camera_image()`

**Why:**
The original code had timing issues where the flash would turn on, but the camera would capture buffered "dark" frames that were taken before the flash was ready.

**Solution implemented:**
- Turn on flash and wait 2000ms (pre_time)
- Discard 2 buffered frames to ensure fresh flash-lit frames
- Capture new frame
- Keep flash on for 4000ms (post_time)
- Process image

This ensures the captured image actually has the flash illumination.

---

### 3. Camera Initialization Improvements
**Files:** `meter_reader_tflite/meter_reader_tflite.cpp`

**Changes:**
- Added I2C pull-up mitigation
- Added hardware reset sequence for camera
- Added staged delay initialization

**Why:**
The ESP32-CAM was frequently failing to initialize with `ESP_ERR_NOT_SUPPORTED` errors. The camera wasn't responding to I2C commands reliably.

**Mitigations added:**
- Hardware reset via GPIO32
- I2C pull-up configuration
- Delayed initialization sequence

---

### 4. Memory Management "Fixes" (Questionable)
**Files:** `meter_reader_tflite/meter_reader_tflite.cpp`

**Changes made and reverted:**
- Added `job->frame.reset()` after crop extraction (line 903)
- Added `this->last_preview_image_.reset()` before storing new preview (line 1170)

**Why attempted:**
Believed there was memory fragmentation causing allocation failures after 3-4 readings.

**Status:** REVERTED - These changes were speculative and didn't solve the actual problem.

---

## Why These Changes Were Made

### Timeline Context

1. **Initial Setup (March 8):** Basic components created/added
2. **Flash Timing Issues:** Discovered flash wasn't syncing properly with captures
3. **Camera Init Failures:** Camera frequently failed to initialize
4. **Memory Investigation:** Tried various memory "fixes" for fragmentation

### Current Status

- **Flash sync changes:** Working, necessary for proper operation
- **Camera init changes:** Working, reduces initialization failures
- **Macro conflict fix:** Required for compilation with Arduino framework
- **Memory changes:** Reverted, not effective

## Recommendation for Upstream Migration

### Critical Changes to Preserve:
1. **Arduino macro fix** (rotator.h) - Required for compilation
2. **Flash frame discarding** - Critical for proper flash operation

### Changes to Drop:
1. Memory management "fixes" - Already reverted

### Optional Changes:
1. Camera init mitigations - Helpful but may not be necessary with proper hardware

## Testing Plan

1. Apply Arduino macro fix to upstream
2. Test if flash sync works without frame discarding logic
3. If flash doesn't sync properly, port frame discarding logic
4. Evaluate if camera init changes are needed

## Files Modified

```
components/esphome_ai_component/components/
├── esp32_camera_utils/
│   └── rotator.h                    (Arduino macro fix)
├── meter_reader_tflite/
│   ├── meter_reader_tflite.h        (state machine, frame discarding)
│   ├── meter_reader_tflite.cpp      (flash sync, camera init)
│   ├── flashlight_coordinator.h     (timing coordination)
│   └── flashlight_coordinator.cpp   (timing logic)
└── tflite_micro_helper/
    └── model_handler.cpp            (minor changes)
```

## Diff Location

Full diffs are saved in: `LOCAL_COMPONENT_CHANGES/component_diffs.txt`
