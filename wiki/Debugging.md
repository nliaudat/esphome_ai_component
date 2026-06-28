# Debugging Guide

This document describes all debug flags, their effects, activation methods, and common troubleshooting recipes.

---

## Quick Reference Table

| YAML Flag | Build Define | Effect | Section |
|-----------|-------------|--------|---------|
| `debug: true` | `DEBUG_METER_READER_TFLITE` | Verbose inference logs, model architecture dump | [1.1](#11-debug) |
| `debug_image: true` | `DEBUG_METER_READER_TFLITE` | Static test image mode (loads `debug.jpg`) | [1.2](#12-debug_image) |
| `debug_image_out_serial: true` | `DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL` | Zone bounds + first pixels to serial | [1.3](#13-debug_image_out_serial) |
| `debug_memory: true` | `DEBUG_METER_READER_MEMORY` | Heap/PSRAM/arena/pool efficiency sensors | [1.4](#14-debug_memory) |
| `debug_timing: true` | `DEBUG_METER_READER_TIMING` | Acquisition/processing/inference timing | [1.5](#15-debug_timing) |
| `generate_preview: true` | Runtime flag | Web preview at `http://<ip>/preview` | [2.1](#21-web-preview) |
| `show_crop_areas: true` | Runtime flag | Draw boxes on preview image | [2.2](#22-show_crop_areas) |
| *(build flag)* | `DEBUG_ESP32_CAMERA_UTILS` | Full pixel dump, ASCII zone previews | [1.6](#16-debug_esp32_camera_utils) |

---

## 1. Debug Flags (Per-Component)

### 1.1 `debug: true`

**Location:** `meter_reader_tflite:` section

**Build define:** `DEBUG_METER_READER_TFLITE`

**Effects:**
- `ModelHandler`: Logs operator codes found in model, input tensor dimensions, model architecture (subgraphs/tensors/operators count)
- `TFLiteCoordinator`: Logs each inference result (value + confidence per zone)
- `ImageProcessor`: Zone bounds, first pixels, channel order on each processed zone
- `DebugCoordinator`: Prints full debug info on startup (model size, arena size, memory footprint)

**Example YAML:**
```yaml
meter_reader_tflite:
  debug: true
```

**Sample output:**
```
[D][ModelHandler:023]: Registering op: CONV_2D
[D][tflite_coordinator:147]: Running inference on 8 zones
[D][ModelHandler:433]: Direct class - Value: 5.0, Confidence: 0.894531
[D][tflite_coordinator:154]: Inference result: val=5.00, conf=0.89
```

---

### 1.2 `debug_image: true`

**Location:** `meter_reader_tflite:` section

**Build define:** `DEBUG_METER_READER_TFLITE` (same as `debug: true`)

**Effect:** Loads a hardcoded `debug.jpg` file from the `components/meter_reader_tflite/` directory instead of using the live camera. Injects this as the test image for inference.

**Important:** The name is misleading — this does **not** generate a debug image of what the camera sees. It uses a **static embedded JPEG** for reproducible testing.

**Usage:**
```yaml
meter_reader_tflite:
  debug_image: true
```

To view the camera's actual image, use the Web Preview (section 2.1).

---

### 1.3 `debug_image_out_serial: true`

**Location:** `meter_reader_tflite:` section

**Build define:** `DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL`

**Effect:** Enables zone bound logging and first pixel analysis in `ImageProcessor::split_image_in_zone()`. Logs:
- Crop zone coordinates and target dimensions
- First 5 pixel values from each processed zone (float32)
- Channel order detection (BGR vs RGB)

**Usage:**
```yaml
meter_reader_tflite:
  debug_image_out_serial: true
```

**Sample output:**
```
[I][ImageProcessor:052]: ZONE: [67,9,98,77] -> 31x68 -> 20x32
[I][ImageProcessor:058]: FIRST_PIXELS:
[I][ImageProcessor:063]:   Pixel 0: Ch0: 0.5 Ch1: 0.5 Ch2: 0.5
[I][ImageProcessor:073]: CHANNEL_ORDER_TEST:
[I][ImageProcessor:074]:   First pixel: 0.5, 0.5, 0.5
```

---

### 1.4 `debug_memory: true`

**Location:** `meter_reader_tflite:` section

**Build define:** `DEBUG_METER_READER_MEMORY`

**Effect:** Creates diagnostic sensors for memory analysis:
- `tensor_arena_size`: Allocated arena size in bytes
- `tensor_arena_used`: Peak bytes used during inference
- `process_free_heap`: Free internal heap during processing
- `process_free_psram`: Free PSRAM during processing
- `pool_job_efficiency`: InferenceJob pool hit rate (%)
- `pool_result_efficiency`: InferenceResult pool hit rate (%)
- `arena_efficiency`: Arena usage efficiency (%)
- `heap_fragmentation`: Free heap fragmentation (0% = perfect)

Also logs memory status every 60 seconds (arena efficiency, pool efficiency, fragmentation).

**Usage:**
```yaml
meter_reader_tflite:
  debug_memory: true
```

---

### 1.5 `debug_timing: true`

**Location:** `meter_reader_tflite:` section

**Build define:** `DEBUG_METER_READER_TIMING`

**Effect:** Logs detailed timing of image processing stages:
- Image acquisition time (request → camera callback)
- Preprocessing time (JPEG decode → crop → scale)
- Total inference time

**Usage:**
```yaml
meter_reader_tflite:
  debug_timing: true
```

**Sample output:**
```
[I][meter_reader_tflite:737]: Image Acquisition took 125 ms
[I][meter_reader_tflite:738]: Preprocessing (Crop/Scale) took 245 ms
[I][meter_reader_tflite:878]: Total Processing took 412 ms
```

---

### 1.6 `DEBUG_ESP32_CAMERA_UTILS` (Build Flag)

**Location:** Not settable from YAML — must be added via `board_build.extra_flags`

**Effect:** Enables the most verbose debug mode in `ImageProcessor`:
- Full pixel statistics (min, max, mean) per zone
- ASCII art preview of zone pixels (`#` and `.` characters)
- Float32 image analysis
- Internal processing stage logging

**Activation:**
```yaml
board_build.extra_flags:
  - -DDEBUG_ESP32_CAMERA_UTILS
```

**Caution:** Very verbose. Can saturate UART output at 115200 baud.

---

## 2. Visual Debugging (Web Preview)

### 2.1 Web Preview

**Prerequisites** — all four must be enabled simultaneously:

```yaml
# 1. In esp32_camera_utils section:
esp32_camera_utils:
  enable_drawing: true  # Enable drawing subsystem

# 2. In meter_reader_tflite section:
meter_reader_tflite:
  generate_preview: true  # Enable preview capture
  show_crop_areas: true   # Draw zone boxes (see 2.2)

# 3. In root level (uncomment web_server section):
web_server:
  port: 80

# 4. Uncomment the include near the bottom:
packages:
  - !include camera_webserver.yaml  # Add web preview handler
```

**Access:** `http://<board-ip>/preview` — serves the rotated camera view with optional crop zone boxes.

---

### 2.2 `show_crop_areas: true`

**Status:** ❌ **Currently non-functional.** The drawing code path in `meter_reader_tflite.cpp:703-730` is guarded by `USE_CAMERA_DRAWING` but the actual `draw_rectangle()` calls are not implemented. The loop iterates over zones but only comments describe the intended behavior.

**Workaround:** Use `tools/draw_regions.py` to visually calibrate crop zones from a captured image.

---

## 3. Diagnostic Tools

### 3.1 `tools/draw_regions.py`

Generates a visual overlay of crop zones on a captured camera image. Helps verify zone positions without reflashing the board.

```bash
python tools/draw_regions.py --image snapshot.jpg --zones "[...]"
```

### 3.2 `tools/check_tflite_model.py`

Analyzes a `.tflite` model file for:
- Tensor sizes and types
- Memory usage analysis
- Arena size estimate
- TFLite Micro compatibility
- Delegate operations check
- Peak memory analysis

```bash
python tools/check_tflite_model.py models/digit_recognizer_v3_quantized_integer_quant_uint8.tflite --verbose
```

### 3.3 `tools/regenerate_txt_reports.py`

Regenerates `.txt` companion files for all `.tflite` models. The `.txt` files contain the arena size recommendation and are parsed by `__init__.py` to auto-configure the component.

```bash
python tools/regenerate_txt_reports.py --force
```

### 3.4 `tools/analyze_peak_memory.py`

Performs tensor lifetime analysis and provides a refined arena size estimate.

---

## 4. Troubleshooting Recipes

### 4.1 "All zones output class 0"

**Symptom:** All digit predictions show value `0` with near-perfect confidence (`0.988`).

**Check:**
1. Is rotation correct? `Synced rotation from esp32_camera_utils: 0.0` vs expected `270.0`
2. Are crop zones within the rotated image bounds? Look for `Zone bounds error vs Master` in logs
3. Enable `debug_image_out_serial: true` to verify first pixel values are non-zero

**Fix:** Adjust `rotation` in config or recalibrate crop zones.

### 4.2 "Failed to allocate tensors"

**Symptom:** Model fails to load with `Failed to allocate tensors`.
See [Memory Management](setup.md#memory-management) for details.

**Check:**
1. `Tensor arena: <size> bytes` — verify the arena size matches the `.txt` recommendation
2. For ESP32-S3: the arena is automatically bumped by 1.5× due to 64B cache-line alignment
3. Enable `debug_memory: true` to monitor actual arena usage after successful load

**Fix:** Re-run `python tools/regenerate_txt_reports.py --force` to update arena estimates, then reflash.

### 4.3 "Low accuracy on S3 vs ESP32"

**Symptom:** Same model, same water meter, but Prokyber S3 gives different predictions than AiThinker ESP32.

**Check:**
1. Are crop zones calibrated for the specific board mount position?
2. Is `rotation` correctly set for the board's camera orientation?
3. Enable `debug_image_out_serial: true` and compare pixel values on both boards for the same meter state

**Fix:** Adjust crop zones per board — zone coordinates are mount-specific, not transferable between boards with different physical camera positions.

### 4.4 "Web preview not working"

**Symptom:** `http://<ip>/preview` returns 404 or empty response.

**Check:**
1. All four prerequisites from [section 2.1](#21-web-preview) are enabled
2. `enable_drawing: true` is under `esp32_camera_utils:` not `meter_reader_tflite:`
3. `camera_webserver.yaml` include is **after** the camera configs, not before
4. `logger:` has at least INFO level to see preview handler messages

**Fix:** Compare your config against the template in `config_prokyber_s3.yaml` (lines 94-116 for `esp32_camera_utils`, 202-205 for `meter_reader_tflite`, 285-286 for `web_server`, 312 for the include).

### 4.5 "Model loads but inference returns garbage"

**Symptom:** Model loads, inference runs, but predictions don't match digits.

**Check:**
1. Zone bounds match the rotated image dimensions
2. Input order matches model training (RGB vs BGR)
3. `normalize` setting matches model expectations (0=raw uint8, 1=float32 [0,1])

**Fix:** The default is uint8 with no normalization — this is correct for TFLite Micro quantized models. For float32 models, set `normalize: true` and verify `input_type: float32`.

---

## 5. Log Level Configuration

The `logger.yaml` controls which log levels appear on the serial monitor:

```yaml
logger:
  level: DEBUG  # Show all debug messages
  # level: INFO  # Default — shows warnings and above
  # level: WARN  # Only warnings and errors
  # level: ERROR # Only errors
```

At `DEBUG` level, all the debug flags described above produce visible output. At `INFO`, only the startup info messages appear.

---

## 6. YAML Configuration Cheat Sheet

```yaml
meter_reader_tflite:
  # Enable all debug modes at once (for initial board bringup):
  debug: true
  debug_memory: true
  debug_timing: true
  debug_image_out_serial: true
  generate_preview: true       # Requires web_server + camera_webserver.yaml
  show_crop_areas: true        # Non-functional (drawing not implemented)

esp32_camera_utils:
  enable_drawing: true          # Required for preview

web_server:
  port: 80                      # Required for preview

packages:
  - !include camera_webserver.yaml  # Required for preview
