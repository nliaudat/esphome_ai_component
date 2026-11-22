# Meter Reader TFLite Component

`meter_reader_tflite` is the core component that orchestrates the AI meter reading process. It integrates the camera, TFLite model, and image processing to deliver sensor readings.

Model configuration is handled by the **`tflite_micro_runtime`** component. You declare the model once in a `tflite_micro_runtime` entry and reference it by ID from `meter_reader_tflite`.

> **Note:** `model:` is a required **ID reference**. The old `model: "file.tflite"` string syntax has been removed — see [Migration from `tflite_micro_helper`](#migration-from-tflite_micro_helper) below.

> **Requires ESPHome 2026.11 or later.**

## ⚙️ Configuration

### Recommended Setup (tflite_micro_runtime)

```yaml
# 1. Declare the model in tflite_micro_runtime (MULTI_CONF: a list)
tflite_micro_runtime:
  - id: my_digit_model
    model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
    # Input shape / arena size read from the model (optional .txt report for extra metadata)
    # Optional overrides:
    # tensor_arena_size: "110KB"

# 2. Reference the model by ID in meter_reader_tflite
meter_reader_tflite:
  id: my_meter_reader
  model: my_digit_model            # ← required ID reference to a tflite_micro_runtime entry
  camera_id: my_camera             # ID of esp32_camera component

  # Validator (optional but recommended)
  validator: my_validator

  # Optional Settings
  update_interval: 60s             # How often to process images
  confidence_threshold: 0.85       # Minimum confidence to publish

  # Debugging
  debug: false                     # Enable verbose logging
  debug_image: false               # Use embedded static image for testing

  # Sensors
  value_sensor: my_value_sensor
  confidence_sensor: my_confidence_sensor

  # Integration
  flash_light_controller: my_flash
  crop_zones_global: my_zones
  data_collector: my_collector
```

> `meter_reader_tflite` is a **consumer-driven** runtime user: it fills the input
> tensor itself. Model parameters it needs in code (`input_order`, `normalize`)
> are options on `meter_reader_tflite`; the rest (`tensor_arena_size`, `debug`)
> belong to the runtime entry. Image rotation is configured on
> `esp32_camera_utils` (see below).
>
> The alternative is a **kind entry**: `tflite_image:` (or detection,
> segmentation, pose, tabular, text, audio) is the YAML key, auto-loads
> `tflite_micro_runtime` and supplies the input pipeline and output decoder. A
> `meter_reader_tflite` can reference such an entry by ID, but it would not use
> that pipeline, so the plain runtime entry above is the right choice here.

### Migration from `tflite_micro_helper`

The monolithic `tflite_micro_helper` entry has been replaced by the split TFLite components:

| Before | After |
|--------|-------|
| `tflite_micro_helper:` (list entry) | `tflite_micro_runtime:` (MULTI_CONF list) |
| `meter_reader_tflite: model: "file.tflite"` | `meter_reader_tflite: model: <runtime entry id>` |
| `debug: true` on the model entry | `debug: true` on the `tflite_micro_runtime` entry (auto-loads `tflite_model_debug`) |
| `tensor_arena_size` on the model entry | unchanged: still on the `tflite_micro_runtime` entry |

```yaml
# Before (tflite_micro_helper)
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer.tflite"

# After (tflite_micro_runtime) — `meter_reader_tflite: model: my_digit_model` is unchanged
tflite_micro_runtime:
  - id: my_digit_model
    model: "digit_recognizer.tflite"
```

> `tflite_micro_helper` was removed from this repository. `micro_wake_word` now uses the upstream self-contained component; `meter_reader_tflite` uses `tflite_micro_runtime`.

### Image Rotation

**Image Rotation**:
Rotation is configured on the **`esp32_camera_utils`** component (it is no longer a `meter_reader_tflite` option). Enabling it lets the AI read an image rotated by 90, 180, or 270 degrees, which is useful if your camera is mounted sideways or upside down.

```yaml
esp32_camera_utils:
  id: my_camera_utils
  camera_id: my_camera
  enable_rotation: true
  rotation: 90.0 # clockwise degrees; 0/90/180/270 are hardware-optimized, other values use software rotation
```

> [!IMPORTANT]
> **Important Note on "Link to HA"**:
> *   The `rotation` setting **ONLY affects the AI Meter Reader**. The AI will now "see" the image rotated.
> *   The **Live Camera View** in Home Assistant (Hassio) comes directly from the camera hardware/webserver, which typically **does NOT support 90-degree rotation** (only 180 flip/mirror).
> *   **Result**: Your HA view will remain sideways, but the AI should now correctly read the digits because it is processing the rotated version internally.

### Camera Window (digital zoom)

Windowing is configured on `esp32_camera` with the native `window:` key, not in `meter_reader_tflite`. Keep `camera_resolution` in sync with `esp32_camera.resolution` so the component parses the frames with the right dimensions:

```yaml
esp32_camera:
  id: my_camera
  resolution: 640x480
  window:
    offset_x: 160
    offset_y: 80
    width: 320
    height: 240

meter_reader_tflite:
  camera_resolution: 640x480   # must match esp32_camera.resolution
```

Coordinates are in `resolution` pixels (not full-sensor pixels) and the window must fit inside the resolution. See [`esp32_camera_utils`](../esp32_camera_utils/README.md) for the per-sensor rules.

## ⚙️ Configuration Reference

| Option | Type | Default | Description |
|---|---|---|---|
| `model` | ID | *required* | ID of the `tflite_micro_runtime` entry |
| `camera_id` | ID | *required on ESP32* | ID of the `esp32_camera` |
| `validator` | ID | — | `value_validator` instance |
| `confidence_threshold` | float 0–1 | `0.85` | Minimum confidence to publish |
| `update_interval` | time | `60s` | Polling interval |
| `camera_resolution` | string | `640x480` | Must match `esp32_camera.resolution` |
| `input_order` | `RGB` / `BGR` / `GRAY` | derived | Channel order; defaults to `GRAY` for one input channel, `RGB` for three |
| `normalize` | bool | `false` | Scale float inputs to 0–1 |
| `frame_request_timeout` | ms | `15000` | Camera frame request timeout |
| `flash_light_controller` | ID | — | Flash control around each capture |
| `data_collector` | ID | — | Upload uncertain images |
| `collect_low_confidence` | bool | `true` | Feed low-confidence readings to the collector |
| `collect_min_global_confidence` | float 0–1 | `0.90` | Collector: global confidence floor |
| `collect_min_digit_confidence` | float 0–1 | `0.90` | Collector: per-digit confidence floor |
| `crop_zones_global` | ID | — | `globals` holding the crop-zone JSON |
| `value_sensor` / `confidence_sensor` | ID | — | Publish reading / confidence |
| `inference_logs` / `main_logs` | ID | — | Text sensors receiving logs |
| `generate_preview` | bool | `false` | Serve `/preview` with crop areas drawn |
| `show_crop_areas` | bool | `true` | Draw crop areas on the preview |
| `enable_flash_calibration` | bool | `false` | Enable flash calibration mode |
| `start_flash_calibration_button` | ID | — | Button that starts a flash calibration |
| `unload_button` / `reload_button` | ID | — | Buttons that unload / reload the model |
| `debug` | bool | `false` | Verbose component logging |
| `debug_image` | bool | `false` | Run inference on the bundled static image |
| `debug_image_out_serial` | bool | `false` | Dump the processed image to serial |
| `debug_camera` | bool | `false` | Verbose `esp32_camera_utils` logging |
| `debug_timing` | bool | `false` | Timing breakdowns |
| `debug_memory` | bool | `false` | Memory logging + diagnostic sensors |

> **Removed options:** `tensor_arena_size`, `input_type`, `input_channels`, `input_width`, `input_height`, `output_processing`, `scale_factor`, `invert` and `rotation` no longer exist on `meter_reader_tflite`. Those belong to the runtime kind pipeline (`tflite_image:`) or `esp32_camera_utils`; `input_order` and `normalize` are accepted here because this component does its own preprocessing.


## 📡 Sensor Configuration

To expose the results to Home Assistant:

```yaml
sensor:
  - platform: template
    name: "Meter Reading"
    id: meter_value

  - platform: template
    name: "Confidence"
    id: meter_confidence
    unit_of_measurement: "%"
```

## 🔍 Debugging

Set `debug: true` to see detailed breakdown of:
- Pre-processing time
- Inference time
- Post-processing logic
- Zone-by-zone analysis
