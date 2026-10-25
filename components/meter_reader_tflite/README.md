# Meter Reader TFLite Component

`meter_reader_tflite` is the core component that orchestrates the AI meter reading process. It integrates the camera, TFLite model, and image processing to deliver sensor readings.

Model configuration is handled by the **`tflite_micro_runtime`** component. You declare the model once in a `tflite_micro_runtime` entry and reference it by ID from `meter_reader_tflite`.

> **Note:** `model:` is a required **ID reference**. The old `model: "file.tflite"` string syntax has been removed — see [Migration from `tflite_micro_helper`](#-migration-from-tflite_micro_helper) below.

## ⚙️ Configuration

### Recommended Setup (tflite_micro_runtime)

```yaml
# 1. Declare the model in tflite_micro_runtime
tflite_micro_runtime:
  id: my_digit_model
  model_type: image
  model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
  # Input shape / arena size auto-detected from the .txt report
  # Optional overrides:
  # tensor_arena_size: "110KB"
  # input_width: 32
  # input_height: 20

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

> Model parameters (input type/shape/channels, `tensor_arena_size`, `scale_factor`, `output_processing`, `normalize`, `invert`, and the audio keys) belong to the `tflite_micro_runtime` entry, not to `meter_reader_tflite`. Image rotation is configured on `esp32_camera_utils` (see below).

### Migration from `tflite_micro_helper`

The monolithic `tflite_micro_helper` entry has been replaced by the split TFLite components:

| Before | After |
|--------|-------|
| `tflite_micro_helper:` (list entry) | `tflite_micro_runtime:` (entry, `id:` required) |
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
  id: my_digit_model
  model_type: image
  model: "digit_recognizer.tflite"
```

> `tflite_micro_helper` still exists for `micro_wake_word` (audio); `meter_reader_tflite` no longer depends on it.

### Image Rotation

**Image Rotation**:
Rotation is configured on the **`esp32_camera_utils`** component (it is no longer a `meter_reader_tflite` option). Enabling it lets the AI read an image rotated by 90, 180, or 270 degrees, which is useful if your camera is mounted sideways or upside down.

```yaml
esp32_camera_utils:
  id: my_camera_utils
  camera_id: my_camera
  enable_rotation: true
  rotation: "90" # Options: "0", "90", "180", "270"
```

> [!IMPORTANT]
> **Important Note on "Link to HA"**:
> *   The `rotation` setting **ONLY affects the AI Meter Reader**. The AI will now "see" the image rotated.
> *   The **Live Camera View** in Home Assistant (Hassio) comes directly from the camera hardware/webserver, which typically **does NOT support 90-degree rotation** (only 180 flip/mirror).
> *   **Result**: Your HA view will remain sideways, but the AI should now correctly read the digits because it is processing the rotated version internally.


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
