# Meter Reader TFLite Component

`meter_reader_tflite` is the core component that orchestrates the AI meter reading process. It integrates the camera, TFLite model, and image processing to deliver sensor readings.

Model configuration is now handled by the **`tflite_micro_helper`** component. You define the model in a `tflite_micro_helper` entry and reference it by ID from `meter_reader_tflite`.

> **Note:** The legacy `model: "file.tflite"` syntax still works but is deprecated and prints a warning. See [Backward Compatibility](#-backward-compatibility) below.

## ⚙️ Configuration

### Recommended Setup (tflite_micro_helper)

```yaml
# 1. Define the model in tflite_micro_helper
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
    # All config auto-detected from .txt file
    # Optional overrides:
    # tensor_arena_size: "110KB"
    # input_width: 32
    # input_height: 20

# 2. Reference the model by ID in meter_reader_tflite
meter_reader_tflite:
  id: my_meter_reader
  model: my_digit_model            # ← ID reference to tflite_micro_helper entry
  camera_id: my_camera             # ID of esp32_camera component

  # Validator (optional but recommended)
  validator: my_validator

  # Optional Settings
  update_interval: 60s             # How often to process images
  confidence_threshold: 0.85       # Minimum confidence to publish

  # Debugging
  debug: false                     # Enable verbose logging
  debug_image: false               # Use embedded static image for testing
  rotation: "0"                    # Image rotation ("0", "90", "180", "270")

  # Sensors
  value_sensor: my_value_sensor
  confidence_sensor: my_confidence_sensor

  # Integration
  flash_light_controller: my_flash
  crop_zones_global: my_zones
  data_collector: my_collector
```

### Legacy Setup (Deprecated)

```yaml
value_validator:
  id: my_validator
  allow_negative_rates: false
  max_absolute_diff: 300

meter_reader_tflite:
  id: my_meter_reader
  model: "digit_recognizer.tflite"  # Direct file path (deprecated)
  camera_id: my_camera
  validator: my_validator
  update_interval: 60s
  debug: false
  rotation: "0"
```

> [!WARNING]
> The `model: "file.tflite"` syntax is **deprecated**. It will print a warning at compile time. Please migrate to the `tflite_micro_helper` approach.

### Image Rotation

**Image Rotation**:
The `rotation` parameter allows you to rotate the camera image by 90, 180, or 270 degrees. This is useful if your camera is mounted sideways or upside down.

To rotate the camera image, configure the `esp32_camera_utils` component:

```yaml
esp32_camera_utils:
  rotation: "90" # Options: "0", "90", "180", "270"
  # Optional: Image Rotation
  # Options: "0", "90", "180", "270"
  rotation: "90"
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
