# Meter Reader TFLite Component

`meter_reader_tflite` is the core component that orchestrates the AI meter reading process. It integrates the camera, TFLite model, and image processing to deliver sensor readings.

## ⚙️ Configuration

```yaml
value_validator:
  id: ${id_prefix}_validator
  allow_negative_rates: false
  max_absolute_diff: 300
  strict_confidence_check: true
  per_digit_confidence_threshold: 0.95

meter_reader_tflite:
  id: my_meter_reader
  
  # Required
  model: "digit_recognizer.tflite"  # Model file in config directory
  camera_id: my_camera             # ID of esp32_camera component
  
  # Validator (optional but recommended)
  validator: ${id_prefix}_validator
  
  # Optional Settings
  update_interval: 60s             # How often to process images
  # confidence_threshold: 0.85        # Used if validator is NOT set (0.0 - 1.0)
  # tensor_arena_size: 512KB         # Memory for TFLite (default: 512KB)
  
  # Debugging
  debug: false                     # Enable verbose logging
  debug_image: false               # Use embedded static image for testing
  debug_image_out_serial: false    # Dump processed image to serial (very slow!)
  rotation: "0"                    # Image rotation in degrees ("0", "90", "180", "270").
  
  # Sensors
  value_sensor: my_value_sensor             # Sensor to publish reading
  confidence_sensor: my_confidence_sensor   # Sensor to publish confidence
  inference_logs: my_log_sensor             # Text sensor for inference details
  
  # Integration
  flash_light_controller: my_flash # Link to flash controller
  crop_zones_global: my_zones      # Link to global crop zone variable
```

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
