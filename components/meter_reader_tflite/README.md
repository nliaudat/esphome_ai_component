# Meter Reader TFLite Component

`meter_reader_tflite` is the core component that orchestrates the AI meter reading process. It integrates the camera, TFLite model, and image processing to deliver sensor readings.

## ⚙️ Configuration

```yaml
meter_reader_tflite:
  id: my_meter_reader
  
  # Required
  model: "digit_recognizer.tflite"  # Model file in config directory
  camera_id: my_camera             # ID of esp32_camera component
  
  # Optional Settings
  update_interval: 60s             # How often to process images
  confidence_threshold: 0.7        # Minimum confidence (0.0 - 1.0)
  tensor_arena_size: 512KB         # Memory for TFLite (default: 512KB)
  
  # Debugging
  debug: false                     # Enable verbose logging
  debug_image: false               # Use embedded static image for testing
  debug_image_out_serial: false    # Dump processed image to serial
  
  # Advanced Validation
  allow_negative_rates: false      # Prevent reading from decreasing
  max_absolute_diff: 100           # Max allowed jump between readings
  
  # Sensors
  value_sensor: my_value_sensor             # Sensor to publish reading
  confidence_sensor: my_confidence_sensor   # Sensor to publish confidence
  inference_logs: my_log_sensor             # Text sensor for inference details
  
  # Integration
  flash_light_controller: my_flash # Link to flash controller
  crop_zones_global: my_zones      # Link to global crop zone variable
```

### Image Rotation

To rotate the camera image, configure the `esp32_camera_utils` component:

```yaml
esp32_camera_utils:
  rotation: "90" # Options: "0", "90", "180", "270"
```

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
