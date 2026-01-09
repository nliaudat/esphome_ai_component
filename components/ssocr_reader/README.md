# SSOCR Reader Component (Alpha)

> [!WARNING]
> **ALPHA STAGE SOFTWARE**
> This component is currently in **ALPHA**. It is experimental, may be unstable, and the configuration API is subject to change. Use with caution.

The `ssocr_reader` is an ESPHome component designed to read **7-segment displays** (like water meters, energy meters) using a simple, lightweight Optical Character Recognition (OCR) algorithm.

Unlike the `meter_reader_tflite` component which uses Neural Networks (AI), this component uses **traditional computer vision** techniques (thresholding, segmentation, and vertical projection).

## Features
*   **Lightweight**: Uses significantly less RAM than TensorFlow Lite solutions (only ~20KB buffer vs 500KB+).
*   **Fast**: Processing time is typically <50ms.
*   **Integrated**: Uses the same Coordinator ecosystem as other AI components for robust camera handling and flash control.

## Trade-offs
*   **Brittle Accuracy**: Requires **perfect alignment** and reliable lighting. It is not "smart" – shifted, rotated, or blurry digits will fail.
*   **Limited Font Support**: Only supports standard 7-segment layouts.

## Configuration

```yaml
value_validator:
  id: ${id_prefix}_validator
  allow_negative_rates: false
  max_absolute_diff: 50

ssocr_reader:
  id: ssocr_main
  # Camera ID to use (from esp32_camera)
  camera_id: my_camera
  
  # Validator (recommended)
  validator: ${id_prefix}_validator
  
  # Update interval
  update_interval: 60s
  
  # Image Processing Config
  threshold_level: 128      # 0-255: Binarization threshold
  digit_count: 6            # Number of digits to expect
  
  # Crop Window (CRITICAL: Must match digits exactly)
  crop_x: 100
  crop_y: 200
  crop_w: 300
  crop_h: 80
  
  # Sensors
  value:
    name: "Water Meter Value"
    id: water_value
  
  confidence: # Always returns 100% or NaN currently
    name: "SSOCR Confidence"
```

## Troubleshooting
Enable debug logging to visualize the segmentation process:
```yaml
logger:
  level: DEBUG
  logs:
    ssocr_reader: VERBOSE
```
Look for `Found X potential digits` in the logs. If X is not your `digit_count`, adjust `threshold_level` or the `crop` window.
