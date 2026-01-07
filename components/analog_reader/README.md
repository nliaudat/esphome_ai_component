# Analog Reader Component (Alpha)

> [!WARNING]
> **ALPHA STAGE SOFTWARE**
> This component is currently in **ALPHA**. It is experimental, may be unstable, and the configuration API is subject to change. Use with caution.

This component provides a lightweight, non-AI solution for reading analog gauges (dials) using traditional computer vision.

## 🚀 Performance Gains vs AI
Using this component instead of an AI model (`meter_reader_tflite`) offers significant advantages for standard analog dials:

| Metric | AI Model (TFLite) | Analog Reader (CV) | Gain |
| :--- | :--- | :--- | :--- |
| **RAM Usage** | ~150KB (Tensor Arena) | ~16KB (Image Buffer) | **~10x Less** |
| **Flash Size** | 50KB - 100KB (Model) | < 4KB (Code) | **~25x Smaller** |
| **Latency** | 200ms - 800ms | < 20ms | **~40x Faster** |
| **Complexity** | High (Black box) | Low (Deterministic) | **Simpler** |

**Trade-off**: This valid algorithm requires the dial to be **cleanly cropped and centered**. Unlike AI, it "sees" using strict geometry, so it cannot strictly ignore obstructions or heavy glare as well as a trained network.

## 🛠️ Configuration

### Basic Usage
The component allows you to define multiple dials. Each dial detects its needle angle and contributes to a total value.

```yaml
analog_reader:
  id: analog_main
  # Camera Reference
  camera_id: my_camera
  
  # The main sensor reporting the total aggregated value
  value_sensor:
    name: "Total Water Reading"
    
  # List of dials to read
  dials: 
    - id: dial_1
      # Location in the camera frame
      crop_x: 0
      crop_y: 0
      crop_w: 100
      crop_h: 100
      
      # Scale: How much is this dial worth? (e.g. x0.1, x0.001)
      scale: 1.0 
      
      # Angle Calibration
      # min/max_angle defines the physical arc of the gauge (e.g. 0 to 360 for full circle)
      min_angle: 0
      max_angle: 360
      
      # Angle Offset: Rotates the coordinate system.
      # 0   = North (12 o'clock)
      # 90  = East  (3 o'clock)
      # 180 = South (6 o'clock)
      # 270 = West  (9 o'clock)
      angle_offset: 0 
      
      # Value Mapping
      # What values do the min/max angles correspond to?
      min_value: 0
      max_value: 10
```

### Tuning Guide
1. **Crop**: Ensure your `crop_x/y/w/h` isolates **only** the dial. The center of the crop must be the center of the needle's axis.
2. **Lighting**: Ensure distinct contrast. The algorithm looks for a **dark needle on a light background** by default.
3. **Offset**: If your needle at "0" points upwards, use `angle_offset: 0`. If it points to the right, use `angle_offset: 90`.

## ⚙️ How it Works
1. **Extract**: The component extracts the defined crop from the camera image and resizes it to 128x128.
2. **Scan**: It scans 180 radial lines (every 2 degrees) from the center outwards.
3. **Sum**: It sums the pixel darkness along each line.
4. **Detect**: The angle with the darkest sum (for dark needles) is selected as the needle angle.
5. **Map**: The detected angle is converted to a value using the `min/max` settings.
