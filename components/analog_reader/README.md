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
value_validator:
  id: ${id_prefix}_validator
  allow_negative_rates: true  # Analog gauges can go both ways
  max_rate_change: 0.25

analog_reader:
  id: analog_main
  # Camera Reference
  camera_id: my_camera
  
  # Global Settings
  update_interval: 60s
  paused: false         # Start paused?
  debug: false          # Enable ASCII dial visualization in logs
  
  # Validator (recommended)
  validator: ${id_prefix}_validator
  
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
      
      # Image Processing
      auto_contrast: true   # Enhances contrast before processing (Recommended)
      contrast: 1.0         # Manual contrast multiplier
      
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

### UI Controls Package (Optional)
To easily adjust settings (Pause, Contrast, Update Interval) from Home Assistant, include the provided controls package:

```yaml
packages:
  analog_controls: !include analog_reader_controls.yaml
```

This exposes:
- **Pause Switch**: Stop processing to check logs or save power.
- **Auto Contrast Switch**: Toggle enhancement.
- **Contrast Slider**: Fine-tune contrast (0.1 - 5.0).
- **Update Interval**: Adjust poll rate dynamically.
- **Algorithm Selector**: Switch between detection algorithms (Legacy, Radial Profile, Hough, Template, Auto) at runtime.

### Tuning Guide
1. **Crop**: Ensure your `crop_x/y/w/h` isolates **only** the dial. The center of the crop must be the center of the needle's axis.
2. **Lighting**: Ensure distinct contrast between needle and background. Use `auto_contrast: true` to help normalize lighting conditions.
3. **Offset**: If your needle at "0" points upwards, use `angle_offset: 0`. If it points to the right, use `angle_offset: 90`.
4. **Debugging**: Enable `debug: true` to see an ASCII art representation of the dial in the logs (with colorized needle). This confirms exactly what the ESP32 sees and where it thinks the needle is.

## ⚙️ How it Works
1. **Decode Once**: The full camera frame is decoded to RGB888 just once, optimizing memory for multi-dial setups.
2. **Extract & Enhance**: Each dial crop is extracted. Auto-contrast is applied if enabled (Min-Max normalization).
3. **Radial Edge Detection**: 
   - Scans from center OUTWARD along 360 radial lines (1° resolution)  
   - Detects the **strongest brightness gradient** (edge transition) along each line
   - This is where the needle crosses that angle
   - Works for both dark AND light needles (gradient is unsigned)
   - Avoids ambiguity from detecting both ends of needle
4. **Detect**: The angle with the **maximum gradient** (sharpest edge) is selected as the needle direction.
5. **Map**: The detected angle is normalized to North-based coordinates and mapped to a value using the configuration.
