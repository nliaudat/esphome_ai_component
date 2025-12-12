# ESP32 Camera Utils

A dedicated component for advanced image processing operations on ESP32, specifically designed to prepare images for AI models. However, for the **Meter Reader** component, you should configure rotation directly in the `meter_reader_tflite` configuration.

## ✨ Features

- **esp_new_jpeg Integration**: Uses the v1.0.0+ library for efficient JPEG decoding.
- **Image Rotation**: Supports 0°, 90°, 180°, and 270° rotation.
    - **JPEG**: Hardare-assisted rotation during decoding.
    - **Raw (RGB/Gray)**: Optimized software rotation.
- **Format Conversion**: Handles RGB888, RGB565, Grayscale, and JPEG.
- **Cropping & Scaling**: Efficiently extracts zones and scales them to model input dimensions.
- **Normalization**: Converts standard pixel data to float32 (0.0-1.0) or int8/uint8 specific ranges.

## ⚙️ Configuration

This component is responsible for image rotation and preprocessing.

```yaml
esp32_camera_utils:
  id: camera_utils
  camera_id: my_camera # ID of the esp32_camera component
  
  # Global Rotation Setting
  rotation: "90"  # Options: "0", "90", "180", "270"
  
  # Camera Windowing (Hardware Cropping)
  # Useful for OV2640/OV3660 to zoom in before capture
  camera_window:
    offset_x: 928
    offset_y: 480
    width: 448
    height: 88
    
  debug: false
```

## 🛠️ Internal Usage

The `ImageProcessor` class provides the core functionality:

```cpp
#include "esphome/components/esp32_camera_utils/image_processor.h"

// Configuration
ImageProcessorConfig config;
config.width = 640;
config.height = 480;
config.rotation = ROTATION_90;

// Processing
ImageProcessor processor(config);
processor.process_zone_to_buffer(image, zone, output_buffer, size);
```

## 🔍 Debugging

Enable `debug: true` to see:
- Buffer allocation details
- JPEG decoding performance stats
- Rotation processing times
- Zone coordinate validations
