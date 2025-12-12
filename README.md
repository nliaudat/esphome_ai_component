# ESPHome Meter Reader TFLite Component

> General-purpose TensorFlow Lite Micro implementation with image processing support for ESPHome

[![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-brightgreen)](https://esphome.io/)

## 🚀 Overview

This project provides a robust, modular framework for running TensorFlow Lite Micro models on ESP32 devices within the ESPHome ecosystem. While originally designed for analog/digital meter reading, it is suitable for various computer vision tasks.

**New in v2.0:** The project has been refactored into modular components for better maintainability and reusability.

## 📦 Components

The repository allows you to use specific components based on your needs:

| Component | Description |
|-----------|-------------|
| **[meter_reader_tflite](./components/meter_reader_tflite)** | The main component for running AI models on camera images. Orchestrates capture, inference, and reporting. |
| **[esp32_camera_utils](./components/esp32_camera_utils)** | Powerful image processing utilities. Handles cropping, scaling, rotation (JPEG/Raw), and format conversion using `esp_new_jpeg` library. |
| **[tflite_micro_helper](./components/tflite_micro_helper)** | Wrapper for TensorFlow Lite Micro runtime (checking model CRC32, etc..) and `esp-nn` optimizations. Manages tensor arena and model loading. |
| **[flash_light_controller](./components/flash_light_controller)** | Manages flash light timing for optimal image capture conditions. |
| **[legacy_meter_reader_tflite](./components/legacy_meter_reader_tflite)** | The previous monolithic version, kept for backward compatibility. |

## 🏁 Quick Start

### 1. Installation

Add the components to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nliaudat/esphome_ai_component
      ref: main
    components: 
      - meter_reader_tflite
      - tflite_micro_helper
      - esp32_camera_utils
      - flash_light_controller
```

### 2. Basic Configuration

```yaml
esp32_camera:
  name: "My Camera"
  # ... standard camera config ...

meter_reader_tflite:
  id: my_meter_reader
  model: "my_model.tflite"
  camera_id: my_camera
  update_interval: 60s
  rotation: "90" # Optional: Rotate image 0, 90, 180, 270
```

*See individual component READMEs for detailed configuration options.*

## 🔄 Backward Compatibility

If you need to use the old version, specify the legacy component:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nliaudat/esphome_ai_component
      ref: main
    components: [legacy_meter_reader_tflite]
```

## ✨ Key Features

- **Modular Architecture**: Use only what you need.
- **Image Rotation**: Full support for 0°, 90°, 180°, 270° rotation on both JPEG and Raw formats.
- **Optimized Performance**: Hardware-accelerated operations where available.
- **Advanced Debugging**: Comprehensive debug modes for image analysis and model inspection.

## 📄 License

* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC-BY-NC-SA)
* No commercial use
* AI models from [haverland](https://github.com/haverland/Tenth-of-step-of-a-meter-digit) are under Apache Licence
