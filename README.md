# ESPHome Meter Reader TFLite Component

> General-purpose TensorFlow Lite Micro implementation with image processing support for ESPHome

[![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-brightgreen)](https://esphome.io/)

## 🚀 Overview

This project provides a robust, modular framework for running TensorFlow Lite Micro models on ESP32 devices within the ESPHome ecosystem. While originally designed for analog/digital meter reading, it is suitable for various computer vision tasks.

**New in v2.0:** The project has been refactored into modular components for better maintainability and reusability.

## 📦 Components

The repository allows you to use specific components based on your needs:

### Core Reading Components

| Component | Description |
|-----------|-------------|
| **[meter_reader_tflite](./components/meter_reader_tflite)** | AI-powered meter reader using TensorFlow Lite models. Orchestrates capture, inference, and reporting for digital meters. |
| **[ssocr_reader](./components/ssocr_reader)** | (Alpha dev !) Seven-segment OCR reader using SSOCR algorithm. Reads digital displays without AI models. |
| **[analog_reader](./components/analog_reader)** | (Alpha dev!) Analog dial/gauge reader using radial intensity sum algorithm. Reads pointer positions without AI. |

### Supporting Components

| Component | Description |
|-----------|-------------|
| **[value_validator](./components/value_validator)** | Robust validation engine for meter readings. Eliminates outliers, tracks history, and prevents impossible value jumps. |
| **[esp32_camera_utils](./components/esp32_camera_utils)** | Powerful image processing utilities. Handles cropping, scaling, rotation (JPEG/Raw), and format conversion using `esp_new_jpeg` library. |
| **[tflite_micro_helper](./components/tflite_micro_helper)** | Wrapper for TensorFlow Lite Micro runtime (checking model CRC32, etc..) and `esp-nn` optimizations. Manages tensor arena and model loading. |
| **[flash_light_controller](./components/flash_light_controller)** | Manages flash light timing for optimal image capture conditions. |

### Legacy

| Component | Description |
|-----------|-------------|
| **[legacy_meter_reader_tflite](./components/legacy_meter_reader_tflite)** | The previous monolithic version, kept for backward compatibility. |

## 🏁 Quick Start

### 1. Installation

***A detailed procedure for setting up is available in [wiki/setup page](https://github.com/nliaudat/esphome_ai_component/wiki/Setup)***

Add the components to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nliaudat/esphome_ai_component
      ref: main
    components: 
      # Choose the reader type you need:
      - meter_reader_tflite   # AI-powered digit recognition
      # - ssocr_reader        # Seven-segment OCR (no AI)
      # - analog_reader       # Analog gauge/dial reader (no AI)
      
      # Supporting components:
      - value_validator       # Validation engine (recommended)
      - tflite_micro_helper   # Required for meter_reader_tflite
      - esp32_camera_utils    # Image processing utilities
      - flash_light_controller
```

### 2. Basic Configuration

#### Option A: AI-Powered Meter Reader (TFLite)

```yaml
esp32_camera:
  name: "My Camera"
  resolution: 640x480
  pixel_format: JPEG

value_validator:
  id: my_validator
  allow_negative_rates: false
  max_absolute_diff: 300

meter_reader_tflite:
  id: my_meter_reader
  model: "digit_recognizer.tflite"
  camera_id: my_camera
  # Optional: Validator
  validator: my_validator

  # If no validator is set, use simple threshold:
  # confidence_threshold: 0.85 

  update_interval: 60s
  
  # Optional: Link to other components
  flash_light_controller: my_flash_controller
  crop_zones_global: my_crop_zones
  
  # Image Rotation (Dev)
  rotation: "90" # Options: "0", "90", "180", "270"
```

#### Option B: Seven-Segment OCR Reader (No AI)

```yaml
value_validator:
  id: my_validator
  allow_negative_rates: false
  max_absolute_diff: 50

ssocr_reader:
  id: my_ssocr
  camera_id: my_camera
  validator: my_validator
  update_interval: 60s
  threshold_level: 128
  digit_count: 8
```

#### Option C: Analog Gauge Reader (No AI)

```yaml
value_validator:
  id: my_validator
  allow_negative_rates: true  # Analog gauges can go both ways
  max_rate_change: 0.25

analog_reader:
  id: my_analog
  camera_id: my_camera
  validator: my_validator
  update_interval: 60s
  dials:
    - min_value: 0
      max_value: 100
      radius_min: 20
      radius_max: 80
```

> [!NOTE]
> **Rotation behavior**: The `rotation` setting only affects the AI processing. The internal webserver and Home Assistant live view will likely remain unrotated (sideways) as the camera hardware only supports 180° flips. This is normal behavior.

### 3. Runtime Configuration (Home Assistant)

Most parameters are now exposed to Home Assistant for real-time adjustment without recompiling:

*   **Calibration**: Camera Window (Offset X/Y, Width, Height)
*   **Timing**: Flash Pre/Post times, Update Interval
*   **Debug**: Toggle debug logging and image generation
*   **Settings**: Meter units and thresholds

These entities normally appear under the device in Home Assistant.


## ✨ Key Features

- **🤖 TensorFlow Lite Micro**: Full TFLite Micro runtime support (with operators detection and auto loading)
- **📷 Camera Integration**: State of the art ESP32 camera integration with TrackedBuffer for better memory management and windowing for OV2640 like camera
- **🖼️ Image Preprocessing**: Automatic cropping, scaling, and format conversion
- **⚡ Optimized Performance**: ESP-NN accelerated operations
- **🎯 Multi-Zone Processing**: Process multiple regions of interest
- **🔧 Flexible Configuration**: Support for various model types and input formats
- **🐛 Advanced Debugging**: Real-time image analysis and model output inspection
- **🔄 Image Rotation (Dev)**: Full support for 0°, 90°, 180°, 270° rotation on both JPEG and Raw formats (via `esp32_camera_utils`).

## 🎯 Use Cases

### Meter Reading
- Water, electricity, gas meter digit recognition
- Analog gauge reading
- Digital display extraction

### Computer Vision Applications
- **Object Detection**: Identify objects in camera frames
- **Image Classification**: Categorize images into classes
- **Anomaly Detection**: Detect unusual patterns or events
- **Quality Control**: Inspect products or components

## 🔍 Troubleshooting

### Common Issues

**❌ Model loading fails**
- **Cause**: Tensor arena size is too small.
- **Solution**: Increase `tensor_arena_size` in `meter_reader_tflite` config.
```yaml
meter_reader_tflite:
  tensor_arena_size: 768KB  # Default is 512KB
```

**❌ Poor inference results**
- **Cause**: Input image poor quality or wrong crop.
- **Solution**: Enable debug mode to see what the model actually "sees".
```yaml
meter_reader_tflite:
  debug: true
```

**❌ Camera frame issues**
- **Cause**: Low memory or bandwidth issues.
- **Solution**: Lower resolution or framerate in `esp32_camera`.

### Performance Tips
1. **Use quantized models** (int8) for better performance
2. **Enable ESP-NN optimizations** (enabled by default)
3. **Use appropriate tensor_arena_size** (already calculated per model)

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

## 🧠 Optimized Models

New optimized models are available for digit recognition that significantly improve performance:

*   **Source**: [nliaudat/digit_recognizer](https://github.com/nliaudat/digit_recognizer)
*   **Performance**: Capable of full image processing and 8-digit inference in **less than 270 ms** [esp32-S3] (2700 ms for legacy esp32).

These models are recommended for faster response times and lower power consumption.

## 📄 License

* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC-BY-NC-SA)
* No commercial use
