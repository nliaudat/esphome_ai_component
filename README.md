# ESPHome Meter Reader TFLite Component

> General-purpose TensorFlow Lite Micro implementation with image processing support for ESPHome

[![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-brightgreen)](https://esphome.io/)
[![Community Spotlight](https://img.shields.io/badge/ESPHome-Community%20Spotlight-blue)](https://esphome.io)
[![License](https://img.shields.io/badge/License-Apache%202.0%20OR%20MIT-blue)](https://github.com/nliaudat/esphome_ai_component)

## 🚀 Overview

This project provides a robust, modular framework for running TensorFlow Lite Micro models on ESP32 devices within the ESPHome ecosystem. While originally designed for analog/digital meter reading, it is suitable for various computer vision tasks.

**New in v2.0:** The project has been refactored into modular components for better maintainability and reusability.

### 🎯 Core Value Proposition
**"Turn any ESP32 Camera into a privacy-first, offline AI Meter Reader for ESPHome."**
Unlike other solutions that are standalone firmwares, this is a **native ESPHome component**, meaning you get all the power of the ESPHome ecosystem combined with edge AI:

*   **Broad Hardware Support**: Compatible with [many microcontrollers](https://esphome.io/components/) and camera boards (ESP32-CAM, ESP32-S3, Xiao, etc.).
*   **Infinite Extensibility**: Easily integrate with thousands of other [ESPHome components](https://esphome.io/) like **MQTT**, Displays, Time, or simple sensors.
*   **Native Control**: Native API, OTA updates, and simple YAML configuration come standard.

### 🚀 Reusable AI Core & Technical Excellence
Beyond just a meter reader, this project provides a robust foundation for **any** ESP32 AI project:

*   **`tflite_micro_helper`**: A standalone, reusable TFLite Micro model loader supporting **image** (single-shot) and **audio** (streaming) model types, with local/remote model fetching, CRC32 verification, and auto-detection from `.txt` reports. Used by `meter_reader_tflite` and `micro_wake_word`.
*   **Memory Safe & Optimized**: Written in modern **Safe C++**. We use `std::span`, zero-copy buffers, and `esp-nn` hardware acceleration to squeeze every drop of performance while preventing crashes and memory leaks.
*   **Production Ready**: Designed for 24/7 operation with robust error handling and watchdog protections.

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
| **[value_validator](./components/value_validator)** | Robust validation engine for meter readings. Eliminates outliers, tracks history, prevents impossible value jumps, and supports dial-aware digit correction when combined with `analog_reader`. Outputs float values when analog_reader is active. Publishes a separate `validated_value_sensor` (only on accept) while the raw value_sensor always shows raw digits. |
| **[esp32_camera_utils](./components/esp32_camera_utils)** | Powerful image processing utilities. Handles cropping, scaling, rotation (JPEG/Raw), and format conversion using `esp_new_jpeg` library. |
| **[tflite_micro_helper](./components/tflite_micro_helper)** | Self-contained TFLite Micro model loader and runtime wrapper. Supports **image** and **audio** model types, model source resolution (local, `github://`, `http(s)://`), CRC32 verification, auto-detection from `.txt` reports, and ESP-NN hardware acceleration. Required by `meter_reader_tflite` and `micro_wake_word`. |
| **[flash_light_controller](./components/flash_light_controller)** | Manages flash light timing for optimal image capture conditions. |
| **[data_collector](./components/data_collector)** | **Active Learning** tool. Automatically collects "hard" images (low confidence) and uploads them to a server for training set improvement. |

### Legacy

| Component | Description |
|-----------|-------------|
| **[legacy_meter_reader_tflite](./components/legacy_meter_reader_tflite)** | The previous monolithic version, kept for backward compatibility. |

## 🏁 Quick Start

> [!IMPORTANT]
> **The project MUST be compiled after cloning locally.** The components ship as C++ source code, not pre-built binaries. After cloning the repository, install ESPHome and compile your configuration:

```bash
pip install esphome
esphome compile config.yaml
```

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
      - data_collector        # Optional: For collecting training data
```

### 2. Basic Configuration

#### Option A: AI-Powered Meter Reader (TFLite)

Model configuration is now handled by the dedicated **`tflite_micro_helper`** component — define the model there and reference it by ID:

```yaml
esp32_camera:
  name: "My Camera"
  resolution: 640x480
  pixel_format: JPEG

value_validator:
  id: my_validator
  allow_negative_rates: false
  max_absolute_diff: 300

# 1. Define the model (auto-detected from .txt report)
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer.tflite"

# 2. Reference the model by ID from consumer component
meter_reader_tflite:
  id: my_meter_reader
  model: my_digit_model          # ← ID reference
  camera_id: my_camera
  validator: my_validator
  update_interval: 60s

  # Optional: Link to other components
  flash_light_controller: my_flash_controller
  crop_zones_global: my_crop_zones

  # Image Rotation (Dev)
  rotation: "90" # Options: "0", "90", "180", "270"
```

> **Note:** The legacy `model: "file.tflite"` syntax inside `meter_reader_tflite` still works for backward compatibility, but prints a deprecation warning. See the [meter_reader_tflite README](./components/meter_reader_tflite) for details.

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

#### Option D: Data Collector (Active Learning)

This optional component automatically uploads images when the model is unsure, helping you improve your model over time.

```yaml
data_collector:
  id: my_data_collector
  upload_url: "http://192.168.1.50:5123/api/upload/my_meter"
  # Optional: Secure your uploads
  # api_key: "change-me-to-a-secure-key"

meter_reader_tflite:
  # ... existing config ...
  data_collector: my_data_collector
  collect_low_confidence: true # Trigger upload on low confidence (uses min_global/digit_confidence)

  # Optional: Customize collection thresholds (defaults to 0.90)
  collect_min_global_confidence: 0.80
  collect_min_digit_confidence: 0.80
```

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
- **Solution**: Increase `tensor_arena_size` in the `tflite_micro_helper` config (auto-detected from `.txt` report by default).
```yaml
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer.tflite"
    tensor_arena_size: "768KB"
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
*   **Performance**: Capable of full image processing and 8-digit inference in **less than 60 ms** [esp32-S3] (1150 ms for legacy esp32) on v3 model.

These models are recommended for faster response times and lower power consumption.

## 🆘 Call for Contributors

**This project needs your help!**

This component has grown to **over 20,000 lines of C++ code**, implementing complex features like Neural Network inference (`tflite-micro`), Image Processing, and native ESPHome integration.

> [!NOTE]
> **AI-Assisted Development**: While the legacy version was manually written, this new modular version (v2.0) leverages AI assistance to achieve advanced features like **dual-core optimizations**, **complex memory management**, and **leak prevention** that were beyond my initial ESPHome knowledge. While the results are excellent and performance is high, this approach has introduced complexity that needs human review.

It was a significant effort to develop, and maintaining it alone is becoming increasingly difficult. I cannot respond to every discussion, issue, or feature request on my own.

**We are looking for:**
*   **C++ Developers**: optimization, bug fixes, and new features. To **review the code** and refine the AI-generated portions.
*   **Power Users**: to help answer questions in discussions and test new releases.
*   **Documentation Writers**: to help keep the wiki and guides up to date.
*   **Promoters**: I don't use social media (personal choice to keep my life quiet), so I rely on you to talk about this project!

If you love this project and want to see it thrive, please consider contributing!

## 📄 License

Licensed under either of:

    Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
    MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)

at your option.
