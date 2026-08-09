# TFLite Micro Helper Component

**`tflite_micro_helper`** is a self-contained TFLite Micro model loader and runtime wrapper for ESPHome. It handles model source resolution, config auto-detection, CRC32 verification, tensor arena management, and ESP-NN hardware acceleration — supporting both **image** (single-shot) and **audio** (streaming) model types.

## 🚀 Overview

Instead of each consumer component (`meter_reader_tflite`, `micro_wake_word`) reimplementing model loading and configuration, `tflite_micro_helper` becomes the **single source of truth** for all TFLite model management:

```yaml
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "models/digit_recognizer_v40.tflite"
    # All config auto-detected from .txt report

  - id: my_wake_word
    model_type: audio
    model: "github://esphome/micro-wake-word-models/models/v2/okay_nabu.json"
```

Consumer components reference models by ID:

```yaml
meter_reader_tflite:
  model: my_digit_model          # ← ID reference to tflite_micro_helper entry

micro_wake_word:
  models:
    - tflite_model: my_wake_word # ← ID reference via tflite_model:
      wake_word: "okay nabu"
      trained_languages: ["en"]
```

## 🎯 Model Types

| Type | Shape | Use Case | Inference Mode |
|------|-------|----------|----------------|
| **image** | `[1, H, W, C]` (4D) | Digit recognition, classification, object detection | Single-shot per image |
| **audio** | `[1, stride, features]` (3D) | Wake word detection, VAD | Streaming with `MicroResourceVariables` |

## 📦 Model Sources

| Source | Format | Example |
|--------|--------|---------|
| **Local** | Direct `.tflite` path | `model: "digit_recognizer_v40.tflite"` |
| **github://** | Shorthand URL | `model: "github://owner/repo/path/file.tflite[@ref]"` |
| **http(s)://** | JSON manifest URL | `model: "https://example.com/models/my_model.json"` |

### Auto-Detection from `.txt` Files

When a `.txt` report file exists alongside the `.tflite` file, the following are auto-detected:

- **Input shape**: width, height, channels, data type (`uint8`/`float32`)
- **Output class count**: determines `scale_factor` (10cls → 1.0, 100cls → 10.0)
- **Tensor arena size**: from peak memory analysis
- **Output processing**: `direct_class` if SOFTMAX present, else `softmax`
- **Operator count**: for `MAX_OPERATORS` build flag
- **Hybrid quantization**: detected and rejected with a clear error

**No `.txt` file?** Fallback to filename heuristics (`_GRAY`, `_RGB`, `_BGR`, `_10cls`, `_100cls`).

## ⚙️ Configuration

### Image Model (Single-Shot)

```yaml
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"

    # Optional overrides (auto-detected from .txt file by default):
    # input_type: "uint8"
    # input_channels: 3
    # input_width: 32
    # input_height: 20
    # output_processing: "direct_class"   # Options: direct_class, softmax, argmax, logits, qat_quantized, etc.
    # scale_factor: 1.0
    # input_order: "RGB"                    # Options: RGB, BGR, GRAY
    # normalize: false
    # invert: false
    # tensor_arena_size: "110KB"
    # debug: false
```

### Audio Model (Streaming)

```yaml
tflite_micro_helper:
  - id: my_wake_word
    model_type: audio
    model: "models/wake_word_model.tflite"

    # Audio-specific config (optional, auto-detected from .txt file):
    # probability_cutoff: 0.7
    # sliding_window_size: 10
    # features_step_size: 20          # ms
    # feature_count: 40               # Mel filterbank bins
    # tensor_arena_size: "30KB"
    # debug: false
```

### Remote Model via `github://`

```yaml
tflite_micro_helper:
  - id: remote_model
    model_type: image
    model: "github://nliaudat/esphome_ai_component/models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
```

### Remote Model via HTTP(s) Manifest

```yaml
tflite_micro_helper:
  - id: http_model
    model_type: image
    model:
      type: http
      url: "https://example.com/models/meter_model.json"
```

### Multiple Models

```yaml
tflite_micro_helper:
  - id: cold_model
    model_type: image
    model: "models/cold_water_v40.tflite"

  - id: hot_model
    model_type: image
    model: "models/hot_water_v40.tflite"

  - id: wake_model
    model_type: audio
    model: okay_nabu
```

## 🔗 Integration with Consumer Components

### meter_reader_tflite (Image)

```yaml
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer_v40.tflite"

meter_reader_tflite:
  id: my_meter
  model: my_digit_model          # ← ID reference
  camera_id: my_camera
  update_interval: 60s
  confidence_threshold: 0.85
```

### micro_wake_word (Audio)

```yaml
tflite_micro_helper:
  - id: my_wake_word
    model_type: audio
    model: okay_nabu

micro_wake_word:
  models:
    - tflite_model: my_wake_word  # ← ID reference via tflite_model:
      wake_word: "okay nabu"
      trained_languages: ["en"]
      probability_cutoff: 0.7
      sliding_window_size: 10
```

## 🔄 Model Lifecycle (C++ API)

The helper supports unloading and reloading models at runtime for dynamic resource management.

### Same-model reload (preserves config)

The `meter_reader_tflite::reload_resources()` path calls `unload_model()` followed by `load_model()`
**without** re-issuing `set_model()` / `set_expected_crc32()`. This is the default behavior:

```cpp
// Config is preserved -- reload uses the same model_data_, arena size, dimensions, etc.
tflite->unload_model();           // reset_config defaults to false
tflite->load_model();
```

### Switching to a different model (P1: reset stale config)

When a consumer unloads one model and assigns a **different** model without overwriting every
setting, `unload_model()` would otherwise retain the previous model's arena size, input
dimensions, preprocessing, and audio configuration -- causing incorrect inference or
tensor-allocation failure on the next `load_model()`.

Call `unload_model(true)` (or `reset_config()`) before configuring the new model:

```cpp
// Switch to a completely different model:
tflite->unload_model(true);       // also calls reset_config()
tflite->set_model(new_data, new_size);
tflite->set_expected_crc32(new_crc);
// ... re-issue all relevant setters ...
tflite->load_model();
```

`reset_config()` clears every model-config field to its type-safe default (matching the
initial state defined in `tflite_micro_helper.h`): model data pointer/length, arena size,
expected CRC, image config, and audio config.

> **Note:** `TFLiteCoordinator::unload_model(bool reset_config = false)` delegates the same
> flag through to the underlying `TFLiteMicroHelper`.

## ⚡ ESP-NN Optimizations

The component automatically enables hardware acceleration:

- **Build flags**: `TF_LITE_STATIC_MEMORY`, `ESP_NN`, `OPTIMIZED_KERNEL=esp_nn`
- **IDF components**: `espressif/esp-tflite-micro` v1.3.7, `espressif/esp-nn` v1.2.3
- **Audio only**: `esphome/esp-micro-speech-features` v1.2.3
- **ESP32-S3**: Automatic 1.5x tensor arena bump for cache alignment

## 🔍 Debugging

```yaml
tflite_micro_helper:
  - id: my_model
    model_type: image
    model: "model.tflite"
    debug: true          # Enable detailed logging
```

With debug enabled, the component logs:
- Model loading status and CRC32 verification
- Tensor arena allocation details
- Inference timing
- Memory usage and efficiency statistics

## 🛡️ Model Integrity

Each model is verified at compile time with a CRC32 checksum:

```yaml
# Computed automatically from the model data in __init__.py
# Verified at runtime by load_model()
```

If the CRC32 doesn't match, the model is rejected with a clear error message — preventing silent corruption from incomplete downloads or storage errors.

## 📋 Configuration Variables

### Common (All Model Types)

| Variable | Required | Description |
|----------|----------|-------------|
| `id` | Yes | The ID of this model entry |
| `model_type` | Yes | `"image"` or `"audio"` |
| `model` | Yes | Model source: local path, `github://` shorthand, or `http(s)://` URL |
| `tensor_arena_size` | No | Override arena size (e.g. `"110KB"`). Auto-detected from `.txt` file. |
| `debug` | No | Enable debug logging. Default: `false` |

### Image Model Overrides (Optional)

| Variable | Default | Description |
|----------|---------|-------------|
| `input_type` | `"uint8"` | Input data type: `"uint8"` or `"float32"` |
| `input_channels` | `3` | Number of input channels (1-4) |
| `input_width` | `32` | Input image width (8-512) |
| `input_height` | `20` | Input image height (8-512) |
| `output_processing` | auto | Processing mode: `direct_class`, `softmax`, `argmax`, `logits`, `qat_quantized`, `experimental_scale`, `logits_jomjol`, `softmax_jomjol`, `auto_detect` |
| `scale_factor` | `1.0` | Output scale factor (0.1-100.0) |
| `input_order` | `"RGB"` | Channel order: `"RGB"`, `"BGR"`, or `"GRAY"` |
| `normalize` | `false` | Normalize pixel values to 0.0-1.0 |
| `invert` | `false` | Invert pixel values |

### Audio Model Overrides (Optional)

| Variable | Default | Description |
|----------|---------|-------------|
| `probability_cutoff` | auto | Detection threshold (0.0-1.0). Auto-detected from `.txt` file. |
| `sliding_window_size` | auto | Number of inference results to average. Auto-detected from `.txt` file. |
| `features_step_size` | auto | Step size in ms (0-30). Auto-detected from `.txt` file. |
| `feature_count` | auto | Number of Mel filterbank bins (10-80). Auto-detected from `.txt` file. |

## 📝 Example: Standalone Image Classification

```yaml
# Use tflite_micro_helper with a custom C++ component:
tflite_micro_helper:
  - id: classifier
    model_type: image
    model: "models/classifier.tflite"
```

Then in your custom component:

```cpp
#include "esphome/components/tflite_micro_helper/tflite_micro_helper.h"

class MyClassifier : public Component {
 public:
  void set_model(tflite_micro_helper::TFLiteMicroHelper *tflite) {
    this->tflite_ = tflite;
  }

  void setup() override {
    if (!this->tflite_->load_model()) {
      ESP_LOGE("classifier", "Failed to load model");
      return;
    }
  }

  void classify(const uint8_t *image_data, size_t image_size) {
    auto output = this->tflite_->run_inference(image_data, image_size);
    ESP_LOGI("classifier", "Class: %d, Confidence: %.3f",
             output.class_id, output.confidence);
  }

 protected:
  tflite_micro_helper::TFLiteMicroHelper *tflite_;
};
```

## 🔄 Backward Compatibility

The `meter_reader_tflite` component still supports the legacy `model: "file.tflite"` syntax for migration purposes. When used, a deprecation warning is printed:

```
WARNING: Deprecated model configuration in meter_reader_tflite.
  The 'model: "<file>.tflite"' syntax is deprecated.
  Please move model configuration to the tflite_micro_helper section instead.
```

## 📄 License

Apache 2.0 OR MIT, at your option.