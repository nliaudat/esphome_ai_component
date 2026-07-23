# tflite_micro_helper Update Plan

> **Goal:** Transform `tflite_micro_helper` from an image-only, passively-configured helper into a generalist, self-contained TFLite model loader that supports **image** and **audio** model types — with remote model fetching, and becomes the single source of truth for model management across all consumer components.

---

## Table of Contents

1. [Current State](#current-state)
2. [Target State](#target-state)
3. [Model Sources (local, remote, shorthand)](#model-sources)
4. [Model Types (image vs audio)](#model-types)
5. [C++ Architecture Changes](#c-architecture-changes)
6. [Python (__init__.py) Changes](#python-__init__py-changes)
7. [Consumer Migration (meter_reader_tflite)](#consumer-migration-meter_reader_tflite)
8. [Example YAMLs](#example-yamls)
9. [Backward Compatibility](#backward-compatibility)
10. [Implementation Order](#implementation-order)
11. [File Change Summary](#file-change-summary)
12. [Inspiration from micro_wake_word](#inspiration-from-micro_wake_word)

---

## Current State

```
                        meter_reader_tflite
                        ┌─────────────────────────────────┐
                        │ __init__.py                     │
                        │  ├─ Reads .tflite file          │
                        │  ├─ Parses .txt for config      │
                        │  ├─ Creates PROGMEM array       │
                        │  ├─ Computes CRC32              │
                        │  ├─ Sets MAX_OPERATORS          │
                        │  └─ Calls var.set_model(...)    │
                        └──────────┬──────────────────────┘
                                   │ passive delegation
                        ┌──────────▼──────────────────────┐
                        │ tflite_micro_helper             │
                        │  ├─ TFLiteMicroHelper (wrapper)  │
                        │  ├─ ModelHandler (image-only)   │
                        │  ├─ MemoryManager               │
                        │  └─ OpResolverManager           │
                        │                                 │
                        │  ❌ No model loading             │
                        │  ❌ No .txt parsing              │
                        │  ❌ No remote fetching            │
                        │  ❌ No audio / streaming          │
                        │  ❌ Image assumptions baked in   │
                        └─────────────────────────────────┘
```

**Problems:**

| Issue | Detail |
|-------|--------|
| Model loading scattered | `meter_reader_tflite/__init__.py` owns file reading, .txt parsing, PROGMEM creation — a future audio consumer would duplicate all of this |
| Image-only assumptions | `ModelConfig` has `input_width/height/channels`, `normalize`, `invert`, `input_order` baked in. Audio models are 3D tensors with different metadata. |
| No streaming support | `ModelHandler::invoke()` is single-shot. Audio needs stateful inference via `MicroResourceVariables`. |
| No remote models | `micro_wake_word` supports `github://`, `http://`, and local paths. `tflite_micro_helper` is local-only. |
| No model_type | Everything is treated as image. Audio models need different IDF components, build flags, config. |

---

## Target State

```
                        tflite_micro_helper (SELF-CONTAINED)
                        ┌──────────────────────────────────────────┐
                        │ __init__.py                              │
                        │  ├─ model_type: image | audio            │
                        │  ├─ Source resolution:                   │
                        │  │   ├─ local: path/to/model.tflite       │
                        │  │   ├─ github://owner/repo/file.json    │
                        │  │   └─ http(s)://url/manifest.json     │
                        │  ├─ Config auto-detection:               │
                        │  │   ├─ image → parse .txt file          │
                        │  │   └─ audio → parse .txt or .json      │
                        │  ├─ PROGMEM array creation               │
                        │  ├─ MAX_OPERATORS build flag             │
                        │  ├─ Per-type IDF components:             │
                        │  │   ├─ image: esp-tflite-micro + esp-nn │
                        │  │   └─ audio: + esp-micro-speech-feat.  │
                        │  └─ Exposes model ID                    │
                        └──────────┬───────────────────────────────┘
                                   │ model_id reference
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
 meter_reader_tflite        future_audio_consumer       any_other_consumer
  "model: my_image_model"    "model: my_wake_word"       "model: my_model"
```

---

## Model Sources

Inspired by `micro_wake_word` which supports local paths, `github://` shorthand, and `http://` URLs via JSON manifests.

### Source Types

| Source | Format | Example |
|--------|--------|---------|
| **local** | Direct `.tflite` path + `.txt` auto-config | `model: "digit_recognizer_v40.tflite"` |
| **github** | `github://owner/repo/path/to/manifest.json[@ref]` | `model: github://esphome/micro-wake-word-models/models/v2/okay_nabu.json` |
| **http** | `http(s)://url/to/manifest.json` | `model: https://example.com/models/wake_word.json` |

### Model Name Shorthand (audio)

```yaml
# Short form (like micro_wake_word):
model: okay_nabu
# Resolves to: github://esphome/micro-wake-word-models/models/v2/okay_nabu.json

# Fully qualified:
model:
  type: git
  url: https://github.com/custom/models
  ref: main
  file: my_model.json
```

### Config Resolution Pipeline

```
YAML input
  │
  ├─ Is it a shorthand string? ──▶ Try model name → github URL
  │                               Try file path  → local
  │                               Try URL        → http
  │                               Try git        → git
  │
  ├─ Is it a dict with 'type'? ──▶ git / local / http
  │
  └─ Download/fetch → Extract .tflite + config → PROGMEM → C++ object
```

---

## Model Types

### `model_type: image` (existing, refactored)

| Aspect | Detail |
|--------|--------|
| Config source | `.txt` file (existing `parse_model_txt_file()`) |
| Model shape | 4D `[1, H, W, C]` |
| Inference | Single-shot `invoke()` per image |
| Output | `process_output()` → value + confidence |
| Config keys | `input_width`, `input_height`, `input_channels`, `input_order`, `normalize`, `invert`, `output_processing`, `scale_factor` |
| IDF components | `esp-tflite-micro` + `esp-nn` |
| Build defines | `TF_LITE_STATIC_MEMORY`, `ESP_NN`, `OPTIMIZED_KERNEL=esp_nn` |

### `model_type: audio` (new)

| Aspect | Detail |
|--------|--------|
| Config source | `.txt` file or `.json` manifest (format TBD) |
| Model shape | 3D `[1, stride, features]` |
| Inference | Streaming via `StreamingModel` (stateful, MRV-backed) |
| Output | `determine_detected()` → DetectionEvent (average/max probability, detected bool) |
| Config keys | `probability_cutoff` (0.0-1.0), `sliding_window_size`, `features_step_size` (ms), `feature_count` (Mel bins), `tensor_arena_size` |
| IDF components | `esp-tflite-micro` + `esp-nn` + `esp-micro-speech-features` |
| Build defines | `TF_LITE_STATIC_MEMORY`, `ESP_NN`, `OPTIMIZED_KERNEL=esp_nn`, `USE_TFLITE_STREAMING` |

### Audio Model Config Format

Audio models can use the same `.txt` format as image models (for consistency) or a `.json` manifest (like micro_wake_word). Preferred approach:

**Option A: `.txt` (consistent with image models, minimal new code)**

```
# Generated alongside wake_word_model.tflite
Input 0:  [ 1 5 40]   <class 'numpy.int8'>
Output 0: [ 1 1]      <class 'numpy.uint8'>
Total operations: 25
Recommended tensor_arena_size: 30KB
model_type: audio
probability_cutoff: 0.7
sliding_window_size: 10
features_step_size: 20
feature_count: 40
```

**Option B: `.json` manifest (like micro_wake_word, future-proof)**

```json
{
  "type": "micro",
  "wake_word": "okay_nabu",
  "version": 2,
  "model": "wake_word_model.tflite",
  "trained_languages": ["en"],
  "micro": {
    "feature_step_size": 20,
    "tensor_arena_size": 30000,
    "probability_cutoff": 0.7,
    "sliding_window_size": 10,
    "minimum_esphome_version": "2025.1.0"
  }
}
```

→ **Recommendation: Support both.** Try `.json` first (like micro_wake_word), fall back to `.txt`, then to user-provided YAML values.

---

## C++ Architecture Changes

### New Config Hierarchy

```cpp
// All models share this
struct ModelConfig {
  std::string model_type;       // "image" or "audio"
  std::string description;
  std::string output_processing;
  float scale_factor{1.0f};
};

// Image models only
struct ImageModelConfig {
  int input_width{32};
  int input_height{20};
  int input_channels{3};
  std::string input_order{"RGB"};
  bool normalize{false};
  bool invert{false};
};

// Audio / streaming models only
struct AudioModelConfig {
  uint8_t probability_cutoff{128};     // quantized 0-255 (from 0.0-1.0)
  size_t sliding_window_size{10};
  uint8_t features_step_size{20};      // ms
  size_t feature_count{40};            // e.g. 40 Mel filterbank bins
};
```

### `ModelHandler` — Optional MRV Parameter (No Separate StreamingModel Class)

**Decision: Avoid creating a separate `StreamingModel` class entirely.**
Instead, add an optional `MicroResourceVariables*` parameter to `ModelHandler::load_model_with_arena()`. This avoids duplicating all the load/unload/resolver/interpreter logic.

```cpp
// model_handler.h — one new parameter (default nullptr = existing behavior)
bool load_model_with_arena(
    const uint8_t *model_data, size_t model_size,
    uint8_t *tensor_arena, size_t tensor_arena_size,
    const ModelConfig &config,
    tflite::MicroResourceVariables *mrv = nullptr);
```

When `mrv` is non-null (audio/streaming mode):
- Interpreter is constructed with 5-arg `MicroInterpreter(model, resolver, arena, size, mrv)`
- `MicroResourceVariables` enables persistent state via `ReadVariable`/`AssignVariable` ops
- The variable arena (1024 bytes) and `MicroResourceVariables` are managed by the **consumer component** (e.g., a future wake-word component), not by a `StreamingModel` wrapper

When `mrv` is null (default, existing image models):
- Interpreter constructed with 4-arg `MicroInterpreter(model, resolver, arena, size)` — no change in behavior

**Strided input + sliding window logic** — these are the only truly streaming-specific features. They live in the **consumer component** that owns the `TFLiteMicroHelper` instance:

```cpp
// Consumer component (e.g., wake_word_detector) — NOT in tflite_micro_helper
class WakeWordDetector : public Component {
  TFLiteMicroHelper tflite_;
  AudioModelConfig audio_config_;

  // Streaming state
  uint8_t var_arena_[1024];
  MicroResourceVariables *mrv_;
  uint8_t current_stride_step_;
  std::vector<uint8_t> sliding_window_probs_;

  bool feed_feature(const int8_t *features) {
    // 1. Strided memmove into input tensor
    // 2. Conditional Invoke when stride full
    // 3. Store output in sliding window
    // 4. Check average vs cutoff
  }
};
```

**Benefits:**
- Zero duplication of ModelHandler's load/unload/resolver code
- No new .h/.cpp files needed in `tflite_micro_helper`
- The `use_tflite_streaming` build flag is still available for consumer components
- Consumer components own their own variable arena + MRV lifecycle
- Existing image models completely unaffected (nullptr default)

### `TFLiteMicroHelper` API (Mode-Aware)

```cpp
class TFLiteMicroHelper {
 public:
  // === Common ===
  void set_model_type(const std::string &t);  // "image" | "audio"
  void set_model(const uint8_t *data, size_t size);
  void set_tensor_arena_size(size_t size);
  void set_output_processing(const std::string &p);
  void set_scale_factor(float f);
  void set_debug(bool d);

  bool load_model();
  void unload_model();
  bool is_model_loaded() const;

  // === Image mode ===
  void set_input_type(const std::string &t);
  void set_input_channels(int c);
  void set_input_width(int w);
  void set_input_height(int h);
  void set_input_order(const std::string &o);
  void set_normalize(bool n);
  void set_invert(bool i);

  bool run_inference_on_buffer(const uint8_t *src, size_t size);
  ProcessedOutput run_inference(const uint8_t *src, size_t size);
  ModelSpec get_model_spec() const;

  // === Audio / streaming mode ===
  void set_probability_cutoff(float f);    // 0.0-1.0
  void set_sliding_window_size(size_t n);
  void set_features_step_size(uint8_t ms);
  void set_feature_count(size_t n);

#ifdef USE_TFLITE_STREAMING
  bool perform_streaming_inference(const int8_t *features);
  DetectionEvent determine_detected();
  void reset_probabilities();
  bool get_unprocessed_probability_status() const;
#endif

  // === Memory ===
  size_t get_arena_used_bytes() const;
  size_t get_tensor_arena_size() const;
  ArenaStats get_arena_stats() const;
  void report_memory_status();

 private:
  std::string model_type_{"image"};

  // Common
  const uint8_t *model_data_{nullptr};
  size_t model_length_{0};
  size_t tensor_arena_size_requested_{100 * 1024};

  // Image path
  ModelHandler model_handler_;
  ImageModelConfig image_config_;

  // Audio path
#ifdef USE_TFLITE_STREAMING
  std::unique_ptr<StreamingModel> streaming_model_;
  AudioModelConfig audio_config_;
  tflite::MicroMutableOpResolver<MAX_OPERATORS> *shared_resolver_{nullptr};
#endif

  // Memory
  MemoryManager::AllocationResult tensor_arena_allocation_;
  std::atomic<bool> model_loaded_{false};
};
```

### Arena Probing (from `micro_wake_word/streaming_model.cpp`)

```cpp
// Ported to ModelHandler::probe_arena_size_()
// Logic:
//   1. Try manifest size (16-byte aligned)
//   2. If AllocateTensors fails → try 1.5x → try 2x
//   3. If succeeds at some size → binary search shrink to minimum
//   4. Return arena_used_bytes() + 16 (padded to 16-byte alignment)
```

This replaces the manual `1.5x bump` for ESP32-S3 cache and the user guessing `tensor_arena_size`.

---

## Python (`__init__.py`) Changes

### `tflite_micro_helper/__init__.py` — New Schema

```python
CONF_MODEL_TYPE = "model_type"
CONF_MODEL = "model"
CONF_MODEL_SOURCE = "model_source"  # local, github, http, git
CONF_PROBABILITY_CUTOFF = "probability_cutoff"
CONF_SLIDING_WINDOW_SIZE = "sliding_window_size"
CONF_FEATURES_STEP_SIZE = "features_step_size"
CONF_FEATURE_COUNT = "feature_count"
# ... image config keys preserved from current code ...

MODEL_SOURCE_SCHEMA = cv.Any(
    # Shorthand: model name or local path or URL
    _validate_source_shorthand,
    # Fully qualified
    cv.typed_schema({
        "local": LOCAL_SCHEMA,
        "git": GIT_SCHEMA,
        "http": HTTP_SCHEMA,
    })
)

PER_MODEL_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(TFLiteMicroHelper),
    cv.Required(CONF_MODEL_TYPE): cv.enum({"image", "audio"}),
    cv.Required(CONF_MODEL): MODEL_SOURCE_SCHEMA,
    cv.GenerateID(CONF_RAW_DATA_ID): cv.declare_id(cg.uint8),
    cv.Optional(CONF_TENSOR_ARENA_SIZE): ...,  # optional override
    # Audio config
    cv.Optional(CONF_PROBABILITY_CUTOFF): cv.percentage,
    cv.Optional(CONF_SLIDING_WINDOW_SIZE): cv.positive_int,
    cv.Optional(CONF_FEATURES_STEP_SIZE): cv.int_range(0, 30),
    cv.Optional(CONF_FEATURE_COUNT): cv.int_range(10, 80),
    # Image overrides (optional, usually auto-detected)
    cv.Optional(CONF_INPUT_TYPE): ...,
    # ...
})

CONFIG_SCHEMA = cv.Schema({
    cv.Optional(DOMAIN): cv.ensure_list(PER_MODEL_SCHEMA),
})
```

### `to_code()` — Per Model Entry

```python
async def to_code(config):
    for model_entry in config.get(DOMAIN, []):
        var = cg.new_Pvariable(model_entry[CONF_ID])
        model_type = model_entry[CONF_MODEL_TYPE]

        # 1. Resolve model source → get .tflite data + config
        model_data, model_len, model_config = resolve_model(model_entry)

        # 2. Create PROGMEM array
        rhs = [HexInt(x) for x in model_data]
        prog_arr = cg.progmem_array(model_entry[CONF_RAW_DATA_ID], rhs)
        cg.add(var.set_model(prog_arr, model_len))

        # 3. CRC32
        crc32_val = zlib.crc32(model_data) & 0xFFFFFFFF
        cg.add_define("MODEL_CRC32", HexInt(crc32_val))

        # 4. Common IDF components & build flags
        esp32.add_idf_component("espressif/esp-tflite-micro", ref="1.3.7")
        esp32.add_idf_component("espressif/esp-nn", ref="1.2.3")
        cg.add_build_flag("-DTF_LITE_STATIC_MEMORY")
        cg.add_build_flag("-DTF_LITE_DISABLE_X86_NEON")
        cg.add_build_flag("-DESP_NN")
        cg.add_build_flag("-DOPTIMIZED_KERNEL=esp_nn")

        # 5. Model-type-specific setup
        if model_type == "image":
            cg.add(var.set_model_type("image"))
            # Set image config (auto-detected or YAML-overridden)
            apply_image_config(var, model_entry, model_config)
            # MAX_OPERATORS
            max_ops = model_config.get("max_operators", 30)
            cg.add_build_flag(f"-DMAX_OPERATORS={max_ops}")

        elif model_type == "audio":
            cg.add(var.set_model_type("audio"))
            cg.add_define("USE_TFLITE_STREAMING")
            esp32.add_idf_component("esphome/esp-micro-speech-features", ref="1.2.3")
            # Set audio config
            apply_audio_config(var, model_entry, model_config)
```

### `resolve_model()` — Source Resolution

```python
def resolve_model(model_entry):
    """Resolves model source → (data, length, config_dict)."""
    model_spec = model_entry[CONF_MODEL]

    if isinstance(model_spec, str):
        # Shorthand resolution
        if is_model_name(model_spec):
            # "okay_nabu" → github://esphome/micro-wake-word-models/.../okay_nabu.json
            return resolve_github_model(model_spec)
        elif os.path.exists(model_spec):
            return load_local_model(model_spec)
        elif model_spec.startswith(("http://", "https://")):
            return download_http_model(model_spec)
        elif model_spec.startswith("github://"):
            return resolve_github_shorthand(model_spec)
        else:
            raise cv.Invalid(f"Cannot resolve model source: {model_spec}")

    elif isinstance(model_spec, dict):
        source_type = model_spec.get(CONF_TYPE, "local")
        if source_type == "local":
            return load_local_model(model_spec[CONF_PATH])
        elif source_type == "git":
            return resolve_git_model(model_spec)
        elif source_type == "http":
            return download_http_model(model_spec[CONF_URL])
```

---

## Consumer Migration: `meter_reader_tflite`

### Before (current YAML)

```yaml
meter_reader_tflite:
  id: ${id_prefix}_meter_reader
  model: "digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
  camera_id: camera_1
  tensor_arena_size: "110KB"
  input_width: 32
  input_height: 20
  crop_zones: my_zones
```

### After (new YAML)

```yaml
tflite_micro_helper:
  - id: my_digit_model
    model_type: image
    model: "digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
    # All image config auto-detected from digit_recognizer_v40_...txt
    # tensor_arena_size: "110KB"  ← optional override

meter_reader_tflite:
  id: ${id_prefix}_meter_reader
  model: my_digit_model          # ← ID reference, not file path
  camera_id: camera_1
  crop_zones: my_zones
  # input_width/height/channels no longer needed here!
```

### `meter_reader_tflite/__init__.py` — Removals

| Removed | Reason |
|---------|--------|
| `parse_model_txt_file()` | Moved to `tflite_micro_helper/__init__.py` |
| `infer_model_config_from_filename()` | Moved to `tflite_micro_helper/__init__.py` |
| `datasize_to_bytes()` | Moved to `tflite_micro_helper/__init__.py` |
| `open(model_path, "rb").read()` | Moved to `tflite_micro_helper/__init__.py` |
| `zlib.crc32(model_data)` | Moved to `tflite_micro_helper/__init__.py` |
| `progmem_array(config[CONF_RAW_DATA_ID], rhs)` | Moved to `tflite_micro_helper/__init__.py` |
| `cg.add_build_flag(f"-DMAX_OPERATORS={...})` | Moved to `tflite_micro_helper/__init__.py` |
| `cg.add(var.set_input_type/channels/width/height/...)` | Config comes from `tflite_micro_helper` model now |
| `CONF_MODEL` schema (was `cv.file_`) | Changed to `cv.use_id(TFLiteMicroHelper)` |

### `meter_reader_tflite/meter_reader_tflite.h` — Changes

```cpp
// OLD:
void set_model(const uint8_t *model, size_t length);

// NEW:
void set_model(tflite_micro_helper::TFLiteMicroHelper *tflite);
```

### `meter_reader_tflite/meter_reader_tflite.cpp` — Changes

```cpp
// OLD: TFLiteCoordinator owns TFLiteMicroHelper, loads from raw data
this->tflite_coord_.set_model(model, length);
this->tflite_coord_.load_model();

// NEW: TFLiteCoordinator holds a pointer to an already-loaded TFLiteMicroHelper
void MeterReaderTFLite::set_model(tflite_micro_helper::TFLiteMicroHelper *tflite) {
  this->tflite_coord_.set_tflite(tflite);
}
```

### Backward Compatibility

If `meter_reader_tflite` receives `model: "file.tflite"` (a string path instead of an ID):

1. Auto-generate a hidden `tflite_micro_helper` config entry
2. Configure it with `model_type: image`
3. Internally wire the reference
4. Emit deprecation warning: `"model: file.tflite" is deprecated. Move model to tflite_micro_helper section.`

---

## Example YAMLs

### Image Model (Meter Reading)

```yaml
tflite_micro_helper:
  - id: digit_model
    model_type: image
    model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"

meter_reader_tflite:
  id: cold_water_reader
  model: digit_model
  camera_id: esp32_camera
  crop_zones: cold_water_zones
  confidence_threshold: 0.85
```

### Audio Model (Wake Word)

```yaml
tflite_micro_helper:
  - id: wake_word_model
    model_type: audio
    model: okay_nabu                    # shorthand → github://.../okay_nabu.json
    probability_cutoff: 0.7
    sliding_window_size: 10

# Future consumer component:
# wake_word_detector:
#   model: wake_word_model
#   microphone: i2s_mic
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

meter_reader_tflite:
  - id: cold_reader
    model: cold_model
    camera_id: cam_cold
  - id: hot_reader
    model: hot_model
    camera_id: cam_hot
```

### Remote Model (GitHub)

```yaml
tflite_micro_helper:
  - id: vad_model
    model_type: audio
    model:
      type: git
      url: https://github.com/esphome/micro-wake-word-models
      ref: main
      file: models/v2/vad.json
```

---

## Backward Compatibility

### Strategy

1. **One release cycle** with deprecation warnings
2. `meter_reader_tflite` with `model: "file.tflite"` (string path) continues to work
3. Internally creates a hidden `tflite_micro_helper` instance
4. Logs deprecation warning to console and ESPHome dashboard

### Deprecation Warning Format

```
WARNING: Deprecated model configuration in meter_reader_tflite.
  The 'model: "file.tflite"' syntax is deprecated.
  Please move model configuration to the tflite_micro_helper section:

  tflite_micro_helper:
    - id: my_model
      model_type: image
      model: "file.tflite"

  meter_reader_tflite:
    model: my_model
```

---

## Quality Gates (Post-Change Verification)

**EVERY change MUST pass these checks before being considered complete.**
These are enforced by `.pre-commit-config.yaml`, CI (`ci-custom.py`), and `.clinerules/CLINE_RULES.md`.

### Per-Change Checklist

| # | Check | Tool / Command | Scope |
|---|-------|---------------|-------|
| 1 | **C++ formatting** | `clang-format --dry-run -Werror <file>` | All `.h`/`.cpp` changes |
| 2 | **C++ static analysis** | `clang-tidy <file> -- -std=gnu++20` | All `.h`/`.cpp` changes |
| 3 | **C++ style compliance** | `.ai/instructions.md` §7: member access (`this->`), naming (§7.5), `constexpr` over `#define` (§7.4), `static_cast<>` over C-casts (§10) | All `.h`/`.cpp` |
| 4 | **Python linting** | `ruff check . && ruff format --check .` | All `.py` changes |
| 5 | **Python docstrings** | `flake8 .` (tools & script dirs) | New `.py` functions in `tools/`/`script/` |
| 6 | **YAML linting** | `yamllint .` | All `.yaml` changes |
| 7 | **Custom lint checks** | `python script/ci-custom.py` | All files (LF, ASCII-only, no tabs, no trailing WS, EOF newline) |
| 8 | **File encoding** | LF-only, UTF-8, ASCII-only in source (§7.7) | All files |
| 9 | **Pre-commit all** | `pre-commit run --all-files` | Everything |
| 10 | **Cross-component grep** | `grep -rn <removed_symbol> components/` (§CLINE_RULES) | Any removed symbol/function/constant |
| 11 | **Build tests** | `./script/test_build_components.py -c meter_reader_tflite -t esp32` and `-t esp32-s3` (§9.1) | After consumer migration |
| 12 | **defines.h mirror** | `esphome/core/defines.h` MUST mirror ALL `cg.add_define()` calls (§7.4) | Every new `cg.add_define()` |
| 13 | **Config validation** | All YAML values bounds-checked with `cv.int_range`/`cv.float_range`/`cv.one_of` (§12.8) | Every new schema field |

### TFLite-Specific Checks (from §3.2)

| # | Check | Detail |
|---|-------|--------|
| T1 | `USE_TFLITE_MICRO_HELPER` guard | All `.h`/`.cpp` files wrapped in `#ifdef USE_TFLITE_MICRO_HELPER` |
| T2 | `USE_TFLITE_STREAMING` guard | New streaming code wrapped in `#ifdef USE_TFLITE_STREAMING` |
| T3 | RAII arena lifetime | `heap_caps_aligned_alloc` → `unique_ptr` with `HeapCapsDeleter` (§3.2) |
| T4 | No stack-allocated arena | Arena must be heap-allocated (stack overflow risk with 50KB+) |
| T5 | Model dimensions from runtime | Never hardcode `input_width`/`input_height` — derive from `TfLiteTensor::dims` |
| T6 | `process_output()` overloads | BOTH `const float*` and `TfLiteTensor*` overloads present |
| T7 | Debug gated | All debug logging guarded by `#ifdef DEBUG_TFLITE_MICRO_HELPER` |
| T8 | No C-style casts | `static_cast<>` only (§10) |
| T9 | No `new`/`delete` | RAII with `unique_ptr`/`make_unique` (§5.2) |
| T10 | No ARDUINO-only APIs | No `Serial`, `delay`, `digitalWrite` in shared code (§3.1) |

### Clinerules Failure Modes (from `.clinerules/CLINE_RULES.md`)

| # | Prevention Rule |
|---|----------------|
| C1 | Before removing any symbol: `grep -rn <symbol> components/` across ALL files |
| C2 | Never use bare `millis()` outside `esphome` namespace — use `esphome::millis()` |
| C3 | Never generate temp Python scripts for <10 files — use `replace_in_file` / `write_to_file` |
| C4 | Always `read_file` before `replace_in_file` to confirm current content |
| C5 | Glob patterns for cleanup: ONLY `_fix_*.py` / `temp_*.py` — never `camera_*` or project patterns |

### Quick Validation Script (per phase)

```bash
# After each implementation phase, run:
clang-format --dry-run -Werror components/tflite_micro_helper/*.cpp components/tflite_micro_helper/*.h
ruff check components/tflite_micro_helper/__init__.py
ruff format --check components/tflite_micro_helper/__init__.py
python script/ci-custom.py
pre-commit run --all-files
```

---

## Implementation Order

| Phase | Step | Description | Risk | Dependencies | Post-Check |
|-------|------|-------------|------|--------------|------------|
| **1** | 1.1 | Split `ModelConfig` in `model_handler.h` → `ModelConfig` + `ImageModelConfig` + `AudioModelConfig` | Low | None | clang-format, clang-tidy, T1-T9 |
| **1** | 1.2 | Arena probing in `model_handler.cpp` (ported from `micro_wake_word/streaming_model.cpp`) | Low | 1.1 | clang-format, clang-tidy, T1-T9 |
| **1** | 1.3 | Create `StreamingModel` class (`streaming_model.h/.cpp`) | Medium | 1.1 | clang-format, clang-tidy, T1-T10 |
| **1** | 1.4 | Mode-aware `TFLiteMicroHelper`: audio setters, streaming dispatch, `model_type_` member | Medium | 1.1, 1.3 | clang-format, clang-tidy, T1-T10 |
| **1** | 1.5 | `tflite_micro_helper/__init__.py`: model source resolution (local/github/http/git) | High | 1.4 | ruff, yamllint, §12.8 config validation |
| **1** | 1.6 | `tflite_micro_helper/__init__.py`: `.txt` parser ported from `meter_reader_tflite` | High | 1.5 | ruff, yamllint, C1 (cross-component grep) |
| **1** | 1.7 | `tflite_micro_helper/__init__.py`: `model_type` schema + per-type codegen | High | 1.5, 1.6 | ruff, yamllint, §12.8, defines.h mirror |
| **2** | 2.1 | `meter_reader_tflite/__init__.py`: change `CONF_MODEL` to `cv.use_id(TFLiteMicroHelper)` | Medium | 1.7 | ruff, yamllint, C1 |
| **2** | 2.2 | `meter_reader_tflite/__init__.py`: remove model loading, .txt parsing, PROGMEM code | Medium | 2.1 | ruff, C1 (grep removed symbols) |
| **2** | 2.3 | `meter_reader_tflite/tflite_coordinator`: hold `TFLiteMicroHelper*` instead of owning | Low | 2.1 | clang-format, clang-tidy |
| **2** | 2.4 | `meter_reader_tflite/meter_reader_tflite`: update `set_model()` signature | Low | 2.3 | clang-format, clang-tidy, C1 |
| **3** | 3.1 | Update all board `.yaml` files to new format | Low | 2.4 | yamllint |
| **3** | 3.2 | Implement backward compat fallback in `meter_reader_tflite/__init__.py` | Medium | 2.1 | ruff, yamllint |
| **4** | 4.1 | Update `docs/tflite_micro_helper.rst` | Low | 3.1 | ci-custom (ASCII, LF, etc.) |
| **4** | 4.2 | Update `docs/meter_reader_tflite.rst` | Low | 3.1 | ci-custom |
| **4** | 4.3 | Test with existing test suite (`tests/components/meter_reader_tflite/`) | High | 3.2 | Build both ESP32 + ESP32-S3 (§9.1) |
| **4** | 4.4 | Final `pre-commit run --all-files` | High | 4.3 | ALL 13 checks above + T1-T10 |
---

## File Change Summary

### New Files

| File | Purpose |
|------|---------|
| `components/tflite_micro_helper/streaming_model.h` | `StreamingModel` class declaration |
| `components/tflite_micro_helper/streaming_model.cpp` | `StreamingModel` implementation (ported from micro_wake_word) |

### Modified Files

| File | Change |
|------|--------|
| `components/tflite_micro_helper/__init__.py` | Major: model source resolution, .txt parsing, `model_type` schema, remote fetching |
| `components/tflite_micro_helper/tflite_micro_helper.h` | Major: audio setters, `StreamingModel` member, mode dispatch |
| `components/tflite_micro_helper/tflite_micro_helper.cpp` | Major: mode-aware `load_model()`, streaming `run_inference()` |
| `components/tflite_micro_helper/model_handler.h` | Split `ModelConfig`, add arena probing declaration |
| `components/tflite_micro_helper/model_handler.cpp` | Arena probing implementation, 3D/4D tensor handling |
| `components/meter_reader_tflite/__init__.py` | Major: remove model loading, use ID refs |
| `components/meter_reader_tflite/meter_reader_tflite.h` | Minor: `set_model()` signature change |
| `components/meter_reader_tflite/meter_reader_tflite.cpp` | Minor: delegate model loading |
| `components/meter_reader_tflite/tflite_coordinator.h` | Minor: reference instead of ownership |
| `components/meter_reader_tflite/tflite_coordinator.cpp` | Minor: use referenced helper |
| `boards/*.yaml` | Mechanical: split `tflite_micro_helper:` from `meter_reader_tflite:` |

### Removed (moved) Code

| From | To |
|------|----|
| `meter_reader_tflite/__init__.py::parse_model_txt_file()` | `tflite_micro_helper/__init__.py` |
| `meter_reader_tflite/__init__.py::infer_model_config_from_filename()` | `tflite_micro_helper/__init__.py` |
| `meter_reader_tflite/__init__.py::datasize_to_bytes()` | `tflite_micro_helper/__init__.py` |
| `meter_reader_tflite/__init__.py` model loading code | `tflite_micro_helper/__init__.py` |

---

## Inspiration from `micro_wake_word`

| Feature | Source | Adapted How |
|---------|--------|-------------|
| Model source resolution (local/github/http/git) | `micro_wake_word/__init__.py` | Ported, generalized for image + audio |
| JSON manifest parsing | `micro_wake_word/__init__.py` | Extended for `.txt` fallback |
| Model name shorthand (`okay_nabu`) | `micro_wake_word/__init__.py` | Same resolution logic |
| `StreamingModel` with MRV | `streaming_model.h/.cpp` | Ported to `tflite_micro_helper/streaming_model.h/.cpp` |
| Strided input + sliding window | `streaming_model.cpp::perform_streaming_inference()` | Ported, same pattern |
| Arena size probing | `streaming_model.cpp::probe_arena_size_()` | Ported to `model_handler.cpp` (image+audio) |
| VAD model gating | `micro_wake_word.cpp` | Optional future feature (USE_TFLITE_VAD) |
| `esp-micro-speech-features` IDF component | `__init__.py::to_code()` | Conditionally added for audio models |
| per-model `WakeWordModel` class with enable/disable | `streaming_model.h` | Simplified: enable/disable at StreamingModel level |

### What's NOT Ported

| Feature | Why excluded |
|---------|-------------|
| Ring buffer + FreeRTOS inference task | Consumer-specific — future audio component provides this |
| Audio frontend (spectrogram generation) | Consumer-specific — `esp-micro-speech-features` integration is per-consumer |
| OTA state listener suspension | Consumer-specific pattern |
| Home Assistant voice assistant integration | Not relevant to this project's scope |
| `on_wake_word_detected` automation trigger | Consumer-specific — future audio component provides this |

---

*Document Version: 1.0 — Generated 2026-07-18*
