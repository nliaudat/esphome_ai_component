# ESPHome AI Component - AI Collaboration Guide

**Repository:** https://github.com/nliaudat/esphome_ai_component  
**Primary Goal:** Run TensorFlow Lite Micro models and computer vision algorithms on ESP32 devices within the ESPHome ecosystem  
**License:** 🚨 **CC-BY-NC-SA (NO COMMERCIAL USE)** - This is NON-NEGOTIABLE  
**Target Boards:** ESP32, ESP32-S2, ESP32-S3 (optimized), ESP32-C3, ESP8266 (limited)  
**ESPHome Version:** 2026.1.0+ (C++20, ESP-IDF 5.5.2)  

---

## 📋 TABLE OF CONTENTS

1.  [Project Overview](#1-project-overview)
2.  [Component Architecture](#2-component-architecture)
3.  [Component-Specific Rules](#3-component-specific-rules)
4.  [Performance Requirements](#4-performance-requirements)
5.  [Memory Management](#5-memory-management-critical)
6.  [Configuration Schema Patterns](#6-configuration-schema-patterns)
7.  [C++ Standards & Style](#7-c-standards--style-non-negotiable)
8.  [Security & Thread Safety](#8-security--thread-safety)
9.  [Testing Requirements](#9-testing-requirements)
10. [Common Anti-Patterns (REJECT)](#10-common-anti-patterns-reject)
11. [Review Priorities](#11-review-priorities)
12. [AI-Specific Instructions](#12-ai-specific-instructions)

---

## 1. PROJECT OVERVIEW

This project provides a modular framework for running **TensorFlow Lite Micro models** and **non-AI computer vision algorithms** on ESP32 devices within ESPHome.

**Key Characteristics:**

-   **Modular Design:** Split into focused, reusable components
-   **Dual Audience:** Both AI-powered (TFLite) and classical (SSOCR, analog) readers
-   **Performance-Critical:** ESP32-S3 target `<270ms`, legacy ESP32 target `<2700ms`
-   **Memory-Constrained:** Must operate with as little as 320KB free heap
-   **Active Learning:** Optional data collection for model improvement

**This file is the SINGLE SOURCE OF TRUTH for all code reviews.**  
Generic C++ advice is OVERRIDDEN by the rules below.

---

## 2. COMPONENT ARCHITECTURE

### 2.1 Component Hierarchy & Dependencies

```
esphome_ai_component/
├── meter_reader_tflite/          # AI PRIMARY - Digit recognition with TFLite
│   ├── DEPENDS ON: tflite_micro_helper, esp32_camera_utils
│   └── OPTIONAL: value_validator, flash_light_controller, data_collector
│
├── ssocr_reader/                 # NO AI - Seven-segment OCR (alpha)
│   ├── DEPENDS ON: esp32_camera_utils
│   └── OPTIONAL: value_validator
│
├── analog_reader/                # NO AI - Analog dial/gauge (alpha)
│   ├── DEPENDS ON: esp32_camera_utils
│   └── OPTIONAL: value_validator
│
├── value_validator/              # SHARED - Reading validation engine
│   ├── USED BY: meter_reader_tflite, ssocr_reader, analog_reader
│   └── DEPENDS ON: None (standalone)
│
├── esp32_camera_utils/           # SHARED - Image processing core
│   ├── USED BY: All reader components
│   └── DEPENDS ON: esp32_camera component
│
├── tflite_micro_helper/         # SHARED - TFLite runtime wrapper
│   ├── USED BY: meter_reader_tflite ONLY
│   └── DEPENDS ON: tensorflow/lite/micro (ESPHome component)
│
├── flash_light_controller/      # SHARED - Illumination timing
│   ├── USED BY: meter_reader_tflite
│   └── DEPENDS ON: None
│
├── data_collector/              # OPTIONAL - Active learning uploader
│   ├── USED BY: meter_reader_tflite
│   └── DEPENDS ON: HTTP client, optional API key auth
│
└── legacy_meter_reader_tflite/  # FROZEN - No new features, bug fixes only
    └── DEPRECATED: Use meter_reader_tflite instead
```

### 2.2 Component Lifecycle States

| State | Meaning | Action Required |
|-------|---------|-----------------|
| **Active** | Full support, new features allowed | Maintain tests, docs, performance |
| **Alpha** | Experimental, API may change | Must be opt-in, clearly documented |
| **Frozen** | No new features, critical bugs only | Do not refactor, do not extend |
| **Deprecated** | Will be removed in future | Document migration path |

**Current Status:**
- `meter_reader_tflite`: **Active** (v2.0+)
- `value_validator`: **Active**
- `esp32_camera_utils`: **Active**
- `tflite_micro_helper`: **Active**
- `flash_light_controller`: **Active**
- `data_collector`: **Active**
- `ssocr_reader`: **Alpha**
- `analog_reader`: **Alpha**
- `legacy_meter_reader_tflite`: **Frozen**

---

## 3. COMPONENT-SPECIFIC RULES

### 3.1 `meter_reader_tflite` - AI Digit Recognition

**✅ REQUIRED:**
- `tensor_arena_size` MUST be user-configurable via YAML
- Model CRC32 verification MUST run on every load
- Support BOTH RGB and GRAYSCALE input models
- Camera windowing MUST work with OV2640/OV3660 sensors
- Rotation setting MUST apply to inference preprocessing
- MUST expose confidence scores to Home Assistant

**⚠️ WARNING:**
- Rotation ONLY affects inference, not web stream (documented limitation)
- Default tensor arena size: 512KB (ESP32), 768KB (ESP32-S3)
- Large models (>1MB) may fail on classic ESP32

**❌ BLOCKER:**
- No hardcoded model input dimensions - MUST derive from model
- No synchronous HTTP in inference path
- No memory allocation during loop() after setup

### 3.2 `esp32_camera_utils` - Image Processing Core

**✅ REQUIRED:**
- MUST implement `TrackedBuffer` pattern for JPEG memory management
- MUST support both JPEG and RAW (RGB565/GRAYSCALE) formats
- Rotation MUST work correctly on both formats
- Crop/scaling operations MUST be bounds-checked
- Buffer lifetime MUST be clearly documented

**❌ BLOCKER:**
- NO JPEG decoding in interrupt context
- NO memory leaks in any code path
- NO direct `esp_camera_fb_get()` exposure - ALWAYS wrap in TrackedBuffer
- NO assumption that PSRAM is available

**TrackedBuffer Pattern:**
```cpp
class TrackedJpegBuffer {
 public:
  uint8_t* data;
  size_t len;
  ~TrackedJpegBuffer() { /* auto-release */ }
};

auto buffer = camera_utils_->get_jpeg_buffer();
if (buffer) {
  process(buffer->data, buffer->len);  // Auto-freed on exit
}
```

### 3.3 `value_validator` - Reading Validation

**✅ REQUIRED:**
- Configurable `max_absolute_diff` and `max_rate_change`
- Maintain reading history for outlier detection
- Expose validation state to Home Assistant entities
- Support string-based readings for digital displays

**⚠️ WARNING:**
- History size MUST be bounded (default: 10 readings)
- Rate limiting on invalid readings to prevent log spam

### 3.4 `data_collector` - Active Learning

**✅ REQUIRED:**
- HTTP upload MUST have timeout (default: 5 seconds)
- Configurable API key/authentication
- NON-BLOCKING operation - MUST use `defer()` or `set_timeout()`
- Rate limiting on uploads (default: max 1 per minute)

**❌ BLOCKER:**
- NO infinite retry loops
- NO blocking on network in main thread
- NO silent failures - log upload errors

### 3.5 `legacy_meter_reader_tflite` - Frozen Component

**❌ BLOCKER:**
- NO new features
- NO refactoring
- NO API changes
- ONLY critical security or crash bug fixes
- Users MUST be directed to migrate to `meter_reader_tflite`

---

## 4. PERFORMANCE REQUIREMENTS

### 4.1 Target Benchmarks

| Board | Full Pipeline (capture + inference) | Tensor Arena Limit | Flash Budget |
|-------|-------------------------------------|-------------------|--------------|
| **ESP32-S3** | **<270ms** ✅ (optimized model) | Up to 1MB | ~2MB |
| **ESP32** | **<2700ms** ⚠️ (legacy) | Max 512KB | ~1.5MB |
| **ESP32-C3** | **<500ms** ⚠️ | Max 512KB | ~1.5MB |
| **ESP8266** | **Not recommended** ❌ | N/A | N/A |

### 4.2 Optimization Rules

**✅ ALWAYS:**
- Use int8 quantized models (NOT float32)
- Enable ESP-NN accelerators (`-DESP_NN=1`)
- Calculate tensor_arena_size from model metadata
- Profile on BOTH ESP32 and ESP32-S3 before merging

**⚠️ AVOID:**
- Dynamic allocations in `loop()`
- Large stack allocations (>1KB)
- Virtual functions in hot paths
- Floating point in pixel loops

**❌ NEVER:**
- `std::regex` on ESP32 (ReDoS risk, massive binary size)
- Full JSON parsing in hot path
- Synchronous network during inference

**🚀 KEY OPTIMIZATIONS:**
- `StaticVector` and `FixedVector` replacing `std::vector` where sizes are known
- `const char*` for static strings instead of `std::string`
- Pre-allocated buffers instead of dynamic allocation in hot paths
- Stack allocation where possible
- Audio/video components using pre-allocated ring buffers

### 4.3 Profiling Requirement

**BEFORE merging any performance-sensitive PR:**
```bash
# ESP32-S3 (optimized target)
esphome compile config.yaml
# Monitor serial, measure end-to-end inference time
# Report: before (ms) -> after (ms) -> improvement (%)

# ESP32 (legacy target)
# Verify no regression >10%
```

---

## 5. MEMORY MANAGEMENT (CRITICAL)

### 5.1 ESP32-Camera Memory Model

```
Camera Framebuffer (PSRAM if available)
         ↓
JPEG Buffer (TrackedBuffer - heap or PSRAM)
         ↓
Decoding Buffer (heap, temporary)
         ↓
Preprocessed Image (tensor arena or heap)
```

**Rules:**
- Camera framebuffer is managed by `esp32_camera` component
- JPEG buffers MUST use `TrackedBuffer` pattern
- NEVER hold camera framebuffer longer than necessary
- Assume PSRAM may be absent or disabled

### 5.2 TFLite Micro Arena

**✅ REQUIRED:**
```cpp
// CORRECT: Configurable, RAII, sized from model
class MeterReaderTFLite : public Component {
 public:
  void setup() override {
    size_t arena_size = this->tensor_arena_size_;
    this->arena_ = std::make_unique<uint8_t[]>(arena_size);
    this->interpreter_ = tflite::GetInterpreter(..., this->arena_.get(), arena_size);
  }
 private:
  std::unique_ptr<uint8_t[]> arena_;
  size_t tensor_arena_size_{512 * 1024};  // Configurable default
};
```

**❌ BLOCKER:**
```cpp
// WRONG: Hardcoded, no RAII, no configuration
uint8_t tensor_arena[512 * 1024];  // Stack overflow risk!
// OR
uint8_t* arena = new uint8_t[512 * 1024];  // Manual delete required!
```

### 5.3 Heap Fragmentation Prevention

**⚠️ CRITICAL FOR LONG-RUNNING DEVICES:**

- **DO** allocate everything in `setup()`
- **DO** use `std::array` or `StaticVector` for fixed-size collections
- **DO NOT** `new`/`delete` repeatedly in `loop()`
- **DO NOT** use `std::string` for temporary paths in hot loops

---

## 6. CONFIGURATION SCHEMA PATTERNS

### 6.1 Standard Component Pattern

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = []  # List component dependencies
AUTO_LOAD = []      # Auto-load these components
MULTI_CONF = True   # Allow multiple instances? Usually False

CONF_MY_PARAM = "my_param"
CONF_CAMERA_ID = "camera_id"

my_ns = cg.esphome_ns.namespace("my_component")
MyComponent = my_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyComponent),
    cv.Optional(CONF_MY_PARAM, default=42): cv.int_,
    cv.Required(CONF_CAMERA_ID): cv.use_id(ESP32Camera),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_MY_PARAM in config:
        cg.add(var.set_my_param(config[CONF_MY_PARAM]))
    
    # Get dependency reference
    cam = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(cam))
```

### 6.2 Sensor Component Pattern

```python
from esphome.components import sensor

CONFIG_SCHEMA = sensor.sensor_schema(
    MySensor,
    accuracy_decimals=0,
    device_class="water",
    state_class="total_increasing",
    icon="mdi:water"
).extend({
    cv.Optional(CONF_PARAM, default=10): cv.int_,
}).extend(cv.polling_component_schema("60s"))

async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
```

### 6.3 Configuration Validation

**✅ GOOD:**
```python
CONF_THRESHOLD = "threshold"
cv.Optional(CONF_THRESHOLD, default=0.85): cv.percentage,  # 0.0-1.0
cv.Optional(CONF_DIGITS, default=8): cv.int_range(min=1, max=12),
cv.Optional(CONF_ROTATION, default="0"): cv.one_of("0", "90", "180", "270"),
```

**❌ BAD:**
```python
cv.Optional(CONF_THRESHOLD, default=85): cv.int_,  # 85 what? Percent? 0-100?
cv.Optional(CONF_DIGITS, default=8): cv.int_,     # No bounds! Could be 9999
```

### 6.4 Platform/Framework Conditions

```python
# ESP32 only feature
CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
    cv.Optional(CONF_PSRAM): cv.only_on_esp32,  # Only available on ESP32
})

# ESP-IDF only feature
CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
    cv.Optional(CONF_ESP_NN): cv.only_with_esp_idf,  # ESP-NN only in IDF
})
```

---

## 7. C++ STANDARDS & STYLE (NON-NEGOTIABLE)

### 7.1 Language Version

**✅ C++20 is ALLOWED and EXPECTED**

ESPHome compiles with `-std=gnu++20`. DO NOT flag C++20 features as "too modern".

**Allowed C++20 features:**
- `std::span` (preferred over pointer+length)
- `std::bit_cast` (type-punning without UB)
- `consteval` / `constinit`
- Designated initializers
- `[[likely]]` / `[[unlikely]]` attributes

### 7.2 Member Access - MANDATORY

**✅ REQUIRED: Prefix ALL class member access with `this->`**

```cpp
// CORRECT
void set_value(int value) {
  this->value_ = value;
  if (this->initialized_) {
    this->update();
  }
}

// WRONG - DO NOT DO THIS
void set_value(int value) {
  value_ = value;  // Missing this->
  if (initialized_) {
    update();      // Missing this->
  }
}
```

### 7.3 Field Visibility

**✅ PREFER `protected` (extensibility first)**

```cpp
class MyComponent : public Component {
 protected:
  int counter_{0};           // OK - derived classes can access
  std::string name_;         // OK - no invariant constraints
};
```

**⚠️ USE `private` ONLY for safety-critical cases:**

1. **Pointer lifetime safety** (validated pointers from known lists)
2. **Invariant coupling** (buffer size + pointer must stay synchronized)
3. **Resource management** (cleanup that derived classes might skip)

```cpp
class ClimateDevice {
 public:
  void set_custom_fan_modes(std::initializer_list<const char*> modes) {
    this->custom_fan_modes_ = modes;
    this->active_custom_fan_mode_ = nullptr;  // Reset on change
  }
  
  bool set_custom_fan_mode(const char* mode) {
    // Store pointer FROM the validated list, NOT the input pointer
    const char* validated = this->find_in_list(mode);
    if (validated) {
      this->active_custom_fan_mode_ = validated;  // Safe: points to list
      return true;
    }
    return false;
  }
  
 private:
  std::vector<const char*> custom_fan_modes_;  // String literals in flash
  const char* active_custom_fan_mode_{nullptr};  // MUST point to list entry
};
```

### 7.4 Preprocessor Directives

**❌ AVOID `#define` for constants**

```cpp
// WRONG
#define MAX_RETRIES 5
#define BUFFER_SIZE 1024

// CORRECT
static constexpr int MAX_RETRIES = 5;
static constexpr size_t BUFFER_SIZE = 1024;
enum : size_t { BUFFER_SIZE = 1024 };
```

**✅ ALLOW `#define` ONLY for:**
- Conditional compilation (`#ifdef`, `#ifndef`)
- Compile-time sizes from Python codegen (`cg.add_define()`)

**⚠️ REQUIRED:**
All `cg.add_define()` calls MUST be mirrored in `esphome/core/defines.h` for static analysis.

### 7.5 Naming Conventions

| Item | Convention | Example |
|------|------------|---------|
| Classes/Structs/Enums | `UpperCamelCase` | `MeterReader`, `PixelFormat` |
| Functions/Methods | `lower_snake_case` | `get_image()`, `validate_reading()` |
| Variables | `lower_snake_case` | `current_value`, `tensor_arena` |
| Top-level constants | `UPPER_SNAKE_CASE` | `MAX_TENSOR_ARENA` |
| Local constants | `lower_snake_case` | `const int max_retries = 5;` |
| Protected/private fields | `lower_snake_case_` (trailing underscore) | `value_`, `camera_` |

### 7.6 Formatting

- **Indentation:** 2 spaces, NO tabs
- **Line length:** Maximum 120 characters
- **Braces:** Google style (opening brace on same line)
- **Tools:** `clang-format` (.clang-format), `ruff` (Python)

---

## 8. SECURITY & THREAD SAFETY

### 8.1 Integer Overflow (CVE-2026-23833 Pattern)

**❌ DANGEROUS:**
```cpp
if (ptr + field_length > end) {  // Can overflow!
  return error;
}
```

**✅ SAFE:**
```cpp
// Check overflow first, then bounds
if (field_length > (end - ptr) || ptr + field_length > end) {
  return error;
}
```

**AUDIT EVERY INSTANCE of pointer arithmetic in bounds checks.**

### 8.2 Buffer Overflows

**❌ NEVER:**
- `strcpy()`, `strcat()`, `sprintf()` - use `snprintf()` or `std::string`
- `memcpy()` without bounds checking destination size
- Fixed-size stack arrays with untrusted input

**✅ ALWAYS:**
```cpp
// Use std::array or std::vector with bounds checks
std::array<char, 64> buffer;
snprintf(buffer.data(), buffer.size(), "%s", user_input);

// Or better: use std::string
std::string result = std::string(user_input);
```

### 8.3 Format String Vulnerabilities

**❌ NEVER:**
```cpp
ESP_LOGD("tag", user_input);  // Format string vulnerability!
```

**✅ ALWAYS:**
```cpp
ESP_LOGD("tag", "%s", user_input);  // Safe
```

### 8.4 Thread Safety (FreeRTOS)

**⚠️ ESPHome runs multiple tasks:**
- Main loop task (component updates)
- WiFi/network task
- Camera task
- Web server task

**✅ REQUIRED for shared state:**
```cpp
// Shared mutable state MUST be protected
std::mutex mutex_;
int shared_counter_;

void update_counter() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->shared_counter_++;
}

int get_counter() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->shared_counter_;
}
```

**❌ BLOCKER:**
- Global/static variables modified from multiple contexts without locks
- TOCTOU (Time-of-Check Time-of-Use) vulnerabilities
- Double-checked locking without proper barriers

### 8.5 TOCTOU (CWE-367)

**❌ DANGEROUS:**
```cpp
if (this->has_state()) {      // CHECK
  float val = this->state;    // USE - state may have changed!
}
```

**✅ SAFE:**
```cpp
float val = this->state;      // Single atomic operation
if (this->has_state(val)) {   // Validate the value, not the flag
}
```

---

## 9. TESTING REQUIREMENTS

### 9.1 Compilation Tests

**✅ EVERY component MUST have a YAML test:**
```
tests/
├── test_build_components/
│   └── test_meter_reader_tflite.yaml
├── components/
│   └── meter_reader_tflite/
│       ├── common.yaml
│       ├── esp32.yaml
│       └── esp32-s3.yaml
```

**Test requirements:**
- Test compiles for both ESP32 and ESP32-S3
- Test both minimal and full-feature configurations
- Test with and without optional dependencies

**Run tests:**
```bash
./script/test_build_components.py -c meter_reader_tflite -t esp32
./script/test_build_components.py -c meter_reader_tflite -t esp32-s3
```

### 9.2 Static Analysis

**✅ MUST pass:**
```bash
# C++ static analysis
clang-tidy components/meter_reader_tflite/*.cpp -- -std=gnu++20

# Python linting
ruff check esphome/components/meter_reader_tflite/
black --check esphome/components/meter_reader_tflite/
```

**⚠️ REQUIRED:**
All new `cg.add_define()` calls MUST be added to `esphome/core/defines.h`

### 9.3 Runtime Testing

**Memory:**
- Profile heap usage before/after changes
- Check for leaks with `esp_get_free_heap_size()`
- Monitor fragmentation over 24+ hours

**Performance:**
- Measure inference time on target hardware
- Report before/after numbers in PR
- Regression >10% requires justification

---

## 10. COMMON ANTI-PATTERNS (REJECT)

### ❌ Direct Camera Buffer Access

```cpp
// WRONG - Bypasses memory management, leak risk
framebuffer_t* fb = esp_camera_fb_get();
process(fb->buf, fb->len);
esp_camera_fb_return(fb);
```

✅ **USE:**
```cpp
// CORRECT - Uses TrackedBuffer, auto-release
auto buffer = this->camera_utils_->get_jpeg_buffer();
if (buffer) {
  process(buffer->data, buffer->len);
}  // Auto-freed
```

### ❌ Blocking Network in Main Thread

```cpp
// WRONG - Blocks loop() for seconds
void loop() override {
  http_client.POST(url, data);  // NO!
}
```

✅ **USE:**
```cpp
// CORRECT - Non-blocking
void loop() override {
  if (this->should_upload_) {
    this->defer([this]() {
      http_client.POST(this->url_, this->data_);
    });
    this->should_upload_ = false;
  }
}
```

### ❌ Manual Memory Management

```cpp
// WRONG - Manual new/delete, exception unsafe
uint8_t* arena = new uint8_t[512 * 1024];
// ...
delete[] arena;
```

✅ **USE:**
```cpp
// CORRECT - RAII, exception safe
auto arena = std::make_unique<uint8_t[]>(512 * 1024);
// Automatically freed
```

### ❌ C-Style Casts

```cpp
// WRONG - C-style cast, dangerous
int x = (int)float_value;
uint8_t* ptr = (uint8_t*)camera_buffer;
```

✅ **USE:**
```cpp
// CORRECT - Explicit, safe
int x = static_cast<int>(float_value);
auto* ptr = static_cast<uint8_t*>(camera_buffer);
```

### ❌ `#define` Constants

```cpp
// WRONG
#define TENSOR_ARENA_SIZE (512 * 1024)
#define MODEL_PATH "/models/digit.tflite"
```

✅ **USE:**
```cpp
// CORRECT
static constexpr size_t TENSOR_ARENA_SIZE = 512 * 1024;
static constexpr const char* MODEL_PATH = "/models/digit.tflite";
```

### ❌ Missing `this->`

```cpp
// WRONG - Unclear origin, breaks refactoring
void set_value(int val) {
  value_ = val;
  update();
}
```

✅ **USE:**
```cpp
// CORRECT - Explicit member access
void set_value(int val) {
  this->value_ = val;
  this->update();
}
```

### ❌ `std::regex` on ESP32

```cpp
// WRONG - Catastrophic backtracking, huge binary
std::regex re("^[0-9]+$");
if (std::regex_match(str, re)) { ... }
```

✅ **USE:**
```cpp
// CORRECT - Simple string operations
if (str.find_first_not_of("0123456789") == std::string::npos) { ... }
```

### ❌ Hardcoded Magic Numbers

```cpp
// WRONG - Why 512? Why 0.85?
if (confidence > 0.85) { ... }
uint8_t buffer[512];
```

✅ **USE:**
```cpp
// CORRECT - Named constants
static constexpr float CONFIDENCE_THRESHOLD = 0.85;
static constexpr size_t JPEG_BUFFER_SIZE = 512;
if (confidence > CONFIDENCE_THRESHOLD) { ... }
std::array<uint8_t, JPEG_BUFFER_SIZE> buffer;
```

---

## 11. REVIEW PRIORITIES

### 🔴 BLOCKER (Must fix, reject PR)

1.  **License violations** (CC-BY-NC-SA - no commercial use)
2.  **Memory leaks** (any unfreed allocation)
3.  **Buffer overflows** (unbounded memcpy, strcpy)
4.  **Integer overflow** in pointer arithmetic (CVE-2026-23833)
5.  **Camera buffer mismanagement** (bypassing TrackedBuffer)
6.  **Blocking operations** in main loop (`delay()`, sync HTTP)
7.  **C-style casts** (use `static_cast<>`)
8.  **`#define` constants** (use `constexpr`)
9.  **Missing `this->`** on member access
10. **TOCTOU race conditions** (check-then-act)

### 🟠 WARNING (Must fix, not necessarily reject)

1.  **Performance regression** (>10% slower on S3)
2.  **Missing configuration validation** (bounds not checked)
3.  **Heap allocation in `loop()`** (allocate in `setup()`)
4.  **Large stack allocations** (>1KB local arrays)
5.  **Missing debug logging** (silent failures)
6.  **Inefficient data structures** (std::set for n<10)
7.  **Unprotected shared state** (missing mutex)
8.  **Magic numbers** (should be named constants)

### 🟡 INFO (Recommend, non-blocking)

1.  **Const correctness** (missing `const` on read-only methods)
2.  **Naming convention violations**
3.  **Missing comments** for non-obvious logic
4.  **Dead code** (unused functions/variables)
5.  **Include-what-you-use** (unnecessary includes)

---

## 12. AI-SPECIFIC INSTRUCTIONS

**This section is DIRECT COMMANDS to AI assistants analyzing this repository.**

### 12.1 Mandatory Pre-Read

**BEFORE analyzing any code, you MUST read this entire document.**
**BEFORE making any suggestion, you MUST check if it violates any rule above.**

### 12.2 License Enforcement

**🚨 THIS PROJECT IS CC-BY-NC-SA - NO COMMERCIAL USE 🚨**

- **NEVER** suggest using this code in commercial products
- **NEVER** suggest incorporating code with incompatible licenses (MIT, Apache, GPL) without explicit review
- **ALWAYS** flag PRs that appear to enable commercial redistribution

### 12.3 C++ Version Awareness

- **C++20 IS ALLOWED AND EXPECTED**
- **DO NOT** suggest "upgrading to C++17" or "avoiding C++20"
- **DO NOT** flag `std::span`, `std::bit_cast`, `consteval` as "too modern"

### 12.4 Component-Specific Rules

**BEFORE reviewing a file, IDENTIFY which component it belongs to:**
- `meter_reader_tflite/` → Apply AI inference rules
- `esp32_camera_utils/` → Apply memory management rules
- `data_collector/` → Apply non-blocking network rules
- `legacy_meter_reader_tflite/` → **EXCLUDED** from AI review. Do not read, analyze, or suggest changes to this component. It is frozen and deprecated.

### 12.5 Performance Dual-Target Awareness

**ALWAYS consider BOTH targets:**
- ESP32-S3: Optimize for SPEED (<270ms target)
- ESP32: Optimize for MEMORY (512KB arena limit)

**NEVER suggest optimizations that work on S3 but break on classic ESP32 without explicit check.**

### 12.6 Memory Safety

**ALWAYS verify:**
- Camera buffers use `TrackedBuffer` pattern
- TFLite arena uses `std::unique_ptr` or `std::array`
- No `new` without matching `delete` (RAII instead)
- No `memcpy` without bounds checking

### 12.7 Anti-Pattern Detection

**Immediately flag:**
- C-style casts → Suggest `static_cast<>`
- `#define` constants → Suggest `constexpr`
- Missing `this->` → Suggest adding it
- `delay()` in loop → Suggest `set_timeout()`
- `std::regex` → Suggest string operations

### 12.8 Configuration Validation

**ALWAYS check:**
- Are user-provided values bounds-checked?
- Are there sensible defaults?
- Are errors reported clearly with `cv.Invalid`?

### 12.9 When Uncertain

**If you are unsure whether a suggestion follows these rules:**
1.  **STATE** the specific rule you are considering
2.  **ASK** the user to confirm before proceeding
3.  **REFERENCE** the section number from this document

### 12.10 No Unauthorized Commits

- **NEVER** commit, push, or create any files in the repository without explicit human acknowledgment and approval.
- Present changes as proposals first. Wait for the user to confirm before writing any files or making commits.
- This rule applies to ALL files including documentation, proposals, and code changes.

### 12.11 Communication Style Directive

- **DO NOT** add "open questions" sections in code comments.
- Keep review output focused on findings, fixes, and actionable recommendations only.

---

## 📌 SUMMARY: NON-NEGOTIABLE RULES (Cheat Sheet)

| Rule | Standard | Exception |
|------|----------|-----------|
| **C++ Standard** | C++20 (`gnu++20`) | None |
| **Member access** | `this->` prefix | None |
| **Field visibility** | `protected` first | `private` for safety-critical |
| **Constants** | `constexpr` / `enum` | `#define` only for conditional compilation |
| **Casts** | `static_cast<>` | Never use C-style casts |
| **Memory** | RAII (`unique_ptr`) | Never manual `new`/`delete` |
| **Camera buffers** | `TrackedBuffer` | Never direct `fb_get` |
| **Blocking** | `defer()` / `set_timeout()` | Never `delay()` in `loop()` |
| **Regex** | String operations | Never `std::regex` |
| **License** | CC-BY-NC-SA | Never commercial use |

---

**END OF AI COLLABORATION GUIDE**

*This document is the authoritative standard for the esphome_ai_component repository.*
*Last updated: February 2026*
```