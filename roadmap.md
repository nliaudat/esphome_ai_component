# Future Development Roadmap

This document outlines a comprehensive plan for evolving the `esphome_ai_component`.

## Phase 1: Robustness & Stability (High Priority)
**Goal**: Ensure the device is unbreakable in production environments.

### 1.1 Strategic Watchdog Integration
- **Objective**: Prevent "Task Watchdog Got Triggered" crashes.
- **Action Plan**:
    - Insert `esphome::App.feed_wdt()` checks within long-running loops (`ModelHandler::invoke`).
    - Implement a "Circuit Breaker" to cool down processing if it causes lags.

### 1.2 Graceful Degradation
- **Objective**: Avoid boot loops on component failure.
- **Action Plan**:
    - Wrap init logic in try-catch.
    - Set `binary_sensor.camera_status` to `Problem` on failure instead of rebooting.

## Phase 2: Quick Wins & Features
**Goal**: Implement high-value features that are low effort.

### 2.1 Dynamic ROI (Region of Interest)
- **Objective**: Adjust crop zones without recompiling.
- **Action Plan**:
    - **Method**: Expose user-modifiable variables (template numbers/globals) for crop coordinates.
    - **Implementation**: Link these variables to the `CropZoneHandler` to update zones at runtime.

## Phase 3: Performance Optimization
**Goal**: Increase framerate and efficiency where hardware allows.

### 3.1 Double Buffering (Pipeline Processing)
- **Objective**: Parallelize Capture and Inference.
- **Action Plan**:
    - **NOTE**: Must be an **optional** feature (e.g., `#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ` or manual config flag) as not all ESP32 variants (S2, C3, S3-mini) have dual cores.
    - **Strategy**: 
        - If Dual Core: Run Capture on Core 0, Inference on Core 1.
        - If Single Core: Keep sequential processing.
    - Leverage ESP32-S3 dual cores capabilities when available.

### 3.2 Model Efficiency
- **Objective**: Ensure max TFLite performance.
- **Action Plan**:
    - **Verified**: Model `digit_recognizer_v4_10cls_GRAY.tflite` is strictly `INT8` quantized.
    - **Benchmark Analysis** (based on provided reports):
        - **V4 GRAY**: 61.4KB, 99.0% Acc, ~7617 inf/s. Balance of size/accuracy.
        - **V4 RGB**: 78.3KB, 99.2% Acc, ~8817 inf/s. Slightly higher accuracy/speed but larger size.
        - **V3 RGB**: 38.4KB, 98.3% Acc. Smallest footprint but lower accuracy.
    - **Recommendation**: 
        - Use **V4 GRAY** for best memory efficiency (25% smaller than RGB).
        - Use **V4 RGB** if ~1.5% accuracy gain is critical and RAM allows.
    - Investigate `ESP-DL` for optimized Conv2D layers.

### 3.3 Camera Pixel Format Optimization (Config Change)
- **Objective**: Eliminate JPEG decoding overhead (high CPU cost).
- **Analysis**:
    - **Current (JPEG)**: Camera (JPEG) -> RAM -> Decode (CPU) -> RGB -> Resize -> Inference. High CPU Load.
    - **Optimal (GRAYSCALE)**: Camera (Gray) -> RAM -> Resize -> Inference. Zero Decode.
        - **Memory**: VGA (640x480) Grayscale takes ~300KB RAM (Fits easily in S3 PSRAM).
        - **Speed**: Removes ~50-80ms of JPEG decoding time per frame.
- **Recommendation**:
    - **For V4 GRAY Model**: Set `camera_pixel_format: GRAYSCALE` in `config.yaml`.
    - **For V4 RGB Model**: Set `camera_pixel_format: RGB565` (or `RGB888` if supported) to avoid decode.
    - **Note**: This makes the Web UI preview Grayscale/Raw, but significantly boosts AI FPS.

### 3.3 Camera Pixel Format Optimization (Config Change)
- **Objective**: Eliminate JPEG decoding overhead (high CPU cost).
- **Analysis**:
    - **Current (JPEG)**: Camera (JPEG) -> RAM -> Decode (CPU) -> RGB -> Resize -> Inference. High CPU Load.
    - **Optimal (GRAYSCALE)**: Camera (Gray) -> RAM -> Resize -> Inference. Zero Decode.
        - **Memory**: VGA (640x480) Grayscale takes ~300KB RAM (Fits easily in S3 PSRAM).
        - **Speed**: Removes ~50-80ms of JPEG decoding time per frame using raw Y-channel data directly from the camera hardware.
- **Recommendation**:
    - **For V4 GRAY Model**: Set `camera_pixel_format: GRAYSCALE` in `config.yaml`.
    - **For V4 RGB Model**: Set `camera_pixel_format: RGB565` (or `RGB888` if supported) to avoid decode.
    - **Note**: This makes the Web UI preview Grayscale/Raw, but significantly boosts AI FPS.

## Phase 4: Developer Experience & CI
**Goal**: Automate testing and ensure code quality without hardware.

### 4.1 Host-Based Testing & CI
- **Objective**: Validate logic via GitHub Actions using the `host` platform.
- **Action Plan**:
    - **Configuration**: Leverage the existing `debug_image: true` setting to mock the camera Input.
    - **CI Pipeline**:
        - Create a GitHub Action workflow.
        - Build the project using the ESPHome `host` platform.
        - Run the binary to verify inference correctness on the `debug.jpg` static image.
    - **Benefit**: Catch regression errors in logic (preprocessing/postprocessing) automatically.

### 4.2 Strict Configuration Validation
- **Objective**: Provide earlier feedback on invalid YAML.
- **Action Plan**:
    - Add checks in `dump_config` for conflicting settings.

## Deprioritized / On Hold
- **Application-Level Model OTA**: Deemed unnecessary for the current stage.
