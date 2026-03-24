# ESP32-CAM Water Meter - Working Configuration Summary

**Date:** 2026-03-22  
**Status:** ✅ WORKING - 97.4% confidence, correct readings

---

## Problem Summary

The AI meter reader was giving zero/low confidence readings because:

1. **Wrong model config being used** - The model `dig-class100-0168_s2_q.tflite` was being loaded but had no config entry, causing auto-detection (which gave wrong processing parameters)
2. **Using remote components** - Components were pulled from GitHub instead of local, so fixes couldn't be applied
3. **Wrong model file** - Had switched to 0180 model which didn't work with the crop zones

---

## Solution Applied

### 1. Switch to Local Components

Changed from remote GitHub components to local components:

```yaml
external_components:
  - source:
      type: local
      path: components
    components:
      - tflite_micro_helper
      - esp32_camera_utils
      - flash_light_controller
      - meter_reader_tflite
```

This allows editing component source code directly.

### 2. Add Model Config Entry

Added config for `dig-class100-0168_s2_q` in `components/meter_reader_tflite/model_config.h`:

```cpp
{"dig-class100-0168_s2_q", 
    ModelConfig{
        .description = "dig-class100-0168",
        .tensor_arena_size = "512KB",
        .output_processing = "softmax_jomjol",
        .scale_factor = 10.0f,
        .input_type = "float32",  
        .input_channels = 3,
        .input_order = "RGB",
        .input_size = {32, 20}, 
        .normalize = false
    }
}
```

### 3. Use Correct Model

Switched back to the working model:

```yaml
meter_reader_tflite:
  model: "dig-class100-0168_s2_q.tflite"
```

### 4. Disable Preview (Memory Issue)

Turned off preview generation to prevent memory corruption:

```yaml
meter_reader_tflite:
  generate_preview: false
```

---

## Working Configuration

### Current Crop Zones (8-digit)
```yaml
initial_value: '"[[151,186,191,253],[188,186,230,252],[228,186,270,252],[266,186,308,252],[305,185,345,252],[347,188,367,223],[366,188,388,223],[385,188,410,223]]"'
```

### Model Settings
- **Model:** `dig-class100-0168_s2_q.tflite` (222KB)
- **Arena Size:** 512KB
- **Framework:** ESP-IDF

### Results
- **Reading:** 00993.330 (matches actual meter)
- **Confidence:** 97.4%
- **Status:** VALID

---

## Model Comparison

| Model | Size | Flash Usage | Config | Result |
|-------|------|-------------|--------|--------|
| dig-class100-0168_s2_q | 222K | ~68% | ✅ Added manually | ✅ **WORKING** |
| dig-class100-0173-s2-q | 298K | ~80% | ✅ Built-in | ❌ Wrong readings |
| dig-class100-0180-s2-q | 221K | ~68% | ✅ Built-in | ❌ Wrong readings |

---

## Key Learnings

1. **Model config matters** - Without proper config, the model uses auto-detect which can give wrong processing parameters
2. **Use local components for development** - Remote components can't be edited without pushing to Git
3. **0168 model works with these crop zones** - Other models may give different results even with same zones
4. **Preview generation causes memory issues** - Disabled to prevent allocation failures

---

## Files Modified

1. `test-ai-reader.yaml` - Switched to local components, disabled preview
2. `components/meter_reader_tflite/model_config.h` - Added 0168 model config

---

## Next Steps

- Monitor for stability over time
- Consider tuning last 3 digit zones if needed
- Flash usage is at ~68% with this configuration
