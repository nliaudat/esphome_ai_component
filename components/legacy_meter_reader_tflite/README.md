# Legacy Meter Reader TFLite

This component preserves the original, monolithic implementation of the meter reader. It is kept in the repository for backward compatibility and as a reference.

**⚠️ Status: Deprecated**

This component is no longer actively developed. New features (like image rotation, modular camera utilities) are only available in the new `meter_reader_tflite` component.

## ⚙️ Configuration

Usage is identical to the old implementation:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nliaudat/esphome_ai_component
      ref: main
    components: [legacy_meter_reader_tflite]

legacy_meter_reader_tflite:
  id: my_meter_reader
  model: "model.tflite"
  camera_id: my_camera
  # ... other options ...
```

## 🔄 Migration

We strongly recommend migrating to the new modular system:

1. Change `legacy_meter_reader_tflite` to `meter_reader_tflite` in `external_components`.
2. Add the dependency components (`esp32_camera_utils`, `tflite_micro_helper`, `flash_light_controller`).
3. Rename the configuration block from `legacy_meter_reader_tflite:` to `meter_reader_tflite:`.
