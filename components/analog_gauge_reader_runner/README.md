# Analog Gauge Reader Runner (Alpha)

> [!WARNING]
> **ALPHA STAGE SOFTWARE**
> This component is experimental and its configuration may change. It replaces
> `analog_gauge_reader` (now `analog_gauge_reader_legacy`) for new setups.

`analog_gauge_reader_runner` reads analog dials out of `esp32_camera` frames.
It is the local counterpart of the upstream `analog_gauge_reader` component and
builds on the upstream building blocks:

- **`camera_decoder`** ([PR #22767](https://github.com/esphome/esphome/pull/22767))
  decodes each JPEG frame once into a shared buffer.
- **`gauge_needle_detector`** ([PR #22985](https://github.com/esphome/esphome/pull/22985))
  finds the needle angle in a grayscale crop with the `legacy`,
  `radial_profile`, `hough_transform`, `template_match` and `auto` algorithms.

On top of that, the runner keeps the two integrations from the legacy
component:

- **`value_validator`**: the summed dial value is fed to the validator as a
  dial fraction, so it can correct digit-boundary transitions in
  `meter_reader_tflite`.
- **`flash_light_controller`**: the runner joins the shared flash cycle, so it
  and `meter_reader_tflite` can read the same camera with a single flash.

## Configuration

```yaml
camera_decoder:
  id: gauge_decoder
  output_type: GRAYSCALE   # RGB888 is required for colour dials

value_validator:
  id: my_validator

flash_light_controller:
  id: my_flash
  flash_light: my_light

analog_gauge_reader_runner:
  id: my_gauge
  camera_id: my_camera
  decoder_id: gauge_decoder
  validator: my_validator
  flash_light_controller: my_flash
  update_interval: 60s
  value_sensor:
    name: "Water meter"
  dials:
    - id: dial_1
      crop_x: 0
      crop_y: 0
      crop_w: 64
      crop_h: 64
      scale: 0.1
      min_value: 0
      max_value: 10
      min_angle: 0
      max_angle: 360
      value:
        name: "Dial 1"
      angle:
        name: "Dial 1 angle"
      confidence:
        name: "Dial 1 confidence"
```

The dial options match the upstream `analog_gauge_reader`: crop, needle type,
algorithm, scan window, deadzone, angle mapping, value range, calibration and
an optional colour target. `stacked_digits` combines dials as odometer digits.

## Detector size

The runner compiles in **all** needle algorithms, because the runtime algorithm
select in the controls package can switch any dial to any algorithm. If you do
not use runtime switching, the upstream `analog_gauge_reader` compiles only the
algorithms your configuration names.

## Requirements

- ESPHome 2026.11 or later.
- The `camera_decoder` and `gauge_needle_detector` components from their PR
  heads (see `tests/components/analog_gauge_reader_runner/common.yaml` for the
  `external_components` entries).
