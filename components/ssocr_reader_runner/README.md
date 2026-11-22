# SSOCR Reader Runner (Alpha)

> [!WARNING]
> **ALPHA STAGE SOFTWARE**
> This component is experimental and its configuration may change. It replaces
> `ssocr_reader` (now `ssocr_reader_legacy`) for new setups.

`ssocr_reader_runner` reads a seven-segment display out of `esp32_camera`
frames. It is the local counterpart of the upstream `ssocr_reader` component
and builds on the upstream building blocks:

- **`camera_decoder`** ([PR #22767](https://github.com/esphome/esphome/pull/22767))
  decodes each JPEG frame once.
- **`seven_segment_ocr`** ([PR #22999](https://github.com/esphome/esphome/pull/22999))
  thresholds the crop, splits it into digit runs and reads the seven segments.

On top of that, the runner keeps the two integrations from the legacy
component:

- **`value_validator`**: a reading the validator rejects is not published, so
  the last good value stays.
- **`flash_light_controller`**: the runner joins the shared flash cycle, so it
  and `meter_reader_tflite` can read the same camera with a single flash.

## Configuration

```yaml
camera_decoder:
  id: ssocr_decoder
  output_type: GRAYSCALE

value_validator:
  id: my_validator

flash_light_controller:
  id: my_flash
  flash_light: my_light

ssocr_reader_runner:
  id: my_ssocr
  camera_id: my_camera
  decoder_id: ssocr_decoder
  validator: my_validator
  flash_light_controller: my_flash
  crop_x: 8
  crop_y: 16
  crop_w: 160
  crop_h: 40
  digit_count: 6
  value:
    name: "Meter reading"
  confidence:
    name: "OCR confidence"
```

A frame is published only when exactly `digit_count` digits were found, all of
them were recognised, and the validator accepted the reading. Otherwise the
frame is logged and the sensors keep their last state.

## Notes

- `digit_count` is limited to 7: the value sensor is a float, which represents
  integers exactly only up to 2^24.
- `crop_w * crop_h` is a code-generated static buffer. On ESPHome 2026.11 it
  lands in internal RAM; later versions place it in PSRAM (feature-detected).
- Keep the crop tight — a large crop costs RAM and can include neighbouring
  digits, which makes the run count disagree with `digit_count`.

## Requirements

- ESPHome 2026.11 or later.
- `camera_decoder` and `seven_segment_ocr` from their PR heads (see
  `tests/components/ssocr_reader_runner/common.yaml` for the
  `external_components` entries).
