
## Todos by priorites : 

### add image rotation
- For Jpeg, it's very cpu and memory intensive as the image must be full decoded and then rotated
- For RGB888/ RGB565 and grayscale, it's simpler and do not need full decoding
waiting for https://github.com/esphome/esphome/pull/9496

### add grayscale support
- waiting for https://github.com/esphome/esphome/pull/9496
- performance inprovement estimated 30%
PR #9496: This PR indeed adds support for sensors that do not output JPEG natively (likely outputting raw formats like YUV or Grayscale), and handles software conversion where necessary. This is a prerequisite for correctly handling direct raw/grayscale streams from cameras in ESPHome without forcing a JPEG pipeline.
Current Flow (likely): Sensor -> JPEG (Encoded) -> Transfer -> JPEG Decode (Software) -> RGB/Gray -> Resize -> Model.
Grayscale Flow: Sensor -> Gray (Raw) -> Transfer -> Resize -> Model.
Savings: You completely eliminate the JPEG Decoding step, which is computationally expensive on the ESP32. You also reduce memory bandwidth if processing 1 byte/pixel vs 2 or 3.

### implement arbitrary rotation
- Support float rotation values (0-360 degrees)
- Nearest Neighbor interpolation for preview images
- Dynamic bounding box calculation

## done in order of make : 

### Model enhancement
- done by https://github.com/nliaudat/digit_recognizer

### add OV2640 zoom
- check [idf esp32camera](https://github.com/espressif/esp32-camera/blob/dfeaa71f0aa78e4ed0b82dd9a18aacee1d5a4ced/sensors/ov2640.c#L137)
- check  [jomjol_controlcamera](https://github.com/jomjol/AI-on-the-edge-device/blob/f3e3ce504e363f104ce5342548383eb892bef132/code/components/jomjol_controlcamera/ClassControllCamera.cpp#L594)

### make OV2640 zoom functionnal with non 4:3 images
- change image processor routine

### add error checking for final readings

### correcting validation logic
- Improve ValueValidator to handle confidence and historical data better

### debugging tflite input mismatch
- Fix input size mismatch error
- Restore detailed debug logging

### expose configuration parameters
- Debug logging for multiple components
- Camera window parameters (offset_x, offset_y, width, height)
- Image rotation settings
- Flash pre- and post- times
- Meter reader parameters (update_interval, allow_negative_rates, max_absolute_diff)

### fix update interval not respected
- Ensure update interval setting is respected and dynamically applied
- Fix `number` component interaction
- Ensure immediate effect on polling schedule
