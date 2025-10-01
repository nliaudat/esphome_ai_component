
## Todos by priorites : 


### add error checking for final readings
(like in AI-on-the-edge-device)
- Prevalue
- AllowNegativeRates
- MaxRateValue
- CheckDigitIncreaseConsistency

### add image rotation
- For Jpeg, it's very cpu and memory intensive as the image must be full decoded and then rotated
- For RGB888/ RGB565 and grayscale, it's simpler and do not need full decoding

### Model enhancement
- [Yolo26](https://docs.ultralytics.com/fr/models/yolo26/)
- [esp-dl](https://github.com/espressif/esp-dl)


## done : 

### add OV2640 zoom
- check [idf esp32camera](https://github.com/espressif/esp32-camera/blob/dfeaa71f0aa78e4ed0b82dd9a18aacee1d5a4ced/sensors/ov2640.c#L137)
- check  [jomjol_controlcamera](https://github.com/jomjol/AI-on-the-edge-device/blob/f3e3ce504e363f104ce5342548383eb892bef132/code/components/jomjol_controlcamera/ClassControllCamera.cpp#L594)

### make OV2640 zoom functionnal with non 4:3 images
- change image processor routine