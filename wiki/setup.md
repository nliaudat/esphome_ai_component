
## Setup : 


### Prerequisites
- Python ([python.org](https://python.org))
- Esphome (pip install esphome)

### Download 
Download full repo or git clone 

## Adapt for your needs
### config.yaml : 
 - [ ] [optional] change the name in the substitutions section
 - [ ] [optional] change the timezone
 - [ ] meter_reader_tflite :
 Start with a minimal config with no camera_window (zoom)
 Copy the model you choose from ./model to the config.yaml directory
 set it's name in model: "digit_recognizer_v4_10cls_RGB.tflite" 
  - [ ] packages :
 Select the board by commenting or not the board include

### secrets.yaml
change your wifi passwords and SSID

The number of wifi must be adapt in wifi.yaml

### globals_AI_component.yaml
comment out or clear "initial_value"
initial_value: "\"[]\""
This are the crop zone for digit identification

## Compile and upload

    esphome compile config.yaml
  If success - connect your board with usb (only needed the first time, the next upload can be done with OTA)

      esphome run config.yaml


## Go to your hassio
* Add the new detected board
* Pause the AI processing
* Enabled Flash LED
* You can view and download the image by clicking the camera.
<img width="496" height="845" alt="image" src="https://github.com/user-attachments/assets/87ca3598-340f-45e6-9036-3b483579899c" />

## Configuration (Camera, ROI & Zones)

*(See also: [Understanding Crop Zones and Loop Time Performance](setting_crop_zones.md) for details on optimizing your ESP32's processing speed.)*

We provide a graphical tool to easily configure your camera rotation, window (ROI), and digit/dial zones.

1.  **Open the tool**: Open `draw_regions/index.html` in your browser.
2.  **Load Image**: Click "Load Image" and select a snapshot from your ESP32 camera (captured via Home Assistant or the web server).
3.  **Set Rotation (Optional)**: If your camera is mounted at an angle (90/180/270), click the corresponding button.
4.  **Set Camera ROI (Optional)**:
    *   Click "1. Set Camera ROI".
    *   Draw a blue rectangle to define the active sensor area. This "digital zoom" reduces data transfer and processing time.
5.  **Set Zones**:
    *   Click "2. Set Digit Zones" or "3. Set Analog Dials".
    *   Draw green boxes for digits and orange boxes for dials.
6.  **Generate Config**:
    *   The tool automatically generates the required YAML configuration.
    *   Click "Copy to Clipboard".

### Apply Configuration

Paste the generated YAML into your configuration files:

1.  **Crop Zones**: Paste the `globals:` section into your main config or `globals_AI_component.yaml`.
2.  **Camera Settings**: Paste the `esp32_camera_utils:` section into your config (replacing the existing block).

*Note: You can also test these settings temporarily using Home Assistant Developer Tools services `esphome.[node_name]_set_camera_window` and `esphome.[node_name]_set_crop_zones`.*

## Final compile

      esphome run config.yaml
