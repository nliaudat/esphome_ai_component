# Flash Light Controller

A specialized component to manage flash/LED timing during image capture. It ensures the light is turned on, stabilized, and turned off at the correct times relative to the camera exposure.

## ✨ Features

- **Pre-flash Delay**: Configurable time to allow auto-exposure/auto-white-balance to stabilize after turning on the light.
- **Post-flash Delay**: cleanup time after capture.
- **Integration**: links directly with standard ESPHome `light` components.

## ⚙️ Configuration

```yaml
flash_light_controller:
  id: my_flash_controller

  # Link to an existing light
  flash_light: my_light_id

  # Timing Configuration
  flash_pre_time: 5s     # Time to wait after turning on before capture
  flash_post_time: 2s    # Time to keep on after (optional)

  debug: false
```

## 🔗 Usage in Readers

This component is used by `meter_reader_tflite` and `analog_reader`:

```yaml
meter_reader_tflite:
  # ...
  flash_light_controller: my_flash_controller

analog_reader:
  # ...
  flash_light_controller: my_flash_controller
```

When a reader initiates a cycle, it triggers the flash controller to prepare the lighting conditions before requesting an image. Multiple readers can share one controller: a reader that triggers during warmup joins the running sequence and captures together with the first reader. A reader that triggers during post-time captures immediately, and the post-time window is restarted so it still gets a lit frame.
