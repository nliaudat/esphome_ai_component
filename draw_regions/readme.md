# ESP32 Camera ROI Configuration Tool

This tool addresses a specific challenge with ESP32 cameras: configuring a hardware window (ROI) when the camera is mounted at a rotation (90°, 180°, 270°). It targets the native `esp32_camera.window` API merged in [esphome/esphome#20480](https://github.com/esphome/esphome/pull/20480) (ESPHome 2026.11+).

**Live tool:** [`roi.html`](https://nliaudat.github.io/esphome_ai_component/roi.html) — open in a browser to use.

## The Challenge

1.  **Hardware Pipeline**: The window configuration applies to the **physical sensor readout** *before* any software rotation or JPEG encoding happens.
    *   If you mount your camera at 90° and want to crop the "top half" of your visible image, you are actually cropping the "left half" (or right, depending on rotation) of the physical sensor.
2.  **Coordinate Mapping**: Humans think in "Visual Coordinates" (what they see on screen). The hardware needs frame coordinates in the camera's own orientation.
3.  **Coordinate Space**: Where the window is counted depends on the sensor. Load a frame captured by the camera (as published to Home Assistant) and draw on that; the tool converts the drawing into the unrotated window.

## Per-Sensor Behaviour (`esp32_camera.window`)

| Sensor | Window counted in | Published frame | Notes |
|---|---|---|---|
| OV2640 | Pixels of the `resolution` | The size of the window | JPEG output only; rounded to 16x8 blocks |
| OV3660, OV5640 | Pixels of the `resolution` | The `resolution` | Scaled down to the frame, so it also zooms in |
| SC101IOT, SC030IOT | Pixels of the whole sensor (1280x720 / 640x480) | The `resolution` | `width` and `height` must equal the `resolution` |

Further rules from the driver:

*   A window that reaches past the edge is **clamped** to it.
*   The window is counted in `resolution` pixels, so set it again when you change the `resolution`.
*   A new window always replaces the previous one; it never nests.
*   Changing the frame size at runtime (from a lambda) clears the window.
*   If a window cannot be applied, the previous one stays in place.
*   Sensors other than the five above cannot read out a window.
*   OV3660/OV5640 only scale **down**: a window that covers fewer sensor pixels than the frame is rejected. Pick a lower `resolution` (e.g. 640x480) if you want to zoom in.
*   SC101IOT/SC030IOT have no scaler, so a window of another size is rejected; they only pan.

## Logic Overview

The tool performs a 3-step transformation pipeline:

### 1. Coordinate Mapping (Visual → Frame)

We map the coordinates drawn on the rotated canvas back to the unrotated frame space.

*   **0°**: `x` → `x`, `y` → `y`
*   **90° (CW)**: `x` → `y`, `y` → `FrameWidth - (x + w)`
*   **180°**: `x` → `FrameWidth - (x + w)`, `y` → `FrameHeight - (y + h)`
*   **270° (CCW)**: `x` → `FrameHeight - (y + h)`, `y` → `x`

*(Note: Width/Height dimensions also swap for 90°/270°)*

### 2. Resolution Scaling

To allow drawing on screenshots that are not at the camera resolution:

1.  Enter the **Camera W / Camera H** matching `esp32_camera.resolution` (defaults to the loaded image size).
2.  Tool calculates a `ScaleFactor = CameraResolution / ImageResolution`.
3.  All geometric coordinates are multiplied by this `ScaleFactor`.

### 3. Sensor Constraints

The window values are handed to `esp32_camera`, which applies the sensor rules above. The tool already:

*   rounds **OV2640** windows down to a width multiple of 16 and a height multiple of 8 (the driver rounds down and JPEG compresses whole blocks);
*   forces **SC101IOT/SC030IOT** windows to the `resolution` size and clamps them to the sensor area, so the YAML is accepted (the ROI then only pans);
*   warns when an **OV3660/OV5640** window would need upscaling (at the sensor's full resolution), which the driver rejects.

### 4. Aspect Ratio Enforcement (Optional)

If the "Force 4:3" option is checked, the tool adds an extra constraint to preserve the square pixel aspect ratio of standard sensors.

*   **Constraint**: `Width = 32 * n`, `Height = 24 * n`
*   This keeps the ratio exactly 4:3 (32:24) while guaranteeing both dimensions are divisible by 8.

## Usage

1. Open [`roi.html`](roi.html) in a browser.
2. Click **📂 Load Image** to load a screenshot of your camera feed.
3. Select the camera sensor model from the **Sensor Model** dropdown.
4. Set **Camera W / Camera H** to match `esp32_camera.resolution`.
5. Set **Rotation** if the camera is mounted at an angle (90°, 180°, 270°).
6. **Click and drag** on the canvas to draw a BLUE ROI rectangle.
7. Optionally enable **Force 4:3** to maintain aspect ratio.
8. Optionally set **Target Resolution** if scaling the output (e.g. for AI model input).
9. Copy the generated YAML config and paste into your ESPHome configuration.

## Output

### `esp32_camera.window` (preferred)

Defines the hardware crop in unrotated `resolution` pixels (whole-sensor pixels on the SC sensors):

```yaml
esp32_camera:
  # ... your existing camera settings ...
  # OV2640 only: window readout requires pixel_format: JPEG
  window:
    offset_x: 160
    offset_y: 80
    width: 320
    height: 240
```

Merge these keys into your existing `esp32_camera:` block — do not add a second
`esp32_camera:` key.

### Runtime actions

The same window can be applied without recompiling:

```yaml
- esp32_camera.set_window: {id: my_camera, offset_x: 160, offset_y: 80, width: 320, height: 240}
- esp32_camera.reset_window: my_camera
```

### `esp32_camera_utils`

The tool also generates the rotation settings for `esp32_camera_utils`:

```yaml
esp32_camera_utils:
  id: ${id_prefix}_camera_utils
  camera_id: ${id_prefix}_espcam
  enable_rotation: true
  rotation: 270
```

`esp32_camera_utils.camera_window` still exists as a compatibility alias applied through the same native API, but it is deprecated: prefer `esp32_camera.window` and do not set both.
