# ESP32 Camera ROI Configuration Tool

This tool addresses a specific challenge with ESP32 cameras: configuring a hardware window (ROI) when the camera is mounted at a rotation (90°, 180°, 270°). It targets the native `esp32_camera.window` API (ESPHome 2026.11.0+).

**Live tool:** [`roi.html`]([roi.html](https://nliaudat.github.io/esphome_ai_component/roi.html)— Open in browser to use.

## The Challenge

1.  **Hardware Pipeline**: The window configuration applies to the **physical sensor interface** (Raw Data) *before* any software rotation or JPEG encoding happens.
    *   If you mount your camera at 90° and want to crop the "top half" of your visible image, you are actually cropping the "left half" (or right, depending on rotation) of the physical sensor.
2.  **Coordinate Mapping**: Humans think in "Visual Coordinates" (what they see on screen). The hardware needs unrotated frame coordinates.
3.  **Camera Resolution**: Window coordinates are in the pixels of `esp32_camera.resolution` (e.g., 640x480), **not** the full sensor. This changed in ESPHome 2026.11.0; older releases used full-sensor coordinates.

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

### 3. Hardware Constraints (Snapping)
The ESP32 camera driver and many sensors (OV2640, etc.) have specific memory alignment requirements for windowing.

*   **Rule**: `offset_x`, `offset_y`, `width`, and `height` must be **multiples of 4**.
*   **Implementation**: `val = Math.round(val / 4) * 4`

### 4. Aspect Ratio Enforcement (Optional)
If the "Force 4:3" option is checked, the tool adds an extra constraint to preserve the square pixel aspect ratio of standard sensors.
*   **Constraint**: `Width = 16 * n`, `Height = 12 * n`
*   This ensures the ratio is exactly 4:3 (16:12) while guaranteeing both dimensions are divisible by 4.

## Usage

1. Open [`roi.html`](roi.html) in a browser.
2. Click **📂 Load Image** to load a screenshot of your camera feed.
3. Select the camera sensor model from the **Sensor Model** dropdown (informational).
4. Set **Camera W / Camera H** to match `esp32_camera.resolution`.
5. Set **Rotation** if the camera is mounted at an angle (90°, 180°, 270°).
6. **Click and drag** on the canvas to draw a BLUE ROI rectangle.
7. Optionally enable **Force 4:3** to maintain aspect ratio.
8. Optionally set **Target Resolution** if scaling the output (e.g. for AI model input).
9. Copy the generated YAML config and paste into your ESPHome configuration.

## Output

### `esp32_camera.window` (preferred)
Defines the hardware crop in **unrotated camera-resolution space**.
```yaml
esp32_camera:
  # ... your existing camera settings ...
  window:
    offset_x: 160   # Multiples of 4
    offset_y: 80
    width: 320
    height: 240
```

The tool also emits the same values as `esp32_camera_utils.camera_window`, a
compatibility alias that is applied through the native window API. Do not set
both.

Also generates `rotation` and `enable_rotation` directives for the
`esp32_camera_utils` configuration block.
