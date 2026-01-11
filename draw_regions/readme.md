# ESP32 Camera ROI & Rotation Tool Logic

This tool addresses a specific challenge with ESP32 cameras: configuring a hardware `camera_window` (ROI) when the camera is mounted at a rotation (90°, 180°, 270°).

## The Challenge

1.  **Hardware Pipeline**: The `camera_window` configuration applies to the **physical sensor interface** (Raw Data) *before* any software rotation or JPEG encoding happens.
    *   If you mount your camera at 90° and want to crop the "top half" of your visible image, you are actually cropping the "left half" (or right, depending on rotation) of the physical sensor.
2.  **Coordinate Mapping**: Humans think in "Visual Coordinates" (what they see on screen). The Hardware needs "Sensor Coordinates" (physical rows/columns).
3.  **Scaling**: Users often take screenshots of a low-res stream (e.g., 640x480) but want to configure the ROI for the full-res sensor (e.g., UXGA 1600x1200) to get maximum detail for AI processing.

## Logic Overview

The tool performs a 3-step transformation pipeline:

### 1. Coordinate Mapping (Visual → Sensor)
We map the coordinates drawn on the rotated canvas back to the unrotated sensor space.

*   **0°**: `x` → `x`, `y` → `y`
*   **90° (CW)**: `x` → `y`, `y` → `SensorWidth - (x + w)`
*   **180°**: `x` → `SensorWidth - (x + w)`, `y` → `SensorHeight - (y + h)`
*   **270° (CCW)**: `x` → `SensorHeight - (y + h)`, `y` → `x`

*(Note: Width/Height dimensions also swap for 90°/270°)*

### 2. Resolution Scaling
To allow drawing on low-res screenshots:
1.  selects the **Target Sensor Resolution** (e.g., 1600x1200).
2.  Tool calculates a `ScaleFactor = TargetRes / ImageRes`.
3.  All geometric coordinates are multiplied by this `ScaleFactor`.

### 3. Hardware Constraints (Snapping)
The ESP32 camera driver and many sensors (OV2640, etc.) have specific memory alignment requirements for windowing.

*   **Rule**: `offset_x`, `offset_y`, `width`, and `height` must be **multiples of 4**.
*   **Implementation**: `val = Math.round(val / 4) * 4`

### 4. Aspect Ratio Enforcement (Optional)
If the "Force 4:3" option is checked, the tool adds an extra constraint to preserve the square pixel aspect ratio of standard sensors.
*   **Constraint**: `Width = 16 * n`, `Height = 12 * n`
*   This ensures the ratio is exactly 4:3 (16:12) while guaranteeing both dimensions are divisible by 4.

## Output Explanation

### `camera_window`
Defines the hardware crop in **unrotated sensor space**.
```yaml
camera_window:
  offset_x: 600   # Multiples of 4
  offset_y: 200
  width: 800
  height: 600
```

### `crop_zones` (Digit Regions)
Defines sub-regions for post-processing.
*   **Reference System**: Relative to the *resulting* output image.
*   **Transform**: Because the `camera_window` generates a new image which is *then* rotated by the implementation, these coordinates must be transformed *again* into the final rotated visual space.
*   **Format**: `[[x1, y1, x2, y2], ...]` stringified JSON.
