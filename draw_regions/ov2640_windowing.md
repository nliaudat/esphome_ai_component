# OV2640 Windowing Calculation Explanation

Based on the [OV2640 Datasheet (OV2640DS.pdf)](https://www.uctronics.com/download/cam_module/OV2640DS.pdf), the camera's region of interest (Windowing) is controlled by setting specific registers that define the horizontal and vertical start and end points of the active image area.

## Key Registers

The primary registers controlling windowing are:

1.  **Horizontal Windowing**:
    -   **`HREFST`** (Horizontal Reference Start): Defines the starting pixel column of the window.
    -   **`HREFEND`** (Horizontal Reference End): Defines the ending pixel column of the window.
    -   **Formula**: `Horizontal Window Size = (HREFEND - HREFST + 1)`

2.  **Vertical Windowing**:
    -   **`VSTRT`** (Vertical Start): Defines the starting pixel row of the window.
    -   **`VEND`** (Vertical End): Defines the ending pixel row of the window.
    -   **Formula**: `Vertical Window Size = (VEND - VSTRT + 1)`

## Output Scaling

After defining the window, the output size is often scaled down. In common driver implementations (like ESP32-CAM), the output size registers (`HSIZE`, `VSIZE`) are typically set to:

*   `Output Width = Window Width / 4`
*   `Output Height = Window Height / 4`

This means if you define a window of 800x600, the raw sensor output might be configured to 200x150, or scaled differently depending on the DSP configuration.

## How the ROI Tool Works

The **ROI Tool** ([`roi.html`](roi.html)) calculates the `offset_x`, `offset_y`, `width`, and `height` from your selection on the camera canvas.

When you draw a rectangle:
1.  **`offset_x` / `offset_y`**: The top-left corner of the crop, in unrotated frame pixels.
2.  **`width` / `height`**: The size of the crop, in unrotated frame pixels.

Since ESPHome 2026.11 the native `esp32_camera.window` configuration takes
these high-level values in `resolution` pixels (not full-sensor pixels):
```yaml
esp32_camera:
  resolution: 640x480
  window:
    offset_x: <calculated>
    offset_y: <calculated>
    width: <calculated>
    height: <calculated>
```
The underlying `esp32_camera` driver then converts these values into the crop
registers the OV2640 needs. The former `esp32_camera_utils.camera_window` key
still works as a deprecated compatibility alias with the same
resolution-pixel semantics.

OV2640 specifics, as implemented by the driver:

*   The window readout requires **JPEG output** (`pixel_format: JPEG`); with any
    other format the window is rejected.
*   `width` is rounded **down** to a multiple of **16** and `height` down to a
    multiple of **8**, because JPEG compresses whole 16x8 blocks. A window
    smaller than one block grows to it.
*   The published frame is the (rounded) size of the window.
*   The crop registers themselves are programmed in multiples of 4 pixels, so
    the offsets are aligned by the driver — the tool's values do not need to be
    pre-aligned.
*   A window that reaches past the edge is clamped, and changing
    `esp32_camera.resolution` requires setting the window again.

## Driver Implementation (`esp32-camera`)

The [espressif/esp32-camera](https://github.com/espressif/esp32-camera) repository implements these controls in the `OV2640` sensor driver (specifically `sensors/ov2640.c`). The driver uses functions usually named `set_res_raw` or `set_window` to perform the following:
1.  **Bank Switching**: Selects the appropriate register bank (often DSP Bank 1).
2.  **Register Calculation**: Converts the requested `offset_x`, `offset_y`, `width`, and `height` into the specific start/end values required by `HREFST`, `HREFEND`, `VSTRT`, and `VEND`.
3.  **Output Scaling**: Configures `HSIZE`, `VSIZE`, `XOFFL`, and `YOFFL` to define the final output resolution sent to the ESP32.

This abstraction allows ESPHome users to simply define the `window` in pixels, and the driver handles the complex register math and timing constraints.

## Datasheet Reference Details

The following sections are derived from the OV2640 datasheet to provide deeper technical context.

### Windowing
The OV2640 allows the user to define window size or region of interest (ROI), as required by the application.
*   **Range**: Window size setting (in pixels) ranges from **2 x 4 to 1632 x 1220 (UXGA)** or **2 x 2 to 818 x 610 (SVGA)**, and **408 x 304 (CIF)**.
*   **Position**: The window can be placed anywhere inside the 1632 x 1220 boundary.
*   **Frame Rate**: Modifying window size or position **does not alter the frame or pixel rate**.
*   **Mechanism**: The windowing control merely alters the assertion of the `HREF` signal to be consistent with the programmed horizontal and vertical ROI.
*   **Default**: The default window size is 1600 x 1200.
*   **Registers**: Refer to Figure 4 and registers `HREFST`, `HREFEND`, `REG32`, `VSTRT`, `VEND`, and `COM1` for details.

### Zooming and Panning Mode
The OV2640 provides zooming and panning modes. The user can select this mode under SVGA/CIF mode timing.
*   **Zoom Ratios**:
    *   **2:1** of UXGA for SVGA.
    *   **4:1** of UXGA for CIF.
*   **Registers**:
    *   `ZOOMS[7:0]` (0x49) and `COM19[1:0]` (0x48) define the **vertical line start point**.
    *   `ARCOM2[2]` (0x34) defines the **horizontal start point**.

### Sub-sampling Mode
The OV2640 supports two sub-sampling modes. Each sub-sampling mode has different resolution and maximum frame rate.

#### SVGA Mode
The OV2640 can be programmed to output **800 x 600 (SVGA)** sized images for applications where higher resolution image capture is not required. In this mode, both horizontal and vertical pixels will be sub-sampled with an **aspect ratio of 4:2**.

#### CIF Mode
The OV2640 can also operate at a higher frame rate to output **400 x 296** sized images. This involves sub-sampling in both horizontal and vertical directions for CIF mode.
