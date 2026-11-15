.. SPDX-License-Identifier: Apache-2.0 OR MIT

esp32_camera_utils
==================

**Status: Active**

The ``esp32_camera_utils`` component provides image processing utilities
including camera windowing (digital zoom), cropping, scaling, rotation (JPEG and
Raw formats), drawing and format conversion using the ``esp_new_jpeg``
library. It is auto-loaded by ``meter_reader_tflite`` and can also be used
standalone.

.. note::

    Requires ESPHome **2026.11** or later. Windowing uses the native
    ``esp32_camera`` window API (:pr:`20480`), so the component no longer
    writes sensor registers itself.

Configuration
-------------

.. code-block:: yaml

    esp32_camera:
      id: my_camera
      resolution: 640x480
      pixel_format: JPEG
      # Preferred place for the hardware window (digital zoom).
      window:
        offset_x: 160
        offset_y: 80
        width: 320
        height: 240

    esp32_camera_utils:
      id: my_camera_utils
      camera_id: my_camera

      # Global rotation (clockwise degrees). Arbitrary float;
      # 0/90/180/270 are hardware-optimized, other values use software rotation.
      rotation: 90.0
      enable_rotation: true

      # Compatibility alias for esp32_camera.window. Do not set both.
      # camera_window:
      #   offset_x: 160
      #   offset_y: 80
      #   width: 320
      #   height: 240

      # Scaling and cropping of the processed zones
      # scaler:
      #   width: 32
      #   height: 20
      # cropper:
      #   width: 32
      #   height: 20
      #   offset_x: 0
      #   offset_y: 0

      # debug: true
      # debug_memory: true

Configuration variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Optional*): The ID of the ``esp32_camera`` component.
- **window** (*Optional*): Compatibility copy of ``esp32_camera.window``.
  Applied through the same native API. Setting both is a configuration error.
- **camera_window** (*Optional*): Deprecated alias of ``window``.
- **rotation** (*Optional*): Clockwise rotation in degrees. Defaults to ``0.0``.
  Any float value is accepted; multiples of 90° are hardware-optimized.
- **enable_rotation** (*Optional*, boolean): Enable the rotation stage.
  Defaults to ``false``. Enabled automatically when ``rotation`` is not 0.
- **scaler** (*Optional*): ``width`` / ``height`` of the scaled output.
- **cropper** (*Optional*): ``width``, ``height``, ``offset_x``, ``offset_y`` of
  the cropped region.
- **enable_scaler**, **enable_cropper**, **enable_drawing** (*Optional*,
  boolean): Compile-in switches for the optional stages. All default to
  ``true``.
- **debug** (*Optional*, boolean): Verbose processing logs. Defaults to
  ``false``.
- **debug_memory** (*Optional*, boolean): Log buffer statistics and create the
  diagnostic sensors below. Defaults to ``false``.
- **camera_buffer_size_sensor**, **camera_free_psram_sensor** (*Optional*):
  Sensor IDs for the buffer size and free PSRAM values.

Window coordinates
------------------

Window coordinates are in the pixels of ``esp32_camera.resolution``, **not** the
full sensor. This differs from older releases, which used full-sensor
coordinates. With ``resolution: 640x480``, the window
``offset_x: 160, offset_y: 80, width: 320, height: 240`` is valid, while a
full-sensor window such as ``width: 1600`` would be clamped.

A window must fit inside ``resolution``: ``offset_x + width`` and
``offset_y + height`` are checked against it. A larger window is rejected at
build time (and reported by a runtime warning). Because a full-sensor window no
longer fits a small ``resolution``, digital zoom wants the camera set to the
sensor's full frame (``1600x1200`` for the OV2640) so the window numbers keep
meaning sensor pixels.

Supported sensors: **OV2640** (JPEG output only), **OV3660**, **OV5640**,
**SC101IOT** and **SC030IOT**. The published frame size depends on the sensor:

.. list-table::
    :header-rows: 1

    * - Sensor
      - Published frame
    * - OV2640
      - Window size, rounded to 16×8 JPEG blocks
    * - OV3660 / OV5640
      - Configured resolution (the window is scaled down into it)
    * - SC101IOT / SC030IOT
      - Configured resolution (the window must equal it)

Debug sensors
-------------

With ``debug_memory: true`` these diagnostic sensors are created automatically:

- ``camera_buffer_size`` — size of the current frame buffer (B)
- ``camera_free_psram`` — free PSRAM (B)
- ``camera_min_free_psram`` — minimum free PSRAM since boot (B)
- ``camera_min_free_internal`` — minimum free internal RAM since boot (B)

The two "minimum free since boot" values are watermarks: they expose long-run
fragmentation and growth that a momentary free-memory value hides.

Dependencies
------------

- ``esp32_camera`` (required)
- ``espressif/esp_new_jpeg`` 1.0.2 (automatically added)
