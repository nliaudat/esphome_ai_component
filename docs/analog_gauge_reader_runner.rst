.. SPDX-License-Identifier: Apache-2.0 OR MIT

analog_gauge_reader_runner
==========================

**Status: Alpha** (experimental, API may change)

The ``analog_gauge_reader_runner`` component reads one or more analog dials out
of ``esp32_camera`` frames. It decodes each JPEG once through the upstream
``camera_decoder`` component, crops every configured dial out of that single
decoded buffer, and asks the upstream ``gauge_needle_detector`` component for
the needle angle of each crop.

It is the local counterpart of the upstream ``analog_gauge_reader`` component,
and keeps the optional ``value_validator`` and ``flash_light_controller``
integration that the upstream component does not carry.

Configuration
-------------

.. code-block:: yaml

    analog_gauge_reader_runner:
      id: my_gauge
      camera_id: my_camera
      decoder_id: my_decoder
      validator: my_validator
      flash_light_controller: my_flash
      update_interval: 60s
      value_sensor:
        name: "Gauge total"
      dials:
        - id: dial1
          crop_x: 0
          crop_y: 0
          crop_w: 64
          crop_h: 64
          scale: 0.1
          min_value: 0
          max_value: 100
          value:
            name: "Dial 1"
          angle:
            name: "Dial 1 angle"
          confidence:
            name: "Dial 1 confidence"

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **decoder_id** (*Required*): The ID of a ``camera_decoder`` component. Its
  ``output_type`` must be ``RGB888`` or ``GRAYSCALE``; a colour dial needs
  ``RGB888``.
- **dials** (*Required*): List of dial configurations. Each dial has:
  - **id** (*Required*): String identifier for logs and sensor entries.
  - **scale** (*Optional*): Weight in the summed value. Default: ``1.0``.
  - **crop_x** / **crop_y** / **crop_w** / **crop_h** (*Optional*): Crop of the
    frame, in pixels. Defaults: ``0``/``0``/``64``/``64``.
  - **min_value** / **max_value** (*Optional*): Value range. Defaults: ``0``/``10``.
  - **min_angle** / **max_angle** / **angle_offset** / **clockwise**
    (*Optional*): Angle mapping, in degrees.
  - **needle_type** (*Optional*): ``DARK`` (default) or ``LIGHT``.
  - **algorithm** (*Optional*): ``radial_profile`` (default), ``legacy``,
    ``hough_transform``, ``template_match`` or ``auto``.
  - **process_channel** (*Optional*): ``GRAYSCALE`` (default), ``RED``,
    ``GREEN`` or ``BLUE``. Mutually exclusive with ``target_color``.
  - **target_color** / **color_tolerance** (*Optional*): Detect a coloured needle.
  - **min_scan_radius** / **max_scan_radius** / **deadzone_diameter**
    (*Optional*): Detector scan window and ignored centre circle.
  - **auto_contrast** / **contrast** (*Optional*): Crop normalisation.
  - **calibration** (*Optional*): List of ``raw``/``mapped`` pairs.
  - **value** / **angle** / **confidence** (*Optional*): Per-dial sensors.
- **value_sensor** (*Optional*): The combined value sensor.
- **validator** (*Optional*): The ID of a ``value_validator`` component. The
  runner feeds it the summed dial fraction for digit-boundary correction.
- **flash_light_controller** (*Optional*): The ID of a
  ``flash_light_controller`` component. The runner then shares its flash cycle
  with ``meter_reader_tflite``.
- **paused** (*Optional*): Start paused. Default: ``false``.
- **debug** (*Optional*): Verbose per-algorithm logging. Default: ``false``.
- **stacked_digits** (*Optional*): Combine dials as odometer digits. Requires
  ``scale`` values that are consecutive powers of ten. Default: ``false``.

Dependencies
------------

- ``esp32_camera`` (required), configured to deliver JPEG frames: either
  ``pixel_format: JPEG`` (the default) or any other ``pixel_format`` combined
  with a non-zero ``jpeg_quality``.
- ``camera_decoder`` (required, upstream).
- ``gauge_needle_detector`` (auto-loaded, upstream).
