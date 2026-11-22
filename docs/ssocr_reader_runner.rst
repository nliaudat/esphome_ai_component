.. SPDX-License-Identifier: Apache-2.0 OR MIT

ssocr_reader_runner
===================

**Status: Alpha** (experimental, API may change)

The ``ssocr_reader_runner`` component reads a seven-segment display out of
``esp32_camera`` frames. It decodes each JPEG once through the upstream
``camera_decoder`` component, crops the configured rectangle out of that image
as one byte per pixel, and asks the upstream ``seven_segment_ocr`` component for
the digits.

It is the local counterpart of the upstream ``ssocr_reader`` component, and
keeps the optional ``value_validator`` and ``flash_light_controller``
integration plus inline sensors.

Configuration
-------------

.. code-block:: yaml

    ssocr_reader_runner:
      id: my_ssocr
      camera_id: my_camera
      decoder_id: my_decoder
      validator: my_validator
      flash_light_controller: my_flash
      crop_x: 8
      crop_y: 16
      crop_w: 160
      crop_h: 40
      digit_count: 6
      value:
        name: "Meter reading"
      confidence:
        name: "OCR confidence"

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **decoder_id** (*Required*): The ID of a ``camera_decoder`` component. Its
  ``output_type`` must be ``GRAYSCALE`` or ``RGB888`` (RGB is converted to luma).
- **crop_x** / **crop_y** (*Optional*): Crop origin, in decoded-frame pixels.
  Default: ``0``.
- **crop_w** / **crop_h** (*Required*): Crop size, in pixels. Validated against
  the frame size after the decoder's rotation.
- **threshold_level** (*Optional*): Binarization threshold (0–255). Default: ``128``.
- **digit_count** (*Optional*): Digits to read (1–7). Default: ``6``. The cap
  exists because the value sensor is a float and floats represent integers
  exactly only up to 2^24.
- **validator** (*Optional*): The ID of a ``value_validator`` component. A
  reading the validator rejects is not published, so the last good value stays.
- **flash_light_controller** (*Optional*): The ID of a
  ``flash_light_controller`` component. The runner then shares its flash cycle
  with ``meter_reader_tflite``.
- **value** / **confidence** (*Optional*): Sensors for the reading and the mean
  segment confidence (0–100 %).
- **debug** (*Optional*): Verbose logging. Default: ``false``.
- **update_interval** (*Optional*): Polling interval. Default: ``60s``.

Dependencies
------------

- ``esp32_camera`` (required), configured to deliver JPEG frames: either
  ``pixel_format: JPEG`` (the default) or any other ``pixel_format`` combined
  with a non-zero ``jpeg_quality``.
- ``camera_decoder`` (required, upstream).
- ``seven_segment_ocr`` (auto-loaded, upstream).

.. note::

    The OCR reads upright, bright-on-dark displays. Correct the skew of slanted
    displays before the crop, and pick a crop that contains the digits with a
    little margin.
