.. SPDX-License-Identifier: Apache-2.0 OR MIT

analog_gauge_reader
===================

**Status: Alpha** (experimental, API may change)

The ``analog_gauge_reader`` component reads analog dial/gauge positions using radial intensity sum or other algorithms. It reads pointer positions without requiring AI models.

Configuration
-------------

.. code-block:: yaml

    analog_gauge_reader:
      id: my_analog
      camera_id: my_camera
      validator: my_validator
      update_interval: 60s
      dials:
        - id: dial1
          min_value: 0
          max_value: 100
          min_scan_radius: 0.3
          max_scan_radius: 0.9

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **dials** (*Required*): List of dial configurations. Each dial has:
  - **id** (*Required*): String identifier for logs.
  - **min_value** / **max_value** (*Required*): Value range.
  - **min_scan_radius** / **max_scan_radius** (*Required*): Scan radius as fraction of crop size.
  - **algorithm** (*Optional*): Detection algorithm. Options: ``radial_profile`` (default), ``legacy``, ``hough_transform``, ``template_match``, ``auto``.
  - **needle_type** (*Optional*): ``DARK`` (default) or ``LIGHT``.
- **validator** (*Optional*): The ID of a ``value_validator`` component.
- **paused** (*Optional*): Start paused. Default: false.

Dependencies
------------

- ``esp32_camera`` (required), configured to deliver JPEG frames: either
  ``pixel_format: JPEG`` (the default) or any other ``pixel_format`` combined
  with a non-zero ``jpeg_quality``, which makes ``esp32_camera`` convert frames
  to JPEG. Other combinations are rejected at config time.
- ``esp32_camera_utils`` (auto-loaded)
