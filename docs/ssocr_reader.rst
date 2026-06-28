.. SPDX-License-Identifier: Apache-2.0 OR MIT

ssocr_reader
============

**Status: Alpha** (experimental, API may change)

The ``ssocr_reader`` component provides seven-segment OCR reading using the SSOCR algorithm. It reads digital displays without requiring AI models.

Configuration
-------------

.. code-block:: yaml

    ssocr_reader:
      id: my_ssocr
      camera_id: my_camera
      validator: my_validator
      update_interval: 5s
      threshold_level: 128
      digit_count: 8

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **validator** (*Optional*): The ID of a ``value_validator`` component.
- **threshold_level** (*Optional*): Binarization threshold (0-255). Default: 128.
- **digit_count** (*Optional*): Number of digits to read (1-16). Default: 6.
- **update_interval** (*Optional*): Polling interval. Default: 5s.

Dependencies
------------

- ``esp32_camera`` (required)
- ``esp32_camera_utils`` (auto-loaded)
