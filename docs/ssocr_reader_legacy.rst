.. SPDX-License-Identifier: Apache-2.0 OR MIT

ssocr_reader_legacy
===================

**Status: Legacy** (superseded by :doc:`ssocr_reader_runner`)

The ``ssocr_reader_legacy`` component is the original seven-segment OCR reader: it bundles the SSOCR algorithm and JPEG decoding through ``esp32_camera_utils`` in one component.

It is kept for existing configurations. New configurations should use :doc:`ssocr_reader_runner`, which builds on the upstream ``camera_decoder`` and ``seven_segment_ocr`` components.

Configuration
-------------

.. code-block:: yaml

    ssocr_reader_legacy:
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
- **value** (*Optional*): A ``sensor`` sub-configuration for the OCR reading.
- **confidence** (*Optional*): A ``sensor`` sub-configuration for recognition confidence (0-100%, averaged per-digit margin in percent).
- **threshold_level** (*Optional*): Binarization threshold (0-255). Default: 128.
- **threshold_type** (*Optional*): Binarization method. Only ``fixed`` is currently implemented. Default: ``fixed``.
- **crop_x** / **crop_y** / **crop_w** / **crop_h** (*Optional*): Crop region. Bounds-checked against the camera resolution at config time. Default: 0 (full frame).
- **digit_count** (*Optional*): Number of digits to read (1-16). Default: 6.
- **update_interval** (*Optional*): Polling interval. Default: 5s.

Dependencies
------------

- ``esp32_camera`` (required)
- ``esp32_camera_utils`` (auto-loaded)
