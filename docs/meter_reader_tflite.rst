.. SPDX-License-Identifier: Apache-2.0 OR MIT

meter_reader_tflite
====================

**Status: Active (v2.0+)**

The ``meter_reader_tflite`` component is the primary AI-powered meter reader using TensorFlow Lite Micro models. It orchestrates camera capture, TFLite inference, digit recognition, and result publishing.

Configuration
-------------

.. code-block:: yaml

    # Minimal configuration
    meter_reader_tflite:
      id: my_meter
      model: "digit_recognizer.tflite"
      camera_id: my_camera
      update_interval: 60s

    # Full configuration
    meter_reader_tflite:
      id: my_meter
      model: "digit_recognizer.tflite"
      camera_id: my_camera
      validator: my_validator
      tensor_arena_size: 512KB
      confidence_threshold: 0.85
      flash_light_controller: my_flash
      data_collector: my_collector
      collect_low_confidence: true
      collect_min_global_confidence: 0.90
      collect_min_digit_confidence: 0.90
      frame_request_timeout: 15000
      debug: false
      debug_image: false

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **model** (*Required*): Path to the TFLite model file (``.tflite``).
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **validator** (*Optional*): The ID of a ``value_validator`` component.
- **tensor_arena_size** (*Optional*): Memory allocation for TFLite. Auto-detected from model metadata. Default: 512KB (ESP32), 768KB (ESP32-S3).
- **confidence_threshold** (*Optional*): Minimum confidence to publish a reading. Default: 0.85.
- **update_interval** (*Optional*): Polling interval. Default: 60s.
- **rotation** (*Optional*): Image rotation (0, 90, 180, 270). Default: 0.
- **debug** (*Optional*): Enable debug logging. Default: false.

Performance
-----------

On ESP32-S3 with an optimized model, the full pipeline (capture + inference) completes in **under 270ms**. On classic ESP32, expect under 2700ms.

Dependencies
------------

- ``tflite_micro_helper`` (required)
- ``esp32_camera_utils`` (required)
- ``value_validator`` (optional)
- ``flash_light_controller`` (optional)
- ``data_collector`` (optional)