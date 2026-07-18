.. SPDX-License-Identifier: Apache-2.0 OR MIT

meter_reader_tflite
====================

**Status: Active (v2.0+)**

The ``meter_reader_tflite`` component is the primary AI-powered meter reader using TensorFlow Lite Micro models. It orchestrates camera capture, TFLite inference, digit recognition, and result publishing.

**Note:** The model configuration has moved from ``meter_reader_tflite`` to the
``tflite_micro_helper`` component. You now configure the model in a
``tflite_micro_helper`` entry and reference it by ID from ``meter_reader_tflite``.

Configuration
-------------

.. code-block:: yaml

    # New format: model defined in tflite_micro_helper, referenced by ID
    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "digit_recognizer_v40_quantized_integer_quant_uint8.tflite"

    meter_reader_tflite:
      id: my_meter
      model: my_digit_model          # ← ID reference to tflite_micro_helper entry
      camera_id: my_camera
      update_interval: 60s

    # Full configuration
    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "digit_recognizer.tflite"
        # Optional overrides:
        # tensor_arena_size: "110KB"
        # input_width: 32
        # input_height: 20

    meter_reader_tflite:
      id: my_meter
      model: my_digit_model
      camera_id: my_camera
      validator: my_validator
      confidence_threshold: 0.85
      flash_light_controller: my_flash
      data_collector: my_collector
      collect_low_confidence: true
      collect_min_global_confidence: 0.90
      collect_min_digit_confidence: 0.90
      frame_request_timeout: 15000
      debug: false
      debug_image: false

    # Legacy format (deprecated — prints warning):
    # meter_reader_tflite:
    #   model: "digit_recognizer.tflite"   # ← Direct file path (old)

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **model** (*Required*): The ID of a ``tflite_micro_helper`` entry that defines the model.
  *Legacy:* A direct ``.tflite`` file path (deprecated, prints warning).
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **validator** (*Optional*): The ID of a ``value_validator`` component.
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