.. SPDX-License-Identifier: Apache-2.0 OR MIT

ESPHome AI Components
=====================

This project provides a modular framework for running TensorFlow Lite Micro models and computer vision algorithms on ESP32 devices within the ESPHome ecosystem.

**New in v2.0:** The project has been refactored into modular components for better maintainability and reusability, and the components are committed to ESPHome (upstream PRs :pr:`21282`–:pr:`21288`).

.. toctree::
    :maxdepth: 2
    :caption: Components

    meter_reader_tflite
    tflite_micro_runtime
    tflite_micro_helper
    value_validator
    esp32_camera_utils
    flash_light_controller
    data_collector
    ssocr_reader_runner
    ssocr_reader_legacy
    analog_gauge_reader_runner
    analog_gauge_reader_legacy
    camera_layer_next
    esp_nn_s3_kernels

Installation
------------

Requires ESPHome **2026.11** or later.

The TFLite stack is split into seven components that are still shipped as
upstream PR heads, so each one needs its own ``source`` entry. The runtime
auto-loads the other six.

.. code-block:: yaml

    external_components:
      # Project components
      - source:
          type: git
          url: https://github.com/nliaudat/esphome_ai_component
          ref: next
        components:
          - meter_reader_tflite
          - value_validator
          - esp32_camera_utils
          - flash_light_controller

      # TFLite stack: the runtime plus the components it auto-loads
      - source: github://pr#21282
        components: [tflite_model_source]
      - source: github://pr#21283
        components: [tflite_op_processor]
      - source: github://pr#21284
        components: [tflite_memory_handler]
      - source: github://pr#21285
        components: [tflite_output_processor]
      - source: github://pr#21286
        components: [tflite_model_loader]
      - source: github://pr#21287
        components: [tflite_model_debug]
      - source: github://pr#21288
        components: [tflite_micro_runtime]

``tflite_micro_helper`` has been **removed**; its page documents the old
component and the migration path. ``micro_wake_word`` is now the upstream
self-contained component (no local runtime dependency).

License
-------

Apache 2.0 OR MIT, at your option.
