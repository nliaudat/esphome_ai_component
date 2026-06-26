.. SPDX-License-Identifier: Apache-2.0 OR MIT

ESPHome AI Components
=====================

This project provides a modular framework for running TensorFlow Lite Micro models and computer vision algorithms on ESP32 devices within the ESPHome ecosystem.

.. toctree::
    :maxdepth: 2
    :caption: Components

    meter_reader_tflite
    value_validator
    esp32_camera_utils
    tflite_micro_helper
    flash_light_controller
    data_collector
    ssocr_reader
    analog_reader

Installation
------------

Add the components to your ESPHome configuration using ``external_components``:

.. code-block:: yaml

    external_components:
      - source:
          type: git
          url: https://github.com/nliaudat/esphome_ai_component
          ref: main
        components:
          - meter_reader_tflite
          - value_validator
          - tflite_micro_helper
          - esp32_camera_utils

License
-------

Apache 2.0 OR MIT, at your option.