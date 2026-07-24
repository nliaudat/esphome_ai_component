.. SPDX-License-Identifier: Apache-2.0 OR MIT

tflite_micro_helper
===================

**Status: Active**

The ``tflite_micro_helper`` component is a self-contained TFLite Micro model loader
and runtime wrapper. It handles model source resolution (local files, ``github://``
shorthand, ``http(s)://`` URLs), model config auto-detection from ``.txt`` files,
CRC32 verification, tensor arena management, PROGMEM array creation, and
ESP-NN optimizations.

Supports two model types:

- **image**: Single-shot inference models (4D tensors ``[1, H, W, C]``)
- **audio**: Streaming inference models (3D tensors ``[1, stride, features]``)
  with ``MicroResourceVariables`` for persistent state

Configuration
-------------

.. code-block:: yaml

    # Image model (single-shot inference)
    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
        # Optional overrides (auto-detected from .txt file):
        # input_width: 32
        # input_height: 20
        # input_channels: 3
        # tensor_arena_size: "110KB"

    # Audio model (streaming inference)
      - id: my_wake_word
        model_type: audio
        model: "models/wake_word_model.tflite"
        probability_cutoff: 0.7
        sliding_window_size: 10

    # Remote model via github:// shorthand
      - id: remote_model
        model_type: image
        model: "github://owner/repo/path/to/model.tflite"

Model Sources
-------------

- **Local**: Direct path to a ``.tflite`` file. Auto-detects config from companion ``.txt`` file.
- **github://**: ``github://owner/repo/path/to/file[@ref]`` shorthand.
- **http(s)://**: Download from URL (expects JSON manifest with ``model`` field).

Auto-Detected Configuration (Image Models)
------------------------------------------

When a ``.txt`` file exists alongside the ``.tflite`` file, the following are
auto-detected:

- Input shape (width, height, channels) and data type
- Output class count and scale factor
- Tensor arena size from peak analysis
- Output processing method (``direct_class`` if SOFTMAX present, else ``softmax``)
- Hybrid quantization detection (rejected with clear error)

Configuration Variables
-----------------------

**Common:**
- **id** (*Required*, :ref:`config-id`): The ID of this component.
- **model_type** (*Required*): ``"image"`` or ``"audio"``.
- **model** (*Required*): Model source path or URL.
- **tensor_arena_size** (*Optional*): Override tensor arena size (e.g. ``"110KB"``).
- **debug** (*Optional*, boolean): Enable debug logging.

**Image model overrides (optional, override auto-detection):**
- **input_type**: ``"uint8"`` (default) or ``"float32"``.
- **input_channels**: 1-4 (default: 3).
- **input_width**: 8-512 (default: 32).
- **input_height**: 8-512 (default: 20).
- **output_processing**: ``"direct_class"``, ``"softmax"``, etc.
- **scale_factor**: 0.1-100.0 (default: 1.0).
- **input_order**: ``"RGB"``, ``"BGR"``, ``"GRAY"``.
- **normalize**: Boolean.
- **invert**: Boolean.

**Audio model config (optional, override auto-detection):**
- **probability_cutoff**: 0.0-1.0.
- **sliding_window_size**: Positive integer.
- **features_step_size**: 0-30 (ms).
- **feature_count**: 10-80.

Dependencies
------------

- ``tensorflow/lite/micro`` (ESPHome component)
- ``espressif/esp-tflite-micro`` (IDF component)
- ``espressif/esp-nn`` (IDF component)
- ``esphome/esp-micro-speech-features`` (IDF, audio models only)
