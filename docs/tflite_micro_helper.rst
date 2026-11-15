.. SPDX-License-Identifier: Apache-2.0 OR MIT

tflite_micro_helper (removed)
=============================

**Status: Removed.** The component no longer exists in this repository (the
``components/tflite_micro_helper`` directory was deleted). This page keeps its
documentation for existing configurations and links the migration path.

For the replacement, see :doc:`tflite_micro_runtime`. Once the split components
are merged, their reference documentation lives on `esphome.io
<https://esphome.io/components/>`_, and ``micro_wake_word`` is documented there
as a standard ESPHome component.

What replaced it
----------------

.. list-table::
    :header-rows: 1

    * - Was
      - Now
    * - ``tflite_micro_helper:`` for image models
      - :doc:`tflite_micro_runtime` (one entry per model, referenced by ID from
        ``meter_reader_tflite: model:``)
    * - ``tflite_micro_helper:`` for audio models with ``micro_wake_word``
      - Upstream ESPHome ``micro_wake_word`` (self-contained: model manifests,
        arena, op resolver, streaming inference)
    * - Model loading, CRC32, arena, resolver, output processing
      - ``tflite_micro_runtime`` + its auto-loaded split components
        (``tflite_model_source``, ``tflite_model_loader``,
        ``tflite_memory_handler``, ``tflite_op_processor``,
        ``tflite_output_processor``, ``tflite_model_debug``)

The original README is preserved in git history (last revision:
``git show 7cc6c1a:components/tflite_micro_helper/README.md``).

What it did
-----------

``tflite_micro_helper`` was the monolithic TFLite Micro model loader and runtime
wrapper used before the split-component series. It handled:

- model source resolution: local files, ``github://`` shorthand, ``http(s)://``
  URLs (JSON manifests),
- model metadata auto-detection from a companion ``.txt`` report (optional) or
  filename conventions,
- CRC32 verification of the model data,
- tensor arena management (PSRAM-aware) and PROGMEM model arrays,
- operator table registration and ESP-NN hardware acceleration,
- two model types:

.. list-table::
    :header-rows: 1

    * - Type
      - Shape
      - Use case
      - Inference mode
    * - **image**
      - ``[1, H, W, C]`` (4D)
      - Digit recognition, classification, object detection
      - Single-shot per image
    * - **audio**
      - ``[1, stride, features]`` (3D)
      - Wake word detection, VAD
      - Streaming with ``MicroResourceVariables``

Former configuration
--------------------

.. code-block:: yaml

    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "models/digit_recognizer_v40.tflite"
        # image overrides (auto-detected from the model/.txt report):
        # input_type: "uint8"
        # input_channels: 3
        # input_width: 32
        # input_height: 20
        # tensor_arena_size: "110KB"

      - id: my_wake_word
        model_type: audio
        model: "github://esphome/micro-wake-word-models/models/v2/okay_nabu.json"
        probability_cutoff: 0.7
        sliding_window_size: 10

Consumer components referenced a model by ID: ``meter_reader_tflite: model:
my_digit_model`` and ``micro_wake_word: models: - tflite_model: my_wake_word``.

Configuration variables (former)
--------------------------------

**Common:** ``id`` (required), ``model_type`` (``image``/``audio``, required),
``model`` (required: path, git shorthand or URL), ``tensor_arena_size``
(optional, e.g. ``"110KB"``), ``debug`` (optional).

**Image overrides:** ``input_type`` (``uint8``/``float32``), ``input_channels``
(1-4), ``input_width``/``input_height`` (8-512), ``output_processing``,
``scale_factor`` (0.1-100.0), ``input_order`` (``RGB``/``BGR``/``GRAY``),
``normalize``, ``invert``.

**Audio overrides:** ``probability_cutoff`` (0-1), ``sliding_window_size``,
``features_step_size`` (0-30 ms), ``feature_count`` (10-80).

Model sources
-------------

- **Local**: path to a ``.tflite`` file; a companion ``.txt`` report is
  optional.
- **git**: ``github://owner/repo/path/file.tflite[@ref]`` (``gitlab://`` and
  ``codeberg://`` work the same way).
- **HTTP(S)**: a ``.tflite`` URL, or a JSON manifest URL with a ``model``
  field pointing at the model file.

Migration examples
------------------

Image model, before::

    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "digit_recognizer.tflite"

    meter_reader_tflite:
      model: my_digit_model

after::

    tflite_micro_runtime:
      id: my_digit_model
      model_type: image
      model: "digit_recognizer.tflite"

    meter_reader_tflite:
      model: my_digit_model

Audio model, before::

    tflite_micro_helper:
      - id: my_wake_word
        model_type: audio
        model: okay_nabu

    micro_wake_word:
      models:
        - tflite_model: my_wake_word

after (upstream ``micro_wake_word``; the model carries its own metadata)::

    micro_wake_word:
      models:
        - model: okay_nabu

C++ integration (former)
------------------------

Components that wanted the helper used it as an externally owned runtime::

    #include "esphome/components/tflite_micro_helper/tflite_micro_helper.h"

    void set_tflite(tflite_micro_helper::TFLiteMicroHelper *tflite) {
      this->tflite_ = tflite;
    }

    tflite_micro_helper::TFLiteMicroHelper *tflite_;

The equivalent type is now
``tflite_micro_runtime::TFLiteMicroHelper`` (see
``esphome/components/tflite_micro_runtime/tflite_micro_runtime.h``), and audio
consumers such as ``micro_wake_word`` own their streaming interpreter directly.

Dependencies (former)
---------------------

- ``espressif/esp-tflite-micro`` 1.4.1
- ``espressif/esp-nn`` 1.4.2 (the split series and upstream ``micro_wake_word``
  now pin 1.5.0)
- ``esphome/esp-micro-speech-features`` 1.2.3 (audio models only)
