.. SPDX-License-Identifier: Apache-2.0 OR MIT

tflite_micro_runtime
====================

**Status: Active** — split-component series (upstream PRs
:pr:`21282`–:pr:`21288`).

The ``tflite_micro_runtime`` component owns everything about a TFLite Micro
model: source resolution, loading, CRC32 verification, the tensor arena, the
operator resolver, the interpreter, input preprocessing and output
post-processing. It supports two model types:

- **image**: single-shot inference models (4D input tensor ``[1, H, W, C]``)
- **audio**: streaming inference models (3D input tensor ``[1, frames, features]``)

The component is ``MULTI_CONF``: declare one entry per model and reference it by
ID from a consumer such as ``meter_reader_tflite``.

The runtime is split into seven cooperating components. Only
``tflite_micro_runtime`` is configured by the user; the other six are loaded
automatically:

.. list-table::
    :header-rows: 1

    * - Component
      - Role
    * - ``tflite_micro_runtime``
      - Runtime entry point: arena, interpreter, preprocessing, post-processing
    * - ``tflite_model_source``
      - Model source resolution (local, ``github://``, ``http(s)://``), metadata parsing
    * - ``tflite_op_processor``
      - Operator table, resolver sizing, ESP-NN kernel registration
    * - ``tflite_memory_handler``
      - PSRAM-aware tensor arena allocator
    * - ``tflite_output_processor``
      - Output decoding (argmax, softmax, QAT quantized, auto-detect)
    * - ``tflite_model_loader``
      - Loader, CRC32 verification and ``ModelHandler`` facade
    * - ``tflite_model_debug``
      - Optional verbose model diagnostics, enabled by ``debug: true``

Requirements
------------

- ESPHome **2026.11** or later.
- ESP32 target (the component is ``only_on_esp32``).
- The model buffer is emitted with 16-byte alignment. ESPHome 2026.12 uses the
  ``align`` parameter of ``progmem_array()`` (upstream :pr:`22620`) for this; on
  2026.11 the component emits the equivalent ``alignas(16)`` declaration.

Installation
------------

One ``source`` carries a single ref, so the seven components are listed one per
PR head while the series is unmerged:

.. code-block:: yaml

    external_components:
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

``tflite_micro_runtime`` auto-loads the other six, so only the last entry is
needed once the components ship inside ESPHome.

Configuration
-------------

.. code-block:: yaml

    # One entry per model (MULTI_CONF); an `id:` is required on each.
    tflite_micro_runtime:
      # Image model (single-shot inference)
      - id: my_digit_model
        model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
        debug: false

      # Audio model (streaming inference)
      - id: my_wake_word
        model: "github://esphome/micro-wake-word-models/models/v2/okay_nabu.json"
        probability_cutoff: 0.7
        sliding_window_size: 10

    # Consumer: reference the runtime entry by ID
    meter_reader_tflite:
      id: my_meter_reader
      model: my_digit_model
      camera_id: my_camera

Model sources
-------------

The ``model:`` option accepts a local path, a git shorthand or an HTTP URL, and
may also be written as a mapping:

.. list-table::
    :header-rows: 1

    * - Source
      - Example
    * - Local file
      - ``model: "models/digit_recognizer.tflite"``
    * - Git shorthand
      - ``model: "github://owner/repo/path/model.tflite[@ref]"``
        (``gitlab://`` and ``codeberg://`` work the same way)
    * - HTTP(S) model file
      - ``model: "https://example.com/models/digit.tflite"``
    * - HTTP(S) JSON manifest
      - ``model: "https://example.com/models/digit.json"`` — any URL that does
        not end in ``.tflite`` is read as a JSON object with a ``model`` field

A companion ``.txt`` report is **optional**. The input shape, data type,
channel count, operator list and an arena estimate are read from the ``.tflite``
file itself, so a model works without any report. When one does sit next to the
model (or next to the downloaded model) it is used to fill in values the model
cannot supply, such as ``output_processing``, ``scale_factor`` or a hand-tuned
``tensor_arena_size``. For HTTP sources the report URL is derived by replacing
``.tflite`` with ``.txt``.

Configuration variables
-----------------------

**Common**

- **id** (*Required*): ID of this runtime entry.
- **model** (*Required*): Model source (see `Model sources`_).
- **tensor_arena_size** (*Optional*): Tensor arena size, e.g. ``"110KB"``.
  Range 8 KB–2000 KB. For image models it defaults to an estimate computed from
  the model's peak activations; for audio models it comes from the optional
  ``.txt`` report or the C++ defaults.
- **debug** (*Optional*, boolean): Enable verbose model logging. Defaults to
  ``false``.

.. note::

    There are two ways to declare a model, and no ``model_type`` option:

    * **Kind entry (recommended for new work).** The kind component is the
      configuration key: ``tflite_image``, ``tflite_detection``,
      ``tflite_segmentation``, ``tflite_pose``, ``tflite_tabular``,
      ``tflite_text``, ``tflite_audio`` or ``tflite_camera``. It auto-loads
      ``tflite_micro_runtime`` in the background, calls ``declare_model()`` and
      attaches its input pipeline and output decoder. Do not add a
      ``tflite_micro_runtime`` entry for the same model.
    * ``tflite_camera`` is the camera variant: it listens to an
      ``esp32_camera``, decodes each JPEG through ``camera_decoder``, scales the
      whole frame into the model input and fires ``on_result``. It runs one
      model on one frame; it is not a multi-crop reader. For several dials out
      of one frame, use ``meter_reader_tflite`` with zone crops.
    * **Plain runtime entry.** ``tflite_micro_runtime`` on its own stays valid
      for a consumer that fills the input tensor itself: this repository's
      ``meter_reader_tflite`` and the upstream ``micro_wake_word``. The
      consumer reads ``input_tensor()`` / ``output_tensor()`` or calls
      ``run_inference_on_buffer()``.

.. code-block:: yaml

    # Kind entry: the input pipeline and output decoder come with it.
    tflite_image:
      - id: my_digit_model
        model: "models/digit_recognizer_v40_quantized_integer_quant_uint8.tflite"
        input_order: RGB
        normalize: false

**Image models**

These keys belong to the ``tflite_image`` kind entry, not to the runtime.

- **input_type** (*Optional*): ``uint8`` or ``float32``. Taken from the model's
  input tensor.
- **input_channels** (*Optional*): 1–4.
- **input_width** / **input_height** (*Optional*): 8–512.
- **input_order** (*Optional*): ``RGB``, ``BGR`` or ``GRAY``. Forced to
  ``GRAY`` when the model has a single input channel.
- **output_processing** (*Optional*): ``argmax`` (default), ``softmax``,
  ``qat_quantized`` or ``auto_detect``. The legacy names ``direct_class``
  (→ ``argmax``), ``logits``, ``logits_jomjol``, ``softmax_jomjol`` and
  ``experimental_scale`` (→ ``softmax``) are still accepted.
- **scale_factor** (*Optional*): 0.1–100.0. Defaults to ``1.0``.
- **normalize** (*Optional*, boolean): Normalize pixel values.
- **invert** (*Optional*, boolean): Invert pixel values.

**Audio models**

- **probability_cutoff** (*Optional*): Detection threshold, 0–100 %.
- **sliding_window_size** (*Optional*): Positive integer.
- **features_step_size** (*Optional*): 0–30.
- **feature_count** (*Optional*): 10–80. Taken from the model when omitted.

Auto-detection
--------------

Values are resolved in this order (later wins):

1. Filename conventions (only for image models without a ``.txt`` report).
2. The optional ``.txt`` report, when present and not stale. It only adds values
   the model cannot provide (``output_processing``, ``scale_factor``,
   ``normalize``, ``invert``, an explicit arena size, audio parameters).
3. The model's own input tensor, which is authoritative for the input shape,
   data type and channel count (and for ``feature_count`` on audio models).
4. The YAML values above.

No ``.txt`` report is required — without one, everything that matters for
loading and inference comes from the model itself.

An image model whose input is not ``[1, H, W, C]``, an audio model whose input
is not ``[1, frames, features]``, or a model that needs an operator missing from
esp-tflite-micro 1.4.1 is rejected during validation, with the offending
operator names listed.

Model caching
-------------

- ``github://`` sources are cloned into the ESPHome external-components cache
  and re-fetched when their ``refresh`` interval expires.
- HTTP sources are downloaded into the shared external-files cache; a ``304``
  revalidation keeps both the model and its matching ``.txt`` report.
- The model is verified against the CRC32 computed at build time before the
  interpreter is created.

Dependencies
------------

- ``esp32`` (required)
- ``tflite_model_source`` (required; the other five are auto-loaded)
- ``espressif/esp-tflite-micro`` 1.4.1 (IDF component)
- ``espressif/esp-nn`` 1.5.0 (IDF component)
- ``esphome/esp-micro-speech-features`` 1.2.3 (audio models only)

Migration from ``tflite_micro_helper``
--------------------------------------

The monolithic ``tflite_micro_helper`` was replaced by this split series. The
consumer syntax is unchanged: it still references a model entry by ID.

.. code-block:: yaml

    # Before
    tflite_micro_helper:
      - id: my_digit_model
        model_type: image
        model: "digit_recognizer.tflite"

    # After
    tflite_micro_runtime:
      - id: my_digit_model
        model: "digit_recognizer.tflite"

``tflite_micro_helper`` was removed from this repository in the split-component
migration: image models use :doc:`tflite_micro_runtime` and audio models use the
upstream ``micro_wake_word`` component, which manages its own interpreter. The
old page in :doc:`tflite_micro_helper` keeps the former configuration and the
migration examples.
