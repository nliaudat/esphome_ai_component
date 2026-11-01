.. SPDX-License-Identifier: Apache-2.0 OR MIT

ESP-NN S3 Optimized Kernels Issue
==================================

**Status:** Known limitation on some ESP32-S3 boards; workaround available.

This page documents a known issue where certain ESP32-S3 boards return the same digit class for every image when using optimized ESP-NN kernels with quantized uint8 digit recognition models. The fix is to force ANSI C kernels in the board configuration.

Symptoms
--------

When running ``meter_reader_tflite`` on an ESP32-S3 with a quantized uint8 model (e.g., ``digit_recognizer_v40_quantized_integer_quant_uint8.tflite`` or v16), you may observe:

- The same digit class returned for every image, regardless of the actual meter reading (e.g., always ``88888`` or ``00000``).
- Identical confidence scores across multiple inferences.
- Analog dials from the same frame read correctly, confirming the input is fine but the model output is constant.

This was reported in :ghissue:`196` on a generic ESP32-S3 N16R8 board with OV2640 and octal PSRAM.

Cause
-----

The issue stems from bugs in certain versions of the ``esp-nn`` library used by ESPHome's ``tflite_micro_helper`` component:

- Versions shipped with ESPHome 2026.10.x pin ``esp-tflite-micro 1.3.7`` and ``esp-nn 1.2.3``.
- These versions contain known S3 assembly bugs in convolution, softmax, and other kernels that can cause incorrect or constant outputs on some boards (especially those with specific PSRAM configurations).
- Upgrading to esp-nn 1.4.1 fixed some issues but not all; esp-nn 1.4.2 addressed further problems but the issue persists on some boards with certain models and configurations.

The problem is board- and model-specific, which is why it has not been possible to create a universal fix. The safest approach for affected boards is to disable the optimized kernels entirely.

Workaround
----------

Force the use of ANSI C kernels by adding these options to your ESP32-S3 board file under ``esp32.framework.sdkconfig_options``:

.. code-block:: yaml

    esp32:
      framework:
        sdkconfig_options:
          CONFIG_NN_ANSI_C: "y"
          CONFIG_NN_OPTIMIZED: "n"

These options should be placed after the line setting ``CONFIG_SPIRAM_SPEED_80M`` (or the last SPIRAM option) in your board file. For example, in ``boards/board_generic_esp32-s3-n16r8.yaml``:

.. code-block:: yaml

    # ...
      CONFIG_SPIRAM_SPEED_80M: "y"
      # ESP-NN: S3 optimized kernels return a constant class with some quantized uint8 digit models.
      # ANSI C kernels are slower but correct on all tested boards.
      CONFIG_NN_ANSI_C: "y"
      CONFIG_NN_OPTIMIZED: "n"
    # ...

Inference time with ANSI C kernels is about **300 ms for 5 digits** on the generic N16R8 board, which is acceptable for meter readings taken every few minutes. The ``-DESP_NN`` flag in board files remains (it controls esp-nn inclusion, not kernel choice).

Affected Boards
---------------

The issue has been observed on:

- Generic ESP32-S3 N16R8 camera boards with octal PSRAM and OV2640.

It may occur on other S3 boards depending on their PSRAM configuration, silicon revision, or model usage. If you experience constant outputs from ``meter_reader_tflite`` that do not match your meter's dial readings, try forcing ANSI C kernels as described above.

Current Status
--------------

- **esp-nn 1.4.2** (shipped in ESPHome 2026.11.x-dev) includes further fixes but the issue has not been confirmed resolved on every S3 board or model combination.
- The maintainer recommends keeping ``CONFIG_NN_ANSI_C: "y"`` / ``CONFIG_NN_OPTIMIZED: "n"`` in ESP32-S3 board files until a universal fix is verified across all configurations.

Related
-------

- :ghissue:`196` — Original report and discussion.
- :pr:`201` — Added ANSI C options to four S3 board files.
- :pr:`207` — Fixed value_validator persistence issues (unrelated but relevant to S3 deployments).
- esp-nn issue #34 / PR #43 — Upstream fixes in progress.
- esp-nn release v1.4.2 — Latest stable version with multiple S3 kernel improvements.
