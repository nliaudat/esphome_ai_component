.. SPDX-License-Identifier: Apache-2.0 OR MIT

tflite_micro_helper
===================

**Status: Active**

The ``tflite_micro_helper`` component is a wrapper for the TensorFlow Lite Micro runtime. It handles model loading, CRC32 verification, tensor arena management, and ESP-NN optimizations.

Configuration
-------------

.. code-block:: yaml

    tflite_micro_helper:
      id: my_tflite

Dependencies
------------

- ``tensorflow/lite/micro`` (ESPHome component)
