.. SPDX-License-Identifier: Apache-2.0 OR MIT

esp32_camera_utils
==================

**Status: Active**

The ``esp32_camera_utils`` component provides image processing utilities including cropping, scaling, rotation (JPEG and Raw formats), and format conversion using the ``esp_new_jpeg`` library.

Configuration
-------------

.. code-block:: yaml

    esp32_camera_utils:
      id: my_camera_utils
      camera_id: my_camera
      rotation: 0.0
      enable_rotation: false

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **camera_id** (*Required*): The ID of the ``esp32_camera`` component.
- **rotation** (*Optional*): Image rotation in degrees. Default: 0.0.
- **enable_rotation** (*Optional*): Enable software rotation. Default: false.

Dependencies
------------

- ``esp32_camera`` (required)
- ``espressif/esp_new_jpeg`` library (automatically added)