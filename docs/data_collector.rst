.. SPDX-License-Identifier: Apache-2.0 OR MIT

data_collector
=============

**Status: Active**

The ``data_collector`` component implements Active Learning by automatically collecting low-confidence inference images and uploading them to a configurable server for training set improvement.

Configuration
-------------

.. code-block:: yaml

    data_collector:
      id: my_collector
      upload_url: "http://192.168.1.50:5123/api/upload/my_meter"
      api_key: "change-me-to-a-secure-key"

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **upload_url** (*Required*): HTTP endpoint for image upload.
- **api_key** (*Optional*): Authentication key for the upload server.

Dependencies
------------

- HTTP client (ESPHome built-in)
