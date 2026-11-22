.. SPDX-License-Identifier: Apache-2.0 OR MIT

camera_layer_next
=================

**Status: test overlay for the `next` branch.**

This branch exercises the camera-layer pull requests together:

- `camera_decoder` ([esphome#22767](https://github.com/esphome/esphome/pull/22767))
  adds ``camera/decoder.h`` and ``PIXEL_FORMAT_RGB888`` / ``PIXEL_FORMAT_CBYCRY``.
- `camera_image` ([esphome#22766](https://github.com/esphome/esphome/pull/22766))
  adds a display image platform and ``camera::COMPONENT_REQUESTER``.
- `esp32_camera` window warning
  ([esphome#22768](https://github.com/esphome/esphome/pull/22768)) warns at
  config time when a ``window`` reaches past the ``resolution``.

Why a local ``camera`` component exists
---------------------------------------

Both #22767 and #22766 edit ``esphome/components/camera/camera.h`` from ``dev``,
and each carries only its own addition. Fetched as external components they
override each other, so a build could use ``camera_decoder`` or
``camera_image``, but not both.

``components/camera/`` in this repository is a local **union overlay**: the
``dev`` (2026.12) camera component plus both additions. Fixtures list it in
``external_components`` and take only ``camera_decoder`` / ``camera_image``
from the PR heads, so all of them compile in one build.

This overlay is test scaffolding. Remove it, and the ``camera`` entries from
the fixtures, once the two PRs are merged and ``dev`` carries both changes.

Fixtures
--------

- ``tests/components/camera_image/`` — the ``image:`` platform from #22766,
  including resize, crop, rotation and the ``on_image`` / ``on_error`` hooks.
- ``tests/components/camera_integration/`` — one camera feeding
  ``camera_image`` and both ``*_runner`` readers through a shared
  ``camera_decoder``, which is the coexistence proof.
- ``tests/components/esp32_camera_window/`` — loads ``esp32_camera`` from
  #22768 with a window that reaches past the frame, and checks the warning.
- ``tests/components/analog_gauge_reader_runner/`` and
  ``tests/components/ssocr_reader_runner/`` — the readers on the overlay.

Requester note
--------------

``camera_image`` requests frames with ``COMPONENT_REQUESTER``, so they are
served only to it. The readers still use ``camera::IDLE`` (as the upstream
readers do); consider moving them to ``COMPONENT_REQUESTER`` once the camera
header is unified in ``dev``.
