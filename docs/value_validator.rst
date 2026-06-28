.. SPDX-License-Identifier: Apache-2.0 OR MIT

value_validator
===============

**Status: Active**

The ``value_validator`` component provides reading validation for meter readers. It eliminates outliers, tracks reading history, prevents impossible value jumps, and supports dial-aware digit correction when combined with ``analog_reader``.

Configuration
-------------

.. code-block:: yaml

    value_validator:
      id: my_validator
      allow_negative_rates: false
      max_absolute_diff: 100
      max_rate_change: 0.15
      enable_smart_validation: true
      smart_validation_window: 5
      high_confidence_threshold: 0.90
      max_history_size: 50kB
      per_digit_confidence_threshold: 0.85
      strict_confidence_check: false
      first_reading_digit_threshold: 0.70
      max_consecutive_rejections: 10
      small_negative_tolerance: 5
      persist_state: false
      enable_dial_correction: true

Configuration Variables
-----------------------

- **id** (*Required*): The ID of this component.
- **allow_negative_rates** (*Optional*): Allow decreasing readings. Default: false.
- **max_absolute_diff** (*Optional*): Maximum absolute difference between consecutive readings. Default: 100.
- **max_rate_change** (*Optional*): Maximum rate of change (0-10). Default: 0.15.
- **max_history_size** (*Optional*): Maximum history buffer size. Default: 50kB.

Dependencies
------------

- None (standalone component)
