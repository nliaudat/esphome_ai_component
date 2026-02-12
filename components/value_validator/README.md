# Value Validator Component

The `value_validator` component provides robust validation and filtering of meter readings. It eliminates outliers, noise, and impossible jumps by tracking reading history and applying intelligent validation rules.

## Features

- **Rate of Change Validation**: Rejects readings with impossible flow rates using configurable thresholds
- **Negative Rate Detection**: Optional rejection of decreasing values (e.g., for non-reversible meters)
- **Smart Validation**: Analyzes recent reading patterns to adapt validation thresholds
- **Self-Correction**: Automatically recovers from stuck readings after consecutive high-confidence rejections
- **Per-Digit Confidence**: Validates individual digit confidence for multi-digit readings
- **History Tracking**: Maintains reading history with configurable memory limits
- **PSRAM Optimized**: Efficiently manages memory for ESP32 devices
- **Multiple Instances**: Support for multiple validator configurations for different readers
- **Optional Usage**: Can be omitted if validation is not required

## Configuration Parameters

You can define a single validator or a list of validators.

### Single Validator
```yaml
value_validator:
  id: my_validator
  allow_negative_rates: false
  debug: false
  # ... other params
```

### Multiple Validators
```yaml
value_validator:
  - id: water_validator
    allow_negative_rates: false
    strict_confidence_check: true

  - id: pressure_validator
    allow_negative_rates: true
    max_rate_change: 0.25
    debug: true
```

## Parameter Details

### Basic Validation

- **allow_negative_rates**: Set to `true` for bidirectional meters (e.g., solar with grid feedback)
- **max_absolute_diff**: Absolute maximum change allowed between consecutive readings
- **max_rate_change**: Percentage-based rate limit (e.g., 0.15 = 15% change maximum)
- **small_negative_tolerance** *(default: 5)*: Allow negative changes up to this many units without triggering rejection. Applies both in `is_digit_plausible` and the consistency check.

### Smart Validation

- **enable_smart_validation**: Analyzes recent patterns to adapt thresholds dynamically
- **smart_validation_window**: Number of recent readings to analyze for patterns

### Self-Correction

- **max_consecutive_rejections** *(default: 10)*: Number of consecutive high-confidence rejections before the validator self-corrects. When the validator gets stuck on an incorrect reading (e.g., due to a misread digit), it counts rejected readings. After this many rejections with average confidence ≥ `high_confidence_threshold`, the most frequent recent value is accepted as the corrected reading — even if it would normally be blocked by `allow_negative_rates: false`.

### Confidence Settings

- **high_confidence_threshold**: Readings above this are trusted more in validation. Also used as the minimum average confidence required for self-correction.
- **per_digit_confidence_threshold**: Minimum confidence required for each digit
- **strict_confidence_check**: When `true`, ALL digits must meet the confidence threshold

### Memory Management

- **max_history_size**: Limits memory usage for history tracking (accepts: B, kB, MB)

### Persistent State

- **persist_state** *(default: false)*: When enabled, saves `last_valid_reading` to flash across reboots. Prevents the validator from starting fresh after power cycles, avoiding the risk of accepting a bad first reading.

## Integration Examples

### Meter Reader TFLite
```yaml
value_validator:
  id: water_validator
  allow_negative_rates: false
  max_absolute_diff: 300
  strict_confidence_check: true
  per_digit_confidence_threshold: 0.95
  high_confidence_threshold: 0.95
  max_consecutive_rejections: 10
  small_negative_tolerance: 5
  persist_state: true
  # Optional diagnostic sensors
  rejection_count_sensor:
    name: "Validator Rejection Count"
  raw_reading_sensor:
    name: "Validator Raw Reading"
  validator_state_sensor:
    name: "Validator State"

meter_reader_tflite:
  validator: water_validator
  # ... other config
```

### SSOCR Reader (Optional Validator)
The validator is optional provided the component supports it (e.g., `ssocr_reader`, `analog_reader`).

```yaml
ssocr_reader:
  # validator: my_validator # Optional: Comment out if not needed
  # ... other config
```

### Multiple Instance Example
```yaml
value_validator:
  - id: digital_validator
    strict_confidence_check: true
  - id: analog_validator
    allow_negative_rates: true

meter_reader_tflite:
  validator: digital_validator
  # ...

analog_reader:
  validator: analog_validator
  # ...
```

## Self-Correction Behavior

When the validator accepts an incorrect high-confidence reading (e.g., a misread digit), it can become "stuck" — all subsequent correct readings are rejected because they appear as negative rates. The self-correction mechanism detects this by tracking consecutive rejections:

1. Each rejected reading increments a counter and accumulates the confidence score
2. After `max_consecutive_rejections` (default: 10) rejections:
   - If average confidence ≥ `high_confidence_threshold`: the **most frequent** value from recent history is accepted as the corrected reading
   - This works even when OCR produces fluctuating values (e.g., alternating between 256517 and 256617)
3. The counter resets when a reading is accepted, or when the validator is reset/manually set

With a ~1 minute reading interval, self-correction triggers within ~10 minutes by default.

## Diagnostic Sensors

Optional sensors that expose the validator's internal state to Home Assistant:

- **rejection_count_sensor**: Current count of consecutive rejected readings (0 during normal operation)
- **raw_reading_sensor**: The raw value from the model before any validation
- **validator_state_sensor**: Text state — one of `initializing`, `normal`, `rejecting`, `stuck`

These enable HA automations (e.g., notify when rejection count exceeds a threshold) and make debugging stuck episodes possible without reading ESP logs.

## Persistent State

When `persist_state: true`, the last valid reading is saved to flash memory. After a reboot, the validator resumes from the saved value instead of waiting for a new first reading. This prevents the risk of blindly accepting a bad first reading after power cycles.

## Runtime Control

Validation parameters can be adjusted at runtime using Home Assistant services or the provided switches/numbers in component YAML packages.
**Note**: The controls package (`value_validator_controls.yaml`) currently defaults to controlling the validator with ID `${id_prefix}_validator`.
