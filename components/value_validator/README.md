# Value Validator Component

The `value_validator` component provides robust validation and filtering of meter readings. It eliminates outliers, noise, and impossible jumps by tracking reading history and applying intelligent validation rules.

## Features

- **Rate of Change Validation**: Rejects readings with impossible flow rates using configurable thresholds
- **Negative Rate Detection**: Optional rejection of decreasing values (e.g., for non-reversible meters)
- **Smart Validation**: Analyzes recent reading patterns to adapt validation thresholds
- **Self-Correction**: Automatically recovers from stuck readings after consecutive high-confidence rejections
- **Per-Digit Confidence**: Validates individual digit confidence for multi-digit readings
- **Dial-Aware Correction**: Uses analog dial fraction to correct digit boundary transitions when combined with `analog_reader`
- **History Tracking**: Maintains reading history with configurable memory limits
- **PSRAM Optimized**: Efficiently manages memory for ESP32 devices
- **Multiple Instances**: Support for multiple validator configurations for different readers
- **Optional Usage**: Can be omitted if validation is not required
- **Float Output**: When `USE_ANALOG_READER` is enabled, outputs float values combining validated integer digits with dial fractional precision

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
- **validated_value_sensor**: The validated float reading (with dial fraction when analog_reader is active). **Only published on accepted readings** — if the reading is rejected, this sensor's value does not change. This allows comparing the raw `value_sensor` (always updated) against the validated value to debug validation issues.

These enable HA automations (e.g., notify when rejection count exceeds a threshold) and make debugging stuck episodes possible without reading ESP logs.

### Publishing Behavior

When the value_validator is configured together with `meter_reader_tflite`:

| Scenario | `value_sensor` (meter_reader) | `validated_value_sensor` (validator) |
|----------|-------------------------------|---------------------------------------|
| **No validator** | Published on valid + high confidence only | N/A |
| **Validator + valid** | Published always (raw digits) | Published (validated float) |
| **Validator + rejected** | Published always (raw digits) | **Frozen** (last valid value) |

This makes it easy to spot when the validator is filtering readings: the raw sensor shows every attempt, while the validated sensor only updates on accepted values.

## Persistent State

When `persist_state: true`, the last valid reading is saved to flash memory. After a reboot, the validator resumes from the saved value instead of waiting for a new first reading. This prevents the risk of blindly accepting a bad first reading after power cycles.

## Dial-Aware Correction (Analog Reader Integration)

When used together with the `analog_reader` component, the value_validator can correct digit boundary transitions using the analog dial fraction. This solves the problem where TFLite digit recognition may misread a digit that is physically transitioning between two values on the meter.

### How It Works

1. **analog_reader** reads multiple dials (e.g. scale=1.0, scale=0.1, scale=0.01) and sums them: `summed_dial_value = Σ(dial_val[i] × scale[i])`
2. The **fractional part** of the summed value (`fmodf(summed_dial_value, 1.0)`) is fed to the validator
3. **meter_reader_tflite** reads the digit wheels and calls `validate_reading()` with the float overload
4. The validator applies correction:
   - **If fraction > high_threshold (default 0.80)**: The digit wheel is about to turn → subtract 1 from the integer (e.g. 210600 → 210599)
   - **If fraction < low_threshold (default 0.20)**: The digit wheel is solid → keep the integer as-is
   - **Middle zone (0.20–0.80)**: No correction, rely on confidence-based validation
5. **Final output** = `corrected_integer + dial_fraction` (e.g. `210599.847`)

### Configuration

```yaml
value_validator:
  id: combined_validator
  enable_dial_correction: true           # Enable correction (default: true)
  dial_correction_high_threshold: 0.80   # Above this → subtract 1 (default: 0.80)
  dial_correction_low_threshold: 0.20    # Below this → keep as-is (default: 0.20)

meter_reader_tflite:
  validator: combined_validator
  # ... digit zones ...

analog_reader:
  validator: combined_validator          # Same validator instance!
  dials:
    - id: dial_1
      scale: 1.0                         # Units digit
    - id: dial_2
      scale: 0.1                         # Tenths digit
    - id: dial_3
      scale: 0.01                        # Hundredths digit
```

> **Important**: Both `meter_reader_tflite` and `analog_reader` must reference the **same** `value_validator` instance. The analog_reader feeds the dial fraction, and the meter_reader_tflite reads digits — the validator combines both into a single validated float.

### Guard Macros

All dial-aware correction logic is guarded by `#ifdef USE_ANALOG_READER`. When `analog_reader` is not included in the project, the validator behaves exactly as before (integer output only). No code size or performance impact.

## Runtime Control

Validation parameters can be adjusted at runtime using Home Assistant services or the provided switches/numbers in component YAML packages.
**Note**: The controls package (`value_validator_controls.yaml`) currently defaults to controlling the validator with ID `${id_prefix}_validator`.
