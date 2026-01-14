# Value Validator Component

The `value_validator` component provides robust validation and filtering of meter readings. It eliminates outliers, noise, and impossible jumps by tracking reading history and applying intelligent validation rules.

## Features

- **Rate of Change Validation**: Rejects readings with impossible flow rates using configurable thresholds
- **Negative Rate Detection**: Optional rejection of decreasing values (e.g., for non-reversible meters)
- **Smart Validation**: Analyzes recent reading patterns to adapt validation thresholds
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

### Smart Validation

- **enable_smart_validation**: Analyzes recent patterns to adapt thresholds dynamically
- **smart_validation_window**: Number of recent readings to analyze for patterns

### Confidence Settings

- **high_confidence_threshold**: Readings above this are trusted more in validation
- **per_digit_confidence_threshold**: Minimum confidence required for each digit
- **strict_confidence_check**: When `true`, ALL digits must meet the confidence threshold

### Memory Management

- **max_history_size**: Limits memory usage for history tracking (accepts: B, kB, MB)

## Integration Examples

### Meter Reader TFLite
```yaml
value_validator:
  id: water_validator
  allow_negative_rates: false
  max_absolute_diff: 300
  strict_confidence_check: true
  per_digit_confidence_threshold: 0.95

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

## Runtime Control

Validation parameters can be adjusted at runtime using Home Assistant services or the provided switches/numbers in component YAML packages.
**Note**: The controls package (`value_validator_controls.yaml`) currently defaults to controlling the validator with ID `${id_prefix}_validator`.
