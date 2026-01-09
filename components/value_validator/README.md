# Value Validator Component

The `value_validator` component provides robust validation and filtering of meter readings. It eliminates outliers, noise, and impossible jumps by tracking reading history and applying intelligent validation rules.

## Features

- **Rate of Change Validation**: Rejects readings with impossible flow rates using configurable thresholds
- **Negative Rate Detection**: Optional rejection of decreasing values (e.g., for non-reversible meters)
- **Smart Validation**: Analyzes recent reading patterns to adapt validation thresholds
- **Per-Digit Confidence**: Validates individual digit confidence for multi-digit readings
- **History Tracking**: Maintains reading history with configurable memory limits
- **PSRAM Optimized**: Efficiently manages memory for ESP32 devices

## Configuration Parameters

```yaml
value_validator:
  - id: my_validator
    
    # Basic Validation
    allow_negative_rates: false          # Allow values < previous value (default: false)
    max_absolute_diff: 100               # Max allowed absolute change (default: 100)
    max_rate_change: 0.15                # Max rate change as percentage (default: 15%)
    
    # Smart Validation
    enable_smart_validation: true        # Enable adaptive validation (default: true)
    smart_validation_window: 5           # Window size for pattern analysis (default: 5)
    
    # Confidence Thresholds
    high_confidence_threshold: 0.90      # Threshold for high confidence (default: 90%)
    per_digit_confidence_threshold: 0.85 # Min confidence per digit (default: 85%)
    strict_confidence_check: false       # Require all digits meet threshold (default: false)
    
    # Memory Management
    max_history_size: 50kB               # Max history memory usage (default: 50kB)
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

### SSOCR Reader
```yaml
value_validator:
  id: gas_validator
  allow_negative_rates: false
  max_absolute_diff: 50

ssocr_reader:
  validator: gas_validator
  # ... other config
```

### Analog Reader
```yaml
value_validator:
  id: pressure_validator
  allow_negative_rates: true  # Pressure can go up/down
  max_rate_change: 0.25

analog_reader:
  validator: pressure_validator
  # ... other config
```

## Runtime Control

Validation parameters can be adjusted at runtime using Home Assistant services or the provided switches/numbers in component YAML packages.
