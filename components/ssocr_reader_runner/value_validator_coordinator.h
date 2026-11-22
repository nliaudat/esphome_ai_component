#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SSOCR_READER_RUNNER

#include "esphome/core/component.h"

#ifdef USE_VALUE_VALIDATOR
#include "esphome/components/value_validator/value_validator.h"
#endif

#include <vector>

namespace esphome {
namespace ssocr_reader_runner {

/// Optional bridge to the local `value_validator` component.
///
/// The validator corrects digit-boundary transitions in `meter_reader_tflite`
/// using the fractional part of this reader's summed dial value, so the reader
/// only has to feed that fraction once per processed frame.
class ValueValidatorCoordinator {
 public:
#ifdef USE_VALUE_VALIDATOR
  void set_validator(value_validator::ValueValidator *validator) { validator_ = validator; }

  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    if (validator_ == nullptr) {
      validated_value = raw_value;
      return true;  // Assume valid if no validator set, or handle as policy
    }
    return validator_->validate_reading(raw_value, confidence, validated_value);
  }

  bool validate_reading(const std::vector<float> &digits, const std::vector<float> &confidences, int &validated_value) {
    if (validator_ == nullptr) {
      // Safe default: fail validation if component is missing to avoid bad data
      return false;
    }
    return validator_->validate_reading(digits, confidences, validated_value);
  }

  void set_last_valid_reading(int value) {
    if (validator_ != nullptr) {
      validator_->set_last_valid_reading(value);
    }
  }

 protected:
  value_validator::ValueValidator *validator_{nullptr};
#else
  // Dummy implementation
  void set_validator(void *validator) {}

  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    validated_value = raw_value;
    return true;
  }

  bool validate_reading(const std::vector<float> &digits, const std::vector<float> &confidences, int &validated_value) {
    return false;
  }

  void set_last_valid_reading(int value) {}
#endif
};

}  // namespace ssocr_reader_runner
}  // namespace esphome

#endif  // USE_SSOCR_READER_RUNNER
