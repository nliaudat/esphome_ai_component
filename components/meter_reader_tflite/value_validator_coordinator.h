#pragma once

#include "esphome/core/component.h"
#include "esphome/components/value_validator/value_validator.h"
#include <limits>

namespace esphome {
namespace meter_reader_tflite {

class ValueValidatorCoordinator {
 public:
  void set_validator(value_validator::ValueValidator *validator) { validator_ = validator; }
  
  bool has_validator() const { return validator_ != nullptr; }

  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    if (validator_ == nullptr) {
      validated_value = raw_value;
      return true; 
    }
    return validator_->validate_reading(raw_value, confidence, validated_value);
  }

  bool validate_reading(const std::vector<float> &digits, const std::vector<float> &confidences, int &validated_value) {
    if (validator_ == nullptr) {
        // Fallback: No validator = No strict per-digit check here.
        // We need to convert digits to value manually as fallback.
        if (digits.empty()) {
            return false;
        }
        long long val = 0;
        for (float d : digits) {
            int digit = static_cast<int>(round(d));
            // Ensure digit is in the range [0, 9]
            if (digit < 0 || digit >= 10) {
                digit = 0;
            }
            val = val * 10 + digit;
        }
        
        if (val > std::numeric_limits<int>::max()) {
            validated_value = std::numeric_limits<int>::max();
        } else {
            validated_value = static_cast<int>(val);
        }
        return true; 
    }
    return validator_->validate_reading(digits, confidences, validated_value);
  }

  void set_last_valid_reading(int value) {
    if (validator_ != nullptr) {
      validator_->set_last_valid_reading(value);
    }
  }

  void set_last_valid_reading(const std::string &value) {
    if (validator_ != nullptr) {
      validator_->set_last_valid_reading(value);
    }
  }

 protected:
  value_validator::ValueValidator *validator_{nullptr};
};

}  // namespace meter_reader_tflite
}  // namespace esphome
