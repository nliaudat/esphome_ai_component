#pragma once

#include "esphome/core/component.h"
#include "esphome/components/value_validator/value_validator.h"

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
        std::string digit_string;
        for (float d : digits) {
            int digit = static_cast<int>(round(d));
            if (digit >= 10) digit = 0;
            digit_string += std::to_string(digit);
        }
        if (!digit_string.empty()) {
             char *end;
             long val = strtol(digit_string.c_str(), &end, 10);
             if (end == digit_string.c_str() + digit_string.length()) { 
                 validated_value = (int)val;
                 return true;
             }
        }
        return false; 
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
