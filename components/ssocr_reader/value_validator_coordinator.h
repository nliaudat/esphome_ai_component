#pragma once

#include "esphome/core/component.h"
#include "esphome/components/value_validator/value_validator.h"

namespace esphome {
namespace ssocr_reader {

class ValueValidatorCoordinator {
 public:
  void set_validator(value_validator::ValueValidator *validator) { this->validator_ = validator; }

  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    if (this->validator_ == nullptr) {
      validated_value = raw_value;
      return true; // Assume valid if no validator set, or handle as policy
    }
    return this->validator_->validate_reading(raw_value, confidence, validated_value);
  }

  bool validate_reading(const std::vector<float> &digits, const std::vector<float> &confidences, int &validated_value) {
    if (this->validator_ == nullptr) {
        // Fallback or error? defaulting to false safely
        return false; 
    }
    return this->validator_->validate_reading(digits, confidences, validated_value);
  }

  void set_last_valid_reading(int value) {
    if (this->validator_ != nullptr) {
      this->validator_->set_last_valid_reading(value);
    }
  }

 protected:
  value_validator::ValueValidator *validator_{nullptr};
};

}  // namespace ssocr_reader
}  // namespace esphome
