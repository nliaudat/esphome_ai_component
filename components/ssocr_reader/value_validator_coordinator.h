#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SSOCR_READER

#include "esphome/core/component.h"

#ifdef USE_VALUE_VALIDATOR
#include "esphome/components/value_validator/value_validator.h"
#endif

namespace esphome {
namespace ssocr_reader {

class ValueValidatorCoordinator {
 public:
#ifdef USE_VALUE_VALIDATOR
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
#else
  // Dummy implementation when ValueValidator component is not used
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

}  // namespace ssocr_reader
}  // namespace esphome

#endif // USE_SSOCR_READER
