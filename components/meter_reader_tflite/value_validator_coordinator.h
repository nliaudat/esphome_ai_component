#pragma once

#include "esphome/core/defines.h"

#ifdef USE_METER_READER_TFLITE

#include "esphome/core/component.h"

#ifdef USE_VALUE_VALIDATOR
#include "esphome/components/value_validator/value_validator.h"
#endif

#include <limits>
#include <span>

namespace esphome {
namespace meter_reader_tflite {

class ValueValidatorCoordinator {
 public:
#ifdef USE_VALUE_VALIDATOR
  void set_validator(value_validator::ValueValidator *validator) { this->validator_ = validator; }
  
  bool has_validator() const { return this->validator_ != nullptr; }

  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    if (this->validator_ == nullptr) {
      validated_value = raw_value;
      return true; 
    }
    return this->validator_->validate_reading(raw_value, confidence, validated_value);
  }

  bool validate_reading(std::span<const float> digits, std::span<const float> confidences, int &validated_value) {
    if (this->validator_ == nullptr) {
        if (digits.empty()) {
            return false;
        }
        long long val = 0;
        for (float d : digits) {
            int digit = static_cast<int>(round(d));
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
    return this->validator_->validate_reading(digits, confidences, validated_value);
  }

  void set_last_valid_reading(int value) {
    if (this->validator_ != nullptr) {
      this->validator_->set_last_valid_reading(value);
    }
  }

  void set_last_valid_reading(const std::string &value) {
    if (this->validator_ != nullptr) {
      this->validator_->set_last_valid_reading(value);
    }
  }

  value_validator::ValueValidator* get_validator() const { return this->validator_; }


 protected:
  value_validator::ValueValidator *validator_{nullptr};
#else
  // Dummy implementation & logic for when validator is missing
  void set_validator(void *validator) {}
  bool has_validator() const { return false; }
  
  bool validate_reading(int raw_value, float confidence, int &validated_value) {
    validated_value = raw_value;
    return true; 
  }

  bool validate_reading(std::span<const float> digits, std::span<const float> confidences, int &validated_value) {
        if (digits.empty()) {
            return false;
        }
        long long val = 0;
        for (float d : digits) {
            int digit = static_cast<int>(round(d));
            if (digit < 0 || digit >= 10) digit = 0;
            val = val * 10 + digit;
        }
        if (val > std::numeric_limits<int>::max()) {
            validated_value = std::numeric_limits<int>::max();
        } else {
            validated_value = static_cast<int>(val);
        }
        return true; 
  }

  bool validate_reading(std::span<const float> digits, std::span<const float> confidences, float &validated_value) {
      if (digits.empty()) return false;
      long long val = 0;
      for (float d : digits) {
          int digit = static_cast<int>(round(d));
          if (digit < 0 || digit >= 10) digit = 0;
          val = val * 10 + digit;
      }
      validated_value = static_cast<float>(val);
      return true;
  }

  void set_last_valid_reading(int value) {}
  void set_last_valid_reading(const std::string &value) {}
  void* get_validator() const { return nullptr; }
#endif
};

}  // namespace meter_reader_tflite
}  // namespace esphome

#endif // USE_METER_READER_TFLITE