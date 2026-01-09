/**
 * @file value_validator.h
 * @brief Output validation and historical data management for meter readings.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace esphome {
namespace value_validator {

/**
 * @class ReadingHistory
 * @brief Manages historical meter readings for validation
 */
class ReadingHistory {
 public:
  struct HistoricalReading {
    int value;  // Use integer for precise digit comparison
    uint32_t timestamp;
    float confidence;
  };

  void setup();
  void add_reading(int value, uint32_t timestamp, float confidence);
  void set_max_history_size_bytes(size_t size) { max_history_size_bytes_ = size; }
  
  int get_last_reading() const;
  float get_last_confidence() const;
  int get_hour_median() const;
  int get_day_median() const;
  std::vector<int> get_recent_readings(size_t count) const;
  
  size_t get_hour_count() const;
  size_t get_day_count() const { return count_; }
  
  void clear();
  ~ReadingHistory();

 private:
  HistoricalReading* buffer_{nullptr};
  size_t capacity_{0};
  size_t head_{0}; // Write index (points to next free slot)
  size_t count_{0}; // Current number of elements
  
  size_t max_history_size_bytes_{51200};
  
  void ensure_capacity();
};

/**
 * @class ValueValidator
 * @brief Validates meter readings against historical data and rules
 */
class ValueValidator : public Component {
 public:
  struct ValidationConfig {
    bool allow_negative_rates{false};
    int max_absolute_diff{100};
    float max_rate_change{0.15f}; // 15% maximum change per reading
    bool enable_smart_validation{true};
    int smart_validation_window{5};
    float high_confidence_threshold{0.90f}; // Threshold for validation override
    size_t max_history_size_bytes{51200}; // 50KB limit for history buffer
    float per_digit_confidence_threshold{0.85f}; // Minimum confidence to accept a changed digit
    bool strict_confidence_check{false}; // If true, requires all digits to be above threshold
  };

  ~ValueValidator();
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Legacy single-value validation
  bool validate_reading(int new_reading, float confidence, int& validated_reading);
  // Per-digit validation
  bool validate_reading(const std::vector<float>& digits, const std::vector<float>& confidences, int& validated_reading);
  
  void set_config(const ValidationConfig& config) { config_ = config; }
  const ValidationConfig& get_config() const { return config_; }
  
  // Setters for configuration
  void set_allow_negative_rates(bool v) { config_.allow_negative_rates = v; }
  void set_max_absolute_diff(int v) { config_.max_absolute_diff = v; }
  void set_max_rate_change(float v) { config_.max_rate_change = v; }
  void set_enable_smart_validation(bool v) { config_.enable_smart_validation = v; }
  void set_smart_validation_window(int v) { config_.smart_validation_window = v; }
  void set_high_confidence_threshold(float v) { config_.high_confidence_threshold = v; }
  void set_max_history_size_bytes(size_t v) { config_.max_history_size_bytes = v; }
  void set_per_digit_confidence_threshold(float v) { config_.per_digit_confidence_threshold = v; }
  void set_strict_confidence_check(bool v) { config_.strict_confidence_check = v; }

  int get_last_valid_reading() const { return last_valid_reading_; }
  const ReadingHistory& get_history() const { return history_; }
  
  void reset();
  void set_last_valid_reading(int value);

 private:
  ValidationConfig config_;
  ReadingHistory history_;
  int last_valid_reading_{0};
  
  // Last valid digits (PSRAM array)
  int* last_valid_digits_data_{nullptr};
  size_t last_valid_digits_count_{0};

  // Per-digit history for stability check (Flat arrays for PSRAM efficiency)
  int* digit_history_data_{nullptr};      // [num_digits * 5]
  uint8_t* digit_history_counts_{nullptr}; // [num_digits]
  uint8_t* digit_history_heads_{nullptr};  // [num_digits]
  size_t digit_history_num_digits_{0};
  static const size_t DIGIT_HISTORY_SIZE = 5;

  bool first_reading_{true};
  
  // Recent good values ring buffer (PSRAM)
  int* last_good_values_data_{nullptr};
  size_t last_good_values_capacity_{0};
  size_t last_good_values_head_{0};
  size_t last_good_values_count_{0};
  
  bool is_digit_plausible(int new_reading, int last_reading) const;
  int apply_smart_validation(int new_reading, float confidence, float last_confidence);
  int find_most_plausible_reading(int new_reading, const std::vector<int>& recent_readings);
  bool is_small_increment(int new_reading, int last_reading) const;
  int calculate_digit_difference(int reading1, int reading2) const;
  int get_stable_digit(int digit_index, int new_digit);
  void ensure_digit_history_size(size_t num_digits);
  void ensure_last_valid_digits_size(size_t num_digits);
  void ensure_last_good_values_capacity(size_t capacity);
  void add_good_value(int value);
  int get_good_values_median() const;
  
  void free_digit_history();
  void free_resources();
};

}  // namespace value_validator
}  // namespace esphome
