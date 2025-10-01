/**
 * @file output_validation.h
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
namespace meter_reader_tflite {

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
  
  int get_last_reading() const;
  float get_last_confidence() const;
  int get_hour_median() const;
  int get_day_median() const;
  std::vector<int> get_recent_readings(size_t count) const;
  
  size_t get_hour_count() const { return hour_readings_.size(); }
  size_t get_day_count() const { return day_readings_.size(); }
  
  void clear();

 private:
  std::deque<HistoricalReading> hour_readings_;
  std::deque<HistoricalReading> day_readings_;
  
  void cleanup_old_readings(uint32_t current_timestamp);
};

/**
 * @class OutputValidator
 * @brief Validates meter readings against historical data and rules
 */
class OutputValidator {
 public:
  struct ValidationConfig {
    bool allow_negative_rates{false};
    int max_absolute_diff{100};
    float max_rate_change{0.15f}; // 15% maximum change per reading
    bool enable_smart_validation{true};
    int smart_validation_window{5};
  };

  void setup();
  bool validate_reading(int new_reading, float confidence, int& validated_reading);
  void set_config(const ValidationConfig& config) { config_ = config; }
  const ValidationConfig& get_config() const { return config_; }
  
  int get_last_valid_reading() const { return last_valid_reading_; }
  const ReadingHistory& get_history() const { return history_; }
  
  void reset();

 private:
  ValidationConfig config_;
  ReadingHistory history_;
  int last_valid_reading_{0};
  bool first_reading_{true};
  std::deque<int> last_good_values_; // Store last N good values for precise comparison
  
  bool is_digit_plausible(int new_reading, int last_reading) const;
  int apply_smart_validation(int new_reading, float confidence);
  int find_most_plausible_reading(int new_reading, const std::vector<int>& recent_readings);
  bool is_small_increment(int new_reading, int last_reading) const;
  int calculate_digit_difference(int reading1, int reading2) const;
};

}  // namespace meter_reader_tflite
}  // namespace esphome