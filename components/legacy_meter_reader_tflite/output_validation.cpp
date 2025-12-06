/**
 * @file output_validation.cpp
 * @brief Implementation of output validation for meter readings.
 */

#include "output_validation.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "output_validation";

void ReadingHistory::setup() {
  hour_readings_.clear();
  day_readings_.clear();
}

void ReadingHistory::add_reading(int value, uint32_t timestamp, float confidence) {
  HistoricalReading reading{value, timestamp, confidence};
  
  hour_readings_.push_back(reading);
  day_readings_.push_back(reading);
  
  cleanup_old_readings(timestamp);
}

int ReadingHistory::get_last_reading() const {
  if (!hour_readings_.empty()) {
    return hour_readings_.back().value;
  }
  return 0;
}

float ReadingHistory::get_last_confidence() const {
  if (!hour_readings_.empty()) {
    return hour_readings_.back().confidence;
  }
  return 0.0f;
}

int ReadingHistory::get_hour_median() const {
  if (hour_readings_.empty()) return 0;
  
  std::vector<int> values;
  for (const auto& reading : hour_readings_) {
    values.push_back(reading.value);
  }
  
  std::sort(values.begin(), values.end());
  size_t size = values.size();
  
  if (size % 2 == 0) {
    return (values[size/2 - 1] + values[size/2]) / 2;
  } else {
    return values[size/2];
  }
}

int ReadingHistory::get_day_median() const {
  if (day_readings_.empty()) return 0;
  
  std::vector<int> values;
  for (const auto& reading : day_readings_) {
    values.push_back(reading.value);
  }
  
  std::sort(values.begin(), values.end());
  size_t size = values.size();
  
  if (size % 2 == 0) {
    return (values[size/2 - 1] + values[size/2]) / 2;
  } else {
    return values[size/2];
  }
}

std::vector<int> ReadingHistory::get_recent_readings(size_t count) const {
  std::vector<int> recent;
  if (hour_readings_.empty()) return recent;
  
  size_t start_idx = (hour_readings_.size() > count) ? hour_readings_.size() - count : 0;
  for (size_t i = start_idx; i < hour_readings_.size(); i++) {
    recent.push_back(hour_readings_[i].value);
  }
  
  return recent;
}

void ReadingHistory::cleanup_old_readings(uint32_t current_timestamp) {
  const uint32_t HOUR_MS = 60 * 60 * 1000;
  const uint32_t DAY_MS = 24 * HOUR_MS;
  
  while (!hour_readings_.empty() && 
         (current_timestamp - hour_readings_.front().timestamp) > HOUR_MS) {
    hour_readings_.pop_front();
  }
  
  while (!day_readings_.empty() && 
         (current_timestamp - day_readings_.front().timestamp) > DAY_MS) {
    day_readings_.pop_front();
  }
}

void ReadingHistory::clear() {
  hour_readings_.clear();
  day_readings_.clear();
}

void OutputValidator::setup() {
  history_.setup();
  last_valid_reading_ = 0;
  first_reading_ = true;
  last_good_values_.clear();
  
  // Initialize with some capacity
  last_good_values_.resize(config_.smart_validation_window, 0);
}

bool OutputValidator::validate_reading(int new_reading, float confidence, int& validated_reading) {
  uint32_t current_time = millis();
  
  // Always add to history for tracking
  history_.add_reading(new_reading, current_time, confidence);
  
  // First reading is always accepted
  if (first_reading_) {
    last_valid_reading_ = new_reading;
    first_reading_ = false;
    validated_reading = new_reading;
    
    // Initialize last good values
    last_good_values_.clear();
    last_good_values_.push_back(new_reading);
    
    ESP_LOGI(TAG, "First reading accepted: %d (confidence: %.2f)", new_reading, confidence);
    return true;
  }
  
  // Apply smart validation
  validated_reading = apply_smart_validation(new_reading, confidence);
  
  bool is_valid = (validated_reading == new_reading);
  
  if (is_valid) {
    // Update last good values
    last_good_values_.push_back(new_reading);
    if (last_good_values_.size() > config_.smart_validation_window) {
      last_good_values_.pop_front();
    }
    last_valid_reading_ = new_reading;
  }
  
  ESP_LOGD(TAG, "Validation: %d -> %d (valid: %s, confidence: %.2f)", 
           new_reading, validated_reading, is_valid ? "yes" : "no", confidence);
  
  return is_valid;
}

bool OutputValidator::is_digit_plausible(int new_reading, int last_reading) const {
  // Calculate absolute difference
  int absolute_diff = std::abs(new_reading - last_reading);
  
  // Check if it's within maximum absolute difference
  if (absolute_diff > config_.max_absolute_diff) {
    ESP_LOGW(TAG, "Absolute difference too large: %d (max: %d)", 
             absolute_diff, config_.max_absolute_diff);
    return false;
  }
  
  // Check for negative rates if not allowed
  if (!config_.allow_negative_rates && new_reading < last_reading) {
    // Allow small negative fluctuations (meter reset or small calibration issues)
    int negative_diff = last_reading - new_reading;
    if (negative_diff > 5) { // Allow very small negative changes (up to 5 units)
      ESP_LOGW(TAG, "Negative rate detected: %d -> %d (diff: %d)", 
               last_reading, new_reading, negative_diff);
      return false;
    }
  }
  
  return true;
}

int OutputValidator::apply_smart_validation(int new_reading, float confidence) {
  // High confidence readings get more lenient validation
  float confidence_factor = std::min(1.0f, confidence / 0.7f); // Normalize to 0.7 threshold
  
  // Adjust max diff based on confidence
  int effective_max_diff = config_.max_absolute_diff * confidence_factor;
  
  // Basic digit plausibility check
  if (!is_digit_plausible(new_reading, last_valid_reading_)) {
    // Try to find a plausible reading from recent history
    auto recent_readings = history_.get_recent_readings(config_.smart_validation_window);
    if (!recent_readings.empty()) {
      int plausible_reading = find_most_plausible_reading(new_reading, recent_readings);
      
      // Check if the plausible reading is better
      if (plausible_reading != new_reading && 
          is_digit_plausible(plausible_reading, last_valid_reading_)) {
        ESP_LOGI(TAG, "Using plausible reading from history: %d (was: %d)", 
                 plausible_reading, new_reading);
        return plausible_reading;
      }
    }
    
    // If no good historical reading, use median of recent values
    if (!last_good_values_.empty()) {
      std::vector<int> good_values(last_good_values_.begin(), last_good_values_.end());
      std::sort(good_values.begin(), good_values.end());
      int median = good_values[good_values.size() / 2];
      
      // Only use median if it's somewhat close to the new reading
      if (std::abs(median - new_reading) < (config_.max_absolute_diff * 2)) {
        ESP_LOGI(TAG, "Using median of last good values: %d (was: %d)", median, new_reading);
        return median;
      }
    }
    
    // Last resort: use last valid reading with small increment if possible
    if (is_small_increment(new_reading, last_valid_reading_)) {
      ESP_LOGI(TAG, "Using small increment from last valid: %d", last_valid_reading_);
      return last_valid_reading_;
    }
    
    ESP_LOGW(TAG, "No plausible alternative found, using last valid: %d", last_valid_reading_);
    return last_valid_reading_;
  }
  
  // Reading is plausible
  return new_reading;
}

int OutputValidator::find_most_plausible_reading(int new_reading, const std::vector<int>& recent_readings) {
  if (recent_readings.empty()) return new_reading;
  
  // Calculate average of recent readings
  int sum = 0;
  for (int reading : recent_readings) {
    sum += reading;
  }
  int average = sum / recent_readings.size();
  
  // Check if average is more plausible
  if (is_digit_plausible(average, last_valid_reading_)) {
    return average;
  }
  
  // Check median
  std::vector<int> sorted_readings = recent_readings;
  std::sort(sorted_readings.begin(), sorted_readings.end());
  int median = sorted_readings[sorted_readings.size() / 2];
  
  if (is_digit_plausible(median, last_valid_reading_)) {
    return median;
  }
  
  // Return the most recent plausible reading
  for (auto it = recent_readings.rbegin(); it != recent_readings.rend(); ++it) {
    if (is_digit_plausible(*it, last_valid_reading_)) {
      return *it;
    }
  }
  
  return new_reading; // Fallback to original
}

bool OutputValidator::is_small_increment(int new_reading, int last_reading) const {
  // Check if this could be a simple digit increment
  int diff = new_reading - last_reading;
  return (diff > 0 && diff <= 10); // Allow small positive increments
}

int OutputValidator::calculate_digit_difference(int reading1, int reading2) const {
  // Convert to strings to compare digit by digit
  std::string str1 = std::to_string(reading1);
  std::string str2 = std::to_string(reading2);
  
  // Make strings same length by padding with zeros
  size_t max_len = std::max(str1.length(), str2.length());
  str1 = std::string(max_len - str1.length(), '0') + str1;
  str2 = std::string(max_len - str2.length(), '0') + str2;
  
  int digit_diff = 0;
  for (size_t i = 0; i < max_len; i++) {
    digit_diff += std::abs((str1[i] - '0') - (str2[i] - '0'));
  }
  
  return digit_diff;
}

void OutputValidator::reset() {
  history_.clear();
  last_valid_reading_ = 0;
  first_reading_ = true;
  last_good_values_.clear();
  ESP_LOGI(TAG, "Output validator reset");
}

}  // namespace meter_reader_tflite
}  // namespace esphome