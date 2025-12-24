/**
 * @file value_validator.cpp
 * @brief Implementation of output validation for meter readings.
 */

#include "value_validator.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "value_validator";

void ReadingHistory::setup() {
  hour_readings_.clear();
  day_readings_.clear();
}

void ReadingHistory::add_reading(int value, uint32_t timestamp, float confidence) {
  HistoricalReading reading{value, timestamp, confidence};
  
  hour_readings_.push_back(reading);
  day_readings_.push_back(reading);
  
  cleanup_old_readings(timestamp);
  
  // Enforce memory limit
  // Estimate size: deque overhead + item size * count
  // HistoricalReading is 12 bytes + deque node overhead (~16-32 bytes depending on platform/impl)
  // Let's approximate roughly 64 bytes per entry total overhead to be safe
  const size_t BYTES_PER_ENTRY = 64; 
  
  size_t current_usage = (hour_readings_.size() + day_readings_.size()) * BYTES_PER_ENTRY;
  
  while (current_usage > max_history_size_bytes_ && !day_readings_.empty()) {
      // If the oldest reading from day_readings is also in hour_readings, remove it.
      // This works because hour_readings is a strict subset of time of day_readings
      if (!hour_readings_.empty() && hour_readings_.front().timestamp <= day_readings_.front().timestamp) {
          hour_readings_.pop_front();
      }
      
      // Remove from day readings (oldest)
      day_readings_.pop_front();
      
      // Recalculate usage
      current_usage = (hour_readings_.size() + day_readings_.size()) * BYTES_PER_ENTRY;
  }
  
  // Also enforce absolute limits just in case
  const size_t MAX_HOUR_READINGS = 360;  // 6 per minute * 60 minutes
  const size_t MAX_DAY_READINGS = 1440;  // 1 per minute * 60 * 24
  
  while (hour_readings_.size() > MAX_HOUR_READINGS) {
      hour_readings_.pop_front();
  }
  while (day_readings_.size() > MAX_DAY_READINGS) {
      day_readings_.pop_front();
  }
  // Check for abnormal growth
  #ifdef DEBUG_METER_READER_MEMORY
  static size_t last_log_time = 0;
  if (millis() - last_log_time > 60000) {
      last_log_time = millis();
      ESP_LOGD(TAG, "History: %zu entries (%zu bytes est), Day: %zu, Hour: %zu", 
              hour_readings_.size() + day_readings_.size(),
              current_usage,
              day_readings_.size(), hour_readings_.size());
  }
  #endif
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
  
  size_t size = values.size();  
  std::nth_element(values.begin(), values.begin() + size/2, values.end());
  int median = values[size/2];
  
  if (size % 2 == 0) {
      auto max_it = std::max_element(values.begin(), values.begin() + size/2);
      return (*max_it + median) / 2;
  }
  return median;
}

int ReadingHistory::get_day_median() const {
  if (day_readings_.empty()) return 0;
  
  std::vector<int> values;
  for (const auto& reading : day_readings_) {
    values.push_back(reading.value);
  }
  
  size_t size = values.size();
  std::nth_element(values.begin(), values.begin() + size/2, values.end());
  int median = values[size/2];
  
  if (size % 2 == 0) {
      auto max_it = std::max_element(values.begin(), values.begin() + size/2);
      return (*max_it + median) / 2;
  }
  return median;
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

void ValueValidator::setup() {
  history_.setup();
  history_.set_max_history_size_bytes(config_.max_history_size_bytes);
  last_valid_reading_ = 0;
  first_reading_ = true;
  last_good_values_.clear();
  
  // Initialize with some capacity
  last_good_values_.resize(config_.smart_validation_window, 0);
}

bool ValueValidator::validate_reading(int new_reading, float confidence, int& validated_reading) {
  uint32_t current_time = millis();
  
  // Capture last confidence before adding the new reading
  float last_confidence = history_.get_last_confidence();

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
  validated_reading = apply_smart_validation(new_reading, confidence, last_confidence);
  
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

bool ValueValidator::is_digit_plausible(int new_reading, int last_reading) const {
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

int ValueValidator::apply_smart_validation(int new_reading, float confidence, float last_confidence) {
  // High confidence readings get more lenient validation
  float confidence_factor = std::min(1.0f, confidence / 0.7f); // Normalize to 0.7 threshold
  
  // Adjust max diff based on confidence
  int effective_max_diff = config_.max_absolute_diff * confidence_factor;
  
  // High confidence override: If confidence is very high (> 90%) AND greater than the previous confidence,
  // trust the reading even if it deviates significantly from history.
  // This solves getting stuck with a bad low-confidence reading in history.
  if (confidence > config_.high_confidence_threshold && confidence > last_confidence) {
      bool is_plausible = is_digit_plausible(new_reading, last_valid_reading_);
      
      if (!is_plausible) {
          ESP_LOGW(TAG, "Validation Override: High confidence (%.2f > %.2f) reading %d accepted despite validation failure (last: %d)", 
                   confidence, last_confidence, new_reading, last_valid_reading_);
                   
          // If the deviation is huge, it's likely a reset or correction of bad history
          // so we shouldn't mix it with old "good" values
          if (std::abs(new_reading - last_valid_reading_) > (config_.max_absolute_diff * 10)) {
              last_good_values_.clear();
              ESP_LOGI(TAG, "History cleared due to high-confidence correction");
          }
          return new_reading;
      }
  }

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

int ValueValidator::find_most_plausible_reading(int new_reading, const std::vector<int>& recent_readings) {
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

bool ValueValidator::is_small_increment(int new_reading, int last_reading) const {
  // Check if this could be a simple digit increment
  int diff = new_reading - last_reading;
  return (diff > 0 && diff <= 10); // Allow small positive increments
}

int ValueValidator::calculate_digit_difference(int reading1, int reading2) const {
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

void ValueValidator::reset() {
  history_.clear();
  last_valid_reading_ = 0;
  first_reading_ = true;
  last_good_values_.clear();
  ESP_LOGI(TAG, "Value validator reset");
}

}  // namespace meter_reader_tflite
}  // namespace esphome