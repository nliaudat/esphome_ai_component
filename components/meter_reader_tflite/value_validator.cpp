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
  readings_.clear();
}

void ReadingHistory::add_reading(int value, uint32_t timestamp, float confidence) {
  HistoricalReading reading{value, timestamp, confidence};
  
  readings_.push_back(reading);
  
  cleanup_old_readings(timestamp);
  
  // Enforce memory limit
  // HistoricalReading is 12 bytes
  // Deque node overhead is platform dependent, estimating ~16 bytes per entry average including block overhead
  const size_t BYTES_PER_ENTRY = 28; 
  
  size_t current_usage = readings_.size() * BYTES_PER_ENTRY;
  
  while (current_usage > max_history_size_bytes_ && !readings_.empty()) {
      readings_.pop_front();
      current_usage = readings_.size() * BYTES_PER_ENTRY;
  }
  
  // Also enforce absolute limits just in case (e.g. 24 hours * 60 minutes)
  const size_t MAX_DAY_READINGS = 1440;
  
  while (readings_.size() > MAX_DAY_READINGS) {
      readings_.pop_front();
  }

  // Check for abnormal growth
  #ifdef DEBUG_METER_READER_MEMORY
  static size_t last_log_time = 0;
  if (millis() - last_log_time > 60000) {
      last_log_time = millis();
      ESP_LOGD(TAG, "History: %zu entries (%zu bytes est)", 
              readings_.size(),
              current_usage);
  }
  #endif
}

int ReadingHistory::get_last_reading() const {
  if (!readings_.empty()) {
    return readings_.back().value;
  }
  return 0;
}

float ReadingHistory::get_last_confidence() const {
  if (!readings_.empty()) {
    return readings_.back().confidence;
  }
  return 0.0f;
}

int ReadingHistory::get_hour_median() const {
  if (readings_.empty()) return 0;
  
  // Collect readings from the last hour
  uint32_t now = readings_.back().timestamp;
  uint32_t threshold = (now >= 3600000) ? (now - 3600000) : 0;

  std::vector<int> values;
  // Iterate backwards to find recent readings efficiently
  for (auto it = readings_.rbegin(); it != readings_.rend(); ++it) {
      if (it->timestamp < threshold) break;
      values.push_back(it->value);
  }
  
  if (values.empty()) return 0;

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
  if (readings_.empty()) return 0;
  
  // Use stride to avoid sorting massive vector if size is large?
  // For < 1440 items, sorting is fine on ESP32 (couple of ms).
  
  std::vector<int> values;
  values.reserve(readings_.size());
  for (const auto& reading : readings_) {
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
  if (readings_.empty()) return recent;
  
  size_t start_idx = (readings_.size() > count) ? readings_.size() - count : 0;
  for (size_t i = start_idx; i < readings_.size(); i++) {
    recent.push_back(readings_[i].value);
  }
  
  return recent;
}

void ReadingHistory::cleanup_old_readings(uint32_t current_timestamp) {
  const uint32_t DAY_MS = 24 * 60 * 60 * 1000;
  
  while (!readings_.empty() && 
         (current_timestamp - readings_.front().timestamp) > DAY_MS) {
    readings_.pop_front();
  }
}

size_t ReadingHistory::get_hour_count() const {
    if (readings_.empty()) return 0;
    
    uint32_t now = readings_.back().timestamp;
    uint32_t threshold = (now >= 3600000) ? (now - 3600000) : 0;
    
    size_t count = 0;
    for (auto it = readings_.rbegin(); it != readings_.rend(); ++it) {
        if (it->timestamp < threshold) break;
        count++;
    }
    return count;
}

void ReadingHistory::clear() {
  readings_.clear();
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

bool ValueValidator::validate_reading(const std::vector<float>& digits, const std::vector<float>& confidences, int& validated_reading) {
  // If we don't have per-digit history yet, fallback to standard validation
  // But wait, we can't do standard validation without a single int.
  // We first construct the "raw" int from the digits.
  
  std::vector<int> current_digits;
  std::string digit_string;
  
  for (float d : digits) {
      int digit = static_cast<int>(round(d));
      if (digit >= 10) digit = 0; // Wrap 10 -> 0 standard TFLite behavior
      current_digits.push_back(digit);
      digit_string += std::to_string(digit);
  }
  
  int raw_val = 0;
  if (!digit_string.empty()) {
      char *end;
      long val = strtol(digit_string.c_str(), &end, 10);
      if (end == digit_string.c_str() + digit_string.length()) { // Full conversion
          raw_val = (int)val;
      }
  }
  
  // Calculate average confidence for the legacy validator call
  float avg_conf = 0.0f;
  if (!confidences.empty()) {
      avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / confidences.size();
  }

  // --- Per-Digit Filtering ---
  int filtered_val = raw_val;
  bool final_valid_check = true; // Assumed valid unless strict check fails
  
  if (!first_reading_ && !last_valid_digits_.empty() && last_valid_digits_.size() == current_digits.size()) {
      std::string filtered_digit_string;
      bool modified = false;
      
      for (size_t i = 0; i < current_digits.size(); i++) {
          int new_d = current_digits[i];
          int old_d = last_valid_digits_[i];
          float conf = (i < confidences.size()) ? confidences[i] : 0.0f;
          
          if (new_d != old_d) {
              // Digit changed. Check confidence.
              if (conf < config_.per_digit_confidence_threshold) {
                  // Confidence too low for a change. Reject it.
                  ESP_LOGW(TAG, "Digit %d change rejected (Val: %d->%d, Conf: %.2f < %.2f)", 
                           (int)i, old_d, new_d, conf, config_.per_digit_confidence_threshold);
                  filtered_digit_string += std::to_string(old_d);
                  modified = true;
              } else {
                  // Accepted
                  filtered_digit_string += std::to_string(new_d);
              }
          } else {
              // Unchanged match
              // Strict check: if high_confidence_threshold is configured (per_digit_confidence_threshold),
              // we require even unchanged digits to meet it IF strict mode is enabled.
              // User request: "I want all digit to be upper".
              if (config_.strict_confidence_check && conf < config_.per_digit_confidence_threshold) {
                  ESP_LOGW(TAG, "Digit %d unchanged but low confidence (Conf: %.2f < %.2f) - Rejecting reading in strict mode", 
                           (int)i, conf, config_.per_digit_confidence_threshold);
                  // Mark invalid, but continue constructing string to maintain state if needed
                  final_valid_check = false;
              }
              filtered_digit_string += std::to_string(new_d);
          }
      }
      
      if (modified) {
          char *end;
          long val = strtol(filtered_digit_string.c_str(), &end, 10);
          if (end == filtered_digit_string.c_str() + filtered_digit_string.length()) {
             filtered_val = (int)val;
             ESP_LOGD(TAG, "Per-digit filter modified reading: %d -> %d", raw_val, filtered_val);
          } else {
             filtered_val = raw_val; // Fallback
          }
      }
  } else {
       // First reading or no history - check all digits strictly
       for (size_t i = 0; i < current_digits.size(); i++) {
           float conf = (i < confidences.size()) ? confidences[i] : 0.0f;
           if (conf < config_.per_digit_confidence_threshold) {
               ESP_LOGW(TAG, "Initial reading digit %d low confidence (Conf: %.2f < %.2f)", 
                        (int)i, conf, config_.per_digit_confidence_threshold);
               final_valid_check = false;
           }
       }
  }
  
  if (!final_valid_check) {
      // If we failed strict per-digit check, we return false immediately?
      // Or we let legacy validator run but return false at end?
      // Better to fail now to avoid logging "valid: yes".
      return false;
  }

  // Now pass the FILTERED value to the standard validator
  // This will perform rate checks, consistency checks, etc.
  bool final_valid = validate_reading(filtered_val, avg_conf, validated_reading);
  
  if (final_valid) {
      // If accepted, update our digit history
      // We must reconstruct the digits of `validated_reading` because the standard validator 
      // might have modified it (e.g. median filter, history recovery, etc.)
      
      // Edge case: if validate_reading returned a value from history (e.g. 210600), we need its digits.
      // Since we don't know the exact digits of that historic integer easily (could have ambiguous length),
      // we assume it has the same length as current input.
      std::string val_str = std::to_string(validated_reading);
      
      // Pad if necessary? usually leading zeros.
      if (val_str.length() < current_digits.size()) {
          val_str = std::string(current_digits.size() - val_str.length(), '0') + val_str;
      }
      
      last_valid_digits_.clear();
      for (char c : val_str) {
          last_valid_digits_.push_back(c - '0');
      }
  }
  
  return final_valid;
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
  // Basic digit plausibility check
  if (is_digit_plausible(new_reading, last_valid_reading_)) {
    return new_reading;
  }

  // If the reading is NOT plausible (e.g. large jump), check for consistency.
  // Instead of an instant override based on confidence, we require the value 
  // to be stable for a few readings. This prevents flickering.
  
  // Get recent history (including the current one we just added)
  // We need enough history to check for N consecutive readings.
  const size_t CONSISTENCY_COUNT = 3;
  auto recent = history_.get_recent_readings(CONSISTENCY_COUNT);
  
  if (recent.size() >= CONSISTENCY_COUNT) {
      bool consistent = true;
      for (int val : recent) {
          if (val != new_reading) {
              consistent = false;
              break;
          }
      }
      
      if (consistent) {
           ESP_LOGW(TAG, "Large change confirmed by %d consecutive readings: %d -> %d", 
                    (int)CONSISTENCY_COUNT, last_valid_reading_, new_reading);
           
           // If the deviation is huge, clear old history to prevent it from dragging us back
           if (std::abs(new_reading - last_valid_reading_) > (config_.max_absolute_diff * 5)) {
               last_good_values_.clear();
           }
           return new_reading;
      }
  }

  // --- Fallback Strategies ---

  // 1. Try to find a plausible reading from recent history
  auto recent_readings = history_.get_recent_readings(config_.smart_validation_window);
  if (!recent_readings.empty()) {
    int plausible_reading = find_most_plausible_reading(new_reading, recent_readings);
    
    // Check if the plausible reading is better
    if (plausible_reading != new_reading && 
        is_digit_plausible(plausible_reading, last_valid_reading_)) {
      ESP_LOGD(TAG, "Using plausible reading from history: %d (was: %d)", 
               plausible_reading, new_reading);
      return plausible_reading;
    }
  }
    
  // 2. If no good historical reading, use median of recent GOOD values
  if (!last_good_values_.empty()) {
    std::vector<int> good_values(last_good_values_.begin(), last_good_values_.end());
    std::sort(good_values.begin(), good_values.end());
    int median = good_values[good_values.size() / 2];
    
    // Only use median if it's somewhat close to the last valid
    if (std::abs(median - last_valid_reading_) <= config_.max_absolute_diff) {
      ESP_LOGD(TAG, "Using median of last good values: %d (was: %d)", median, new_reading);
      return median;
    }
  }
    
  // 3. Last resort: use last valid reading
  // Note: We intentionally avoid small_increment logic here if it failed is_digit_plausible
  // because is_digit_plausible usually allows small increments.
  
  ESP_LOGW(TAG, "No plausible alternative found, keeping last valid: %d (Ignored: %d)", last_valid_reading_, new_reading);
  return last_valid_reading_;
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
  last_valid_digits_.clear();
  ESP_LOGI(TAG, "Value validator reset");
}

void ValueValidator::set_last_valid_reading(int value) {
  last_valid_reading_ = value;
  first_reading_ = false; // We have a valid reading now
  
  // Update last_valid_digits_
  std::string val_str = std::to_string(value);
  last_valid_digits_.clear();
  for (char c : val_str) {
      last_valid_digits_.push_back(c - '0');
  }
  
  // Create a "fake" history for this value to stabilize it
  last_good_values_.clear();
  // Fill with this value to pass median filters immediately
  for(int i=0; i<config_.smart_validation_window; i++) {
     last_good_values_.push_back(value);
  }
  
  // Also clear raw history to prevent "consistency check" from reverting back to old values
  history_.clear(); 
  // Add a few dummy entries so it doesn't look empty? 
  // No, let's just clear it. The next reading will be compared to last_valid_reading_.
  // Consistency check (Layer 2) requires 'recent' readings. If history is empty, recent is empty.
  // The code `if (recent.size() >= CONSISTENCY_COUNT)` handles that. 
  // If we want the next reading to be accepted immediately, we need history?
  // Actually, if we clear history, the consistency check might NOT trigger (size < count).
  // Then it falls back to IsDigitPlausible.
  // If the new real reading is close to this manually set value, IsDigitPlausible returns true. -> Accepted.
  // So clear() is fine.
  
  ESP_LOGW(TAG, "Manually set last valid reading to: %d", value);
}


void ValueValidator::set_strict_confidence_check(bool strict) {
  config_.strict_confidence_check = strict;
}

}  // namespace meter_reader_tflite
}  // namespace esphome