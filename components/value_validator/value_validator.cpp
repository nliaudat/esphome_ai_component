/**
 * @file value_validator.cpp
 * @brief Implementation of output validation for meter readings.
 */

#include "value_validator.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_heap_caps.h>

namespace esphome {
namespace value_validator {

static const char *const TAG = "value_validator";

void ReadingHistory::setup() {
  clear();
  ensure_capacity();
}

ReadingHistory::~ReadingHistory() {
  if (buffer_) {
    free(buffer_);
    buffer_ = nullptr;
  }
}

void ReadingHistory::ensure_capacity() {
  if (capacity_ > 0) return;
  
  // Calculate capacity based on max bytes
  // HistoricalReading size is 12 bytes
  size_t max_count_by_bytes = max_history_size_bytes_ / sizeof(HistoricalReading);
  const size_t MAX_DAY_COUNT = 1440; //prevents excessive memory usage while still allowing for 24 hours of minute-by-minute tracking (24h*60min =1440)
  
  size_t target = std::min(max_count_by_bytes, MAX_DAY_COUNT);
  if (target < 10) target = 10; // Minimum sanity
  
  uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  buffer_ = (HistoricalReading*)heap_caps_malloc(target * sizeof(HistoricalReading), caps);
  if (!buffer_) {
      buffer_ = (HistoricalReading*)malloc(target * sizeof(HistoricalReading));
  }
  
  if (buffer_) {
      capacity_ = target;
      head_ = 0;
      count_ = 0;
      ESP_LOGI(TAG, "Allocated history buffer for %d entries (PSRAM preferred)", (int)target);
  } else {
      ESP_LOGE(TAG, "Failed to allocate history buffer!");
      capacity_ = 0;
  }
}

void ReadingHistory::add_reading(int value, uint32_t timestamp, float confidence) {
  if (!buffer_ || capacity_ == 0) return;
  
  HistoricalReading reading{value, timestamp, confidence};
  
  buffer_[head_] = reading;
  head_ = (head_ + 1) % capacity_;
  
  if (count_ < capacity_) {
      count_++;
  }
}

int ReadingHistory::get_last_reading() const {
  if (count_ == 0 || !buffer_) return 0;
  size_t idx = (head_ == 0) ? (capacity_ - 1) : (head_ - 1);
  return buffer_[idx].value;
}

float ReadingHistory::get_last_confidence() const {
  if (count_ == 0 || !buffer_) return 0.0f;
  size_t idx = (head_ == 0) ? (capacity_ - 1) : (head_ - 1);
  return buffer_[idx].confidence;
}

int ReadingHistory::get_hour_median() const {
  if (count_ == 0 || !buffer_) return 0;
  
  // Iterate backwards
  // Access last reading directly
  size_t last_idx = (head_ == 0) ? (capacity_ - 1) : (head_ - 1);
  uint32_t now = buffer_[last_idx].timestamp;
  
  uint32_t threshold = (now >= 3600000) ? (now - 3600000) : 0;

  std::vector<int> values;
  size_t curr = last_idx;
  for (size_t i = 0; i < count_; i++) {
      if (buffer_[curr].timestamp < threshold) break;
      values.push_back(buffer_[curr].value);
      
      if (curr == 0) curr = capacity_ - 1;
      else curr--;
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
  if (count_ == 0 || !buffer_) return 0;
  
  // Need timestamp of last reading
  size_t last_idx = (head_ == 0) ? (capacity_ - 1) : (head_ - 1);
  uint32_t now = buffer_[last_idx].timestamp;
  
  // 24 hours = 86400000 ms
  uint32_t threshold = (now >= 86400000) ? (now - 86400000) : 0;

  std::vector<int> values;
  size_t curr = last_idx;
  for (size_t i = 0; i < count_; i++) {
      if (buffer_[curr].timestamp < threshold) break;
      values.push_back(buffer_[curr].value);
      
      if (curr == 0) curr = capacity_ - 1;
      else curr--;
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

std::vector<int> ReadingHistory::get_recent_readings(size_t count) const {
  std::vector<int> recent;
  if (count_ == 0 || !buffer_) return recent;
  
  size_t actual_count = std::min(count, count_);
  
  
  for (size_t k = 0; k < actual_count; k++) {
      // We want chronological: so start from k = actual_count - 1 down to 0
      
      size_t offset = actual_count - 1 - k;
      
      size_t idx;
      if (head_ > offset) idx = head_ - 1 - offset;
      else idx = head_ - 1 - offset + capacity_;
      
      recent.push_back(buffer_[idx].value);
  }
  
  return recent;
}

void ReadingHistory::clear() {
  head_ = 0;
  count_ = 0;
}

void ValueValidator::setup() {
  history_.setup();
  history_.set_max_history_size_bytes(config_.max_history_size_bytes);
  last_valid_reading_ = 0;
  first_reading_ = true;
  last_good_values_count_ = 0;
  last_good_values_head_ = 0;
  free_digit_history();
  
  // Initialize with some capacity
  ensure_last_good_values_capacity(config_.smart_validation_window);
}

void ValueValidator::dump_config() {
  ESP_LOGCONFIG(TAG, "Value Validator:");
  ESP_LOGCONFIG(TAG, "  Max Absolute Diff: %d", config_.max_absolute_diff);
  ESP_LOGCONFIG(TAG, "  Allow Negative Rates: %s", YESNO(config_.allow_negative_rates));
  ESP_LOGCONFIG(TAG, "  Strict Confidence Check: %s", YESNO(config_.strict_confidence_check));
  ESP_LOGCONFIG(TAG, "  Per Digit Conf Threshold: %.2f", config_.per_digit_confidence_threshold);
  ESP_LOGCONFIG(TAG, "  Max History Size: %d bytes", (int)config_.max_history_size_bytes);
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
    last_good_values_count_ = 0;
    last_good_values_head_ = 0;
    add_good_value(new_reading);
    
    ESP_LOGI(TAG, "First reading accepted: %d (confidence: %.2f)", new_reading, confidence);
    return true;
  }
  
  // Apply smart validation
  validated_reading = apply_smart_validation(new_reading, confidence, last_confidence);
  
  bool is_valid = (validated_reading == new_reading);
  
  if (is_valid) {
    // Update last good values
    add_good_value(new_reading);
    // Capacity managed by ring buffer logic automatically
    last_valid_reading_ = new_reading;
  }
  
  ESP_LOGD(TAG, "Validation: %d -> %d (valid: %s, confidence: %.2f)", 
           new_reading, validated_reading, is_valid ? "yes" : "no", confidence);
  
  return is_valid;
}

bool ValueValidator::validate_reading(const std::vector<float>& digits, const std::vector<float>& confidences, int& validated_reading) {
  // If we don't have per-digit history yet, fallback to standard validation
  
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
      // Manual safe conversion to avoid exceptions
      bool safe = true;
      for(char c : digit_string) {
          if(!isdigit(c)) { safe = false; break; }
      }
      if(safe) {
          raw_val = std::atoi(digit_string.c_str());
      }
  }
  
  // --- Per-Digit History & Stability Filter ---
  // Before constructing the "raw_val", we process each digit through the history filter.
  // This helps correct single flickering digits.
  
  // Resize history if needed
  ensure_digit_history_size(current_digits.size());
  
  std::string stabilized_digit_string;
  for (size_t i = 0; i < current_digits.size(); i++) {
      int raw_d = current_digits[i];
      int stable_d = get_stable_digit(i, raw_d); // Update history and get stable value
      stabilized_digit_string += std::to_string(stable_d);
      
      // Update our current_digits vector to reflect the stabilized values
      current_digits[i] = stable_d;
  }
  
  // Re-calculate raw_val from STABILIZED digits
  raw_val = 0;
  if (!stabilized_digit_string.empty()) {
      char *end;
      long val = strtol(stabilized_digit_string.c_str(), &end, 10);
      if (end == stabilized_digit_string.c_str() + stabilized_digit_string.length()) { 
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
  
  if (!first_reading_ && last_valid_digits_count_ == current_digits.size()) {
      std::string filtered_digit_string;
      bool modified = false;
      
      for (size_t i = 0; i < current_digits.size(); i++) {
          int new_d = current_digits[i];
          int old_d = last_valid_digits_data_[i];
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
      
      ensure_last_valid_digits_size(val_str.length());
      for (size_t i = 0; i < val_str.length(); i++) {
          last_valid_digits_data_[i] = val_str[i] - '0';
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
           // STRICT ENFORCEMENT: Even if consistent, do not allow negative rates if forbidden.
           if (!config_.allow_negative_rates && new_reading < last_valid_reading_) {
               // Allow small fluctuations only
               if ((last_valid_reading_ - new_reading) > 5) {
                   ESP_LOGW(TAG, "Consistent negative reading rejected (AllowNegativeRates=false): %d -> %d", 
                            last_valid_reading_, new_reading);
                   return last_valid_reading_;
               }
           }

           ESP_LOGW(TAG, "Large change confirmed by %d precise consecutive readings: %d -> %d", 
                    (int)CONSISTENCY_COUNT, last_valid_reading_, new_reading);
           
           // If the deviation is huge, clear old history to prevent it from dragging us back
           if (std::abs(new_reading - last_valid_reading_) > (config_.max_absolute_diff * 5)) {
               last_good_values_count_ = 0;
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
  if (last_good_values_count_ > 0) {
    int median = get_good_values_median();
    
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
  last_good_values_count_ = 0;
  last_good_values_head_ = 0;
  free_digit_history();
  last_valid_digits_count_ = 0;
  ESP_LOGI(TAG, "Value validator reset");
}

void ValueValidator::set_last_valid_reading(int value) {
  last_valid_reading_ = value;
  first_reading_ = false; // We have a valid reading now
  
  // Update last_valid_digits_
  std::string val_str = std::to_string(value);
  ensure_last_valid_digits_size(val_str.length());
  for (size_t i = 0; i < val_str.length(); i++) {
        last_valid_digits_data_[i] = val_str[i] - '0';
  }
  
  // Create a "fake" history for this value to stabilize it
  last_good_values_count_ = 0;
  // Fill with this value to pass median filters immediately
  for(int i=0; i<config_.smart_validation_window; i++) {
     add_good_value(value);
  }
  
  // Also clear raw history to prevent "consistency check" from reverting back to old values
  history_.clear();  
  ESP_LOGW(TAG, "Manually set last valid reading to: %d", value);
}

void ValueValidator::set_last_valid_reading(const std::string &value) {
  int int_val = 0;
  if (!value.empty()) {
      char* end = nullptr;
      long val = strtol(value.c_str(), &end, 10);
      if (end == value.c_str()) {
          ESP_LOGE(TAG, "Failed to parse manual value string: %s", value.c_str());
          return;
      }
      int_val = (int)val;
  }

  // Set the integer value
  last_valid_reading_ = int_val;
  first_reading_ = false;

  // Use the STRING length for digit count, preserving leading zeros
  ensure_last_valid_digits_size(value.length());
  for (size_t i = 0; i < value.length(); i++) {
        if (isdigit(value[i])) {
            last_valid_digits_data_[i] = value[i] - '0';
        } else {
            last_valid_digits_data_[i] = 0; // Default to 0 for non-digits?
        }
  }
  
  // Create a "fake" history for this value
  last_good_values_count_ = 0;
  for(int i=0; i<config_.smart_validation_window; i++) {
     add_good_value(int_val);
  }
  
  history_.clear();  
  ESP_LOGW(TAG, "Manually set last valid reading to: %d (Digits: %d, Str: %s)", int_val, (int)value.length(), value.c_str());
}

void ValueValidator::free_resources() {
  free_digit_history();
  if (last_good_values_data_) {
      free(last_good_values_data_);
      last_good_values_data_ = nullptr;
  }
  if (last_valid_digits_data_) {
      free(last_valid_digits_data_);
      last_valid_digits_data_ = nullptr;
  }
}

ValueValidator::~ValueValidator() {
  free_resources();
}

void ValueValidator::ensure_last_valid_digits_size(size_t num_digits) {
  if (num_digits == last_valid_digits_count_ && last_valid_digits_data_) return;
  
  if (last_valid_digits_data_) free(last_valid_digits_data_);
  
  uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  last_valid_digits_data_ = (int*)heap_caps_malloc(num_digits * sizeof(int), caps);
  if (!last_valid_digits_data_) {
      last_valid_digits_data_ = (int*)malloc(num_digits * sizeof(int));
  }
  last_valid_digits_count_ = num_digits;
}

void ValueValidator::ensure_last_good_values_capacity(size_t capacity) {
  if (capacity == last_good_values_capacity_ && last_good_values_data_) return;
  
  if (last_good_values_data_) free(last_good_values_data_);
  
  uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  last_good_values_data_ = (int*)heap_caps_malloc(capacity * sizeof(int), caps);
  if (!last_good_values_data_) {
      last_good_values_data_ = (int*)malloc(capacity * sizeof(int));
  }
  last_good_values_capacity_ = capacity;
  last_good_values_count_ = 0;
  last_good_values_head_ = 0;
}

void ValueValidator::add_good_value(int value) {
  if (!last_good_values_data_ || last_good_values_capacity_ == 0) return;
  
  last_good_values_data_[last_good_values_head_] = value;
  last_good_values_head_ = (last_good_values_head_ + 1) % last_good_values_capacity_;
  if (last_good_values_count_ < last_good_values_capacity_) {
      last_good_values_count_++;
  }
}

int ValueValidator::get_good_values_median() const {
  if (last_good_values_count_ == 0 || !last_good_values_data_) return 0;
  
  // Copy to temp vector for sorting
  std::vector<int> values;
  size_t curr = (last_good_values_head_ == 0) ? (last_good_values_capacity_ - 1) : (last_good_values_head_ - 1);
  for (size_t i = 0; i < last_good_values_count_; i++) {
      values.push_back(last_good_values_data_[curr]);
      if (curr == 0) curr = last_good_values_capacity_ - 1;
      else curr--;
  }
  
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

void ValueValidator::free_digit_history() {
  if (digit_history_data_) {
      free(digit_history_data_);
      digit_history_data_ = nullptr;
  }
  if (digit_history_counts_) {
      free(digit_history_counts_);
      digit_history_counts_ = nullptr;
  }
  if (digit_history_heads_) {
      free(digit_history_heads_);
      digit_history_heads_ = nullptr;
  }
  digit_history_num_digits_ = 0;
}

void ValueValidator::ensure_digit_history_size(size_t num_digits) {
  if (num_digits == digit_history_num_digits_) return;
  
  // Reallocate
  free_digit_history();
  
  if (num_digits == 0) return;
  
  // Try PSRAM first
  uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  
  digit_history_data_ = (int*)heap_caps_malloc(num_digits * DIGIT_HISTORY_SIZE * sizeof(int), caps);
  if (!digit_history_data_) {
      // Fallback to internal RAM
      digit_history_data_ = (int*)malloc(num_digits * DIGIT_HISTORY_SIZE * sizeof(int));
  }
  
  digit_history_counts_ = (uint8_t*)heap_caps_malloc(num_digits * sizeof(uint8_t), caps);
  if (!digit_history_counts_) {
       digit_history_counts_ = (uint8_t*)malloc(num_digits * sizeof(uint8_t));
  }
  
  digit_history_heads_ = (uint8_t*)heap_caps_malloc(num_digits * sizeof(uint8_t), caps);
  if (!digit_history_heads_) {
       digit_history_heads_ = (uint8_t*)malloc(num_digits * sizeof(uint8_t));
  }
  
  if (digit_history_data_ && digit_history_counts_ && digit_history_heads_) {
      memset(digit_history_counts_, 0, num_digits * sizeof(uint8_t));
      memset(digit_history_heads_, 0, num_digits * sizeof(uint8_t));
      digit_history_num_digits_ = num_digits;
      ESP_LOGI(TAG, "Allocated digit history for %d digits (PSRAM preferred)", (int)num_digits);
  } else {
      ESP_LOGE(TAG, "Failed to allocate digit history!");
      free_digit_history(); // Cleanup partials
  }
}

int ValueValidator::get_stable_digit(int digit_index, int new_digit) {
  if (digit_index >= digit_history_num_digits_ || !digit_history_data_) return new_digit;
  
  // Access buffers
  int* history = &digit_history_data_[digit_index * DIGIT_HISTORY_SIZE];
  uint8_t& count = digit_history_counts_[digit_index];
  uint8_t& head = digit_history_heads_[digit_index];
  
  // Push new digit into circular buffer
  history[head] = new_digit;
  head = (head + 1) % DIGIT_HISTORY_SIZE;
  if (count < DIGIT_HISTORY_SIZE) count++;
  
  // Find the mode (most frequent digit) in O(N)
  int digit_counts[10] = {0};
  
  for (int i = 0; i < count; i++) {
      size_t idx = (head + DIGIT_HISTORY_SIZE - count + i) % DIGIT_HISTORY_SIZE;
      int val = history[idx];
      
      if (val >= 0 && val < 10) {
          digit_counts[val]++;
      }
  }
  
  int max_freq = 0;
  int mode_val = new_digit; // Default to the new digit
  
  for (int i = 0; i < 10; i++) {
      if (digit_counts[i] > max_freq) {
          max_freq = digit_counts[i];
          mode_val = i;
      } else if (digit_counts[i] == max_freq && i == new_digit) {
          // Tie-breaking: prefer the new digit if it has equal max frequency
          mode_val = new_digit; 
      }
  }
  
  return mode_val;
}

}  // namespace value_validator
}  // namespace esphome
