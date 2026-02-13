/**
 * @file value_validator.cpp
 * @brief Implementation of output validation for meter readings.
 */

#include "value_validator.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <esp_heap_caps.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <numeric>

namespace esphome {
namespace value_validator {

static const char *const TAG = "value_validator";

// PSRAM allocation with internal-RAM fallback
static void *psram_alloc(size_t size) {
  void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p ? p : malloc(size);
}

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
  size_t max_count_by_bytes = max_history_size_bytes_ / sizeof(HistoricalReading);
  static constexpr size_t MAX_DAY_COUNT = 1440; // 24h * 60min
  
  size_t target = std::min(max_count_by_bytes, MAX_DAY_COUNT);
  static constexpr size_t MIN_HISTORY_ENTRIES = 10;
  if (target < MIN_HISTORY_ENTRIES) target = MIN_HISTORY_ENTRIES;
  
  buffer_ = static_cast<HistoricalReading *>(psram_alloc(target * sizeof(HistoricalReading)));
  
  if (buffer_) {
      capacity_ = target;
      head_ = 0;
      count_ = 0;
      ESP_LOGI(TAG, "Allocated history buffer for %d entries (PSRAM preferred)", static_cast<int>(target));
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

int ReadingHistory::get_hour_median() const { return get_median_within_ms(3600000UL); }
int ReadingHistory::get_day_median() const { return get_median_within_ms(86400000UL); }

int ReadingHistory::get_median_within_ms(uint32_t max_elapsed_ms) const {
  if (count_ == 0 || !buffer_) return 0;
  
  size_t last_idx = (head_ + capacity_ - 1) % capacity_;
  uint32_t now = buffer_[last_idx].timestamp;

  std::vector<int> values;
  values.reserve(count_);
  size_t curr = last_idx;
  for (size_t i = 0; i < count_; i++) {
      // Unsigned subtraction handles millis() overflow correctly
      uint32_t elapsed = now - buffer_[curr].timestamp;
      if (elapsed > max_elapsed_ms) break;
      values.push_back(buffer_[curr].value);
      
      if (curr == 0) curr = capacity_ - 1;
      else curr--;
  }
  
  if (values.empty()) return 0;

  size_t size = values.size();  
  std::nth_element(values.begin(), values.begin() + size / 2, values.end());
  int median = values[size / 2];
  
  if (size % 2 == 0) {
      auto max_it = std::max_element(values.begin(), values.begin() + size / 2);
      return (*max_it + median) / 2;
  }
  return median;
}

std::vector<int> ReadingHistory::get_recent_readings(size_t count) const {
  std::vector<int> recent;
  if (count_ == 0 || !buffer_) return recent;
  
  size_t actual_count = std::min(count, count_);
  recent.reserve(actual_count);
  
  for (size_t k = 0; k < actual_count; k++) {
      // Chronological order: oldest first
      size_t offset = actual_count - 1 - k;
      size_t idx = (head_ + capacity_ - 1 - offset) % capacity_;
      recent.push_back(buffer_[idx].value);
  }
  
  return recent;
}

std::vector<std::pair<int, float>> ReadingHistory::get_recent_readings_with_confidence(size_t count) const {
  std::vector<std::pair<int, float>> recent;
  if (count_ == 0 || !buffer_) return recent;
  
  size_t actual_count = std::min(count, count_);
  recent.reserve(actual_count);
  
  for (size_t k = 0; k < actual_count; k++) {
      size_t offset = actual_count - 1 - k;
      size_t idx = (head_ + capacity_ - 1 - offset) % capacity_;
      recent.push_back({buffer_[idx].value, buffer_[idx].confidence});
  }
  
  return recent;
}

void ReadingHistory::clear() {
  head_ = 0;
  count_ = 0;
}

void ValueValidator::setup() {
  this->history_.set_max_history_size_bytes(this->config_.max_history_size_bytes);
  this->history_.setup();
  this->last_valid_reading_ = 0;
  this->first_reading_ = true;
  this->first_reading_count_ = 0;
  this->first_reading_candidate_ = 0;
  this->last_good_values_count_ = 0;
  this->last_good_values_head_ = 0;
  this->consecutive_rejections_ = 0;
  this->rejection_confidence_sum_ = 0.0f;
  this->free_digit_history();
  
  // Initialize with some capacity
  this->ensure_last_good_values_capacity(this->config_.smart_validation_window);
  
  // Restore persistent state if enabled
  if (this->config_.persist_state) {
    this->pref_ = global_preferences->make_preference<int>(fnv1_hash("value_validator"));
    int restored = 0;
    if (this->pref_.load(&restored) && restored > 0) {
      this->last_valid_reading_ = restored;
      this->first_reading_ = false;
      // Seed good values with restored reading
      for (int i = 0; i < this->config_.smart_validation_window; i++) {
        this->add_good_value(restored);
      }
      ESP_LOGI(TAG, "Restored persistent state: last_valid_reading = %d", restored);
    }
  }
}

void ValueValidator::dump_config() {
  ESP_LOGCONFIG(TAG, "Value Validator:");
  ESP_LOGCONFIG(TAG, "  Max Absolute Diff: %d", this->config_.max_absolute_diff);
  ESP_LOGCONFIG(TAG, "  Max Rate Change: %.0f%%", this->config_.max_rate_change * 100.0f);
  ESP_LOGCONFIG(TAG, "  Allow Negative Rates: %s", YESNO(this->config_.allow_negative_rates));
  ESP_LOGCONFIG(TAG, "  Strict Confidence Check: %s", YESNO(this->config_.strict_confidence_check));
  ESP_LOGCONFIG(TAG, "  Per Digit Conf Threshold: %.2f", this->config_.per_digit_confidence_threshold);
  ESP_LOGCONFIG(TAG, "  Max Consecutive Rejections: %d", this->config_.max_consecutive_rejections);
  ESP_LOGCONFIG(TAG, "  Small Negative Tolerance: %d", this->config_.small_negative_tolerance);
  ESP_LOGCONFIG(TAG, "  Persist State: %s", YESNO(this->config_.persist_state));
  ESP_LOGCONFIG(TAG, "  Max History Size: %d bytes", (int)this->config_.max_history_size_bytes);
}

bool ValueValidator::validate_reading(int new_reading, float confidence, int& validated_reading) {
  uint32_t current_time = millis();
  
  // Publish raw reading diagnostic
  if (this->raw_reading_sensor_) {
    this->raw_reading_sensor_->publish_state(new_reading);
  }
  
  // Capture last confidence before adding the new reading
  float last_confidence = this->history_.get_last_confidence();

  // Always add to history for tracking
  this->history_.add_reading(new_reading, current_time, confidence);
  
  // First reading: require high confidence or 3 consistent readings
  if (this->first_reading_) {
    if (confidence >= this->config_.high_confidence_threshold) {
      // High confidence — accept immediately
      this->last_valid_reading_ = new_reading;
      this->first_reading_ = false;
      this->first_reading_count_ = 0;
      validated_reading = new_reading;
      
      this->last_good_values_count_ = 0;
      this->last_good_values_head_ = 0;
      this->add_good_value(new_reading);
      
      ESP_LOGI(TAG, "First reading accepted (high confidence): %d (confidence: %.2f)", new_reading, confidence);
      this->publish_diagnostics_("normal");
      this->save_state_();
      return true;
    }
    
    // Low confidence — require consistency
    if (new_reading == this->first_reading_candidate_) {
      this->first_reading_count_++;
    } else {
      this->first_reading_candidate_ = new_reading;
      this->first_reading_count_ = 1;
    }
    
    if (this->first_reading_count_ >= 3) {
      // 3 consistent readings — accept
      this->last_valid_reading_ = new_reading;
      this->first_reading_ = false;
      this->first_reading_count_ = 0;
      validated_reading = new_reading;
      
      this->last_good_values_count_ = 0;
      this->last_good_values_head_ = 0;
      this->add_good_value(new_reading);
      
      ESP_LOGI(TAG, "First reading accepted (3 consistent): %d (confidence: %.2f)", new_reading, confidence);
      this->publish_diagnostics_("normal");
      this->save_state_();
      return true;
    }
    
    ESP_LOGW(TAG, "First reading %d rejected (conf: %.2f < %.2f, consistent: %d/3)", 
             new_reading, confidence, this->config_.high_confidence_threshold, this->first_reading_count_);
    this->publish_diagnostics_("initializing");
    return false;
  }
  
  // Apply smart validation
  validated_reading = this->apply_smart_validation(new_reading, confidence, last_confidence);
  
  bool is_valid = (validated_reading == new_reading);
  
  if (is_valid) {
    // Update last good values
    this->add_good_value(new_reading);
    this->last_valid_reading_ = new_reading;
    this->save_state_();
    this->publish_diagnostics_("normal");
  } else {
    // Publish diagnostic state based on rejection count
    if (this->consecutive_rejections_ >= this->config_.max_consecutive_rejections) {
      this->publish_diagnostics_("stuck");
    } else if (this->consecutive_rejections_ > 0) {
      this->publish_diagnostics_("rejecting");
    }
  }
  
  if (this->debug_) {
    ESP_LOGD(TAG, "Validation: %d -> %d (valid: %s, confidence: %.2f)", 
             new_reading, validated_reading, is_valid ? "yes" : "no", confidence);
  }
  
  return is_valid;
}

bool ValueValidator::validate_reading(const std::vector<float>& digits, const std::vector<float>& confidences, int& validated_reading) {
  // If we don't have per-digit history yet, fallback to standard validation
  
  std::vector<int> current_digits;
  current_digits.reserve(digits.size());
  
  for (float d : digits) {
      int digit = static_cast<int>(std::round(d));
      if (digit >= 10) digit = 0; // Wrap 10 -> 0 standard TFLite behavior
      current_digits.push_back(digit);
  }
  
  // --- Per-Digit History & Stability Filter ---
  // Process each digit through the history filter to correct flickering.
  ensure_digit_history_size(current_digits.size());
  
  int raw_val = 0;
  for (size_t i = 0; i < current_digits.size(); i++) {
      int stable_d = get_stable_digit(i, current_digits[i]);
      current_digits[i] = stable_d;
      raw_val = raw_val * 10 + stable_d;
  }
  
  // Calculate average confidence for the legacy validator call
  float avg_conf = 0.0f;
  if (!confidences.empty()) {
      avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / static_cast<float>(confidences.size());
  }

  // --- Per-Digit Filtering ---
  int filtered_val = raw_val;
  bool final_valid_check = true; // Assumed valid unless strict check fails
  
  if (!first_reading_ && last_valid_digits_count_ != current_digits.size()) {
      ESP_LOGW(TAG, "Digit count changed: %d -> %d. Resetting digit history.",
               static_cast<int>(last_valid_digits_count_), static_cast<int>(current_digits.size()));
  }
  if (!first_reading_ && last_valid_digits_count_ == current_digits.size()) {
      std::string filtered_digit_string;
      bool modified = false;
      
      for (size_t i = 0; i < current_digits.size(); i++) {
          int new_d = current_digits[i];
          int old_d = last_valid_digits_data_[i];
          float conf = (i < confidences.size()) ? confidences[i] : 0.0f;
          
          if (new_d != old_d) {
              // Digit changed. Check confidence.
              ESP_LOGD(TAG, "Digit %d changed (%d -> %d). Conf: %.2f vs Threshold: %.2f", 
                       static_cast<int>(i), old_d, new_d, conf, config_.per_digit_confidence_threshold);
                       
              if (conf < config_.per_digit_confidence_threshold) {
                  // Confidence too low for a change. Reject it.
                  ESP_LOGW(TAG, "Digit %d change rejected (Val: %d->%d, Conf: %.2f < %.2f)", 
                           static_cast<int>(i), old_d, new_d, conf, config_.per_digit_confidence_threshold);
                  filtered_digit_string += std::to_string(old_d);
                  modified = true;
              } else {
                  // Accepted
                  if (debug_) {
                     ESP_LOGD(TAG, "Digit %d changed (%d -> %d) - Accepted (Conf: %.2f >= %.2f)", 
                              static_cast<int>(i), old_d, new_d, conf, config_.per_digit_confidence_threshold);
                  }
                  filtered_digit_string += std::to_string(new_d);
              }
          } else {
              // Unchanged match
              if (debug_) {
                 ESP_LOGD(TAG, "Digit %d unchanged (%d). Conf: %.2f", static_cast<int>(i), new_d, conf);
              }
              // Strict check: if high_confidence_threshold is configured (per_digit_confidence_threshold),
              // we require even unchanged digits to meet it IF strict mode is enabled.
              // User request: "I want all digit to be upper".
              if (config_.strict_confidence_check && conf < config_.per_digit_confidence_threshold) {
                  ESP_LOGW(TAG, "Digit %d unchanged but low confidence (Conf: %.2f < %.2f) - Rejecting reading in strict mode", 
                           static_cast<int>(i), conf, config_.per_digit_confidence_threshold);
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
             filtered_val = static_cast<int>(val);
             ESP_LOGD(TAG, "Per-digit filter modified reading: %d -> %d", raw_val, filtered_val);
          } else {
             filtered_val = raw_val; // Fallback
          }
      }
      
      if (debug_ && !modified) {
          ESP_LOGD(TAG, "Per-digit filter: No changes needed. (Raw: %d)", raw_val);
      }
  } else {
       // First reading or no history 
       
       if (config_.strict_confidence_check) {
             ESP_LOGD(TAG, "First reading (Strict Mode): Checking %d digits against %.2f", 
                      static_cast<int>(current_digits.size()), config_.per_digit_confidence_threshold);
            
            for (size_t i = 0; i < current_digits.size(); i++) {
                float conf = (i < confidences.size()) ? confidences[i] : 0.0f;
                // ... same logic ...
                if (conf < config_.per_digit_confidence_threshold) {
                     ESP_LOGW(TAG, "Initial reading digit %d low confidence (Conf: %.2f < %.2f) - Rejecting", 
                              static_cast<int>(i), conf, config_.per_digit_confidence_threshold);
                     final_valid_check = false;
                }
            }
       } else {
           // Strict check disabled: Accept first reading provided digits are valid (checked above)
           ESP_LOGD(TAG, "First reading: Strict check disabled, accepting.");
       }
  }
  
  if (!final_valid_check) {
      // Strict per-digit check failed. Return false immediately to prevent invalid state.
      return false;
  }

  // Check for Hallucination Patterns (e.g. 11111111)
  if (is_hallucination_pattern(digits)) { // Use the original float digits to check pattern
      ESP_LOGW(TAG, "Rejected reading due to hallucination pattern (e.g. all identical digits)");
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
      
      // Pad with leading zeros if necessary
      if (val_str.length() < current_digits.size()) {
          val_str = std::string(current_digits.size() - val_str.length(), '0') + val_str;
      }
      
      ensure_last_valid_digits_size(val_str.length());
      if (last_valid_digits_data_) {
          for (size_t i = 0; i < val_str.length(); i++) {
              last_valid_digits_data_[i] = val_str[i] - '0';
          }
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
  
  // Check rate of change (relative to last reading)
  if (last_reading > 0 && config_.max_rate_change > 0.0f) {
    float rate = static_cast<float>(absolute_diff) / static_cast<float>(last_reading);
    if (rate > config_.max_rate_change) {
      ESP_LOGW(TAG, "Rate of change too high: %.1f%% (max: %.1f%%), %d -> %d",
               rate * 100.0f, config_.max_rate_change * 100.0f, last_reading, new_reading);
      return false;
    }
  }
  
  // Check for negative rates if not allowed
  if (!config_.allow_negative_rates && new_reading < last_reading) {
    // Allow small negative fluctuations (meter reset or small calibration issues)
    int negative_diff = last_reading - new_reading;
    if (negative_diff > config_.small_negative_tolerance) {
      ESP_LOGW(TAG, "Negative rate detected: %d -> %d (diff: %d, tolerance: %d)", 
               last_reading, new_reading, negative_diff, config_.small_negative_tolerance);
    return false;
    }
  }
  
  return true;
}

int ValueValidator::apply_smart_validation(int new_reading, float confidence, float last_confidence) {
  // Basic digit plausibility check
  if (is_digit_plausible(new_reading, last_valid_reading_)) {
    if (this->debug_) {
        ESP_LOGD(TAG, "SmartValidation: Reading %d is plausible (Last: %d). Accepted.", new_reading, last_valid_reading_);
    }
    consecutive_rejections_ = 0;
    rejection_confidence_sum_ = 0.0f;
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
           // If consistent but negative, check if we should self-correct
           if (!config_.allow_negative_rates && new_reading < last_valid_reading_) {
               int negative_diff = last_valid_reading_ - new_reading;
               if (negative_diff <= config_.small_negative_tolerance) {
                   // Small fluctuation — always allowed
               } else if (consecutive_rejections_ >= config_.max_consecutive_rejections) {
                   // Too many consecutive rejections with high confidence — likely last_valid was wrong
                   float avg_rejection_conf = (consecutive_rejections_ > 0) ? 
                       (rejection_confidence_sum_ / consecutive_rejections_) : 0.0f;
                   if (avg_rejection_conf >= config_.high_confidence_threshold) {
                       ESP_LOGW(TAG, "Self-correcting: %d consecutive high-confidence rejections (avg conf: %.2f >= %.2f). "
                                "Accepting consistent value %d (was %d)",
                                consecutive_rejections_, avg_rejection_conf, config_.high_confidence_threshold,
                                new_reading, last_valid_reading_);
                       consecutive_rejections_ = 0;
                       rejection_confidence_sum_ = 0.0f;
                       // Fall through to accept
                   } else {
                       ESP_LOGW(TAG, "Consistent negative reading rejected - avg confidence too low "
                                "(%.2f < %.2f): %d -> %d", 
                                avg_rejection_conf, config_.high_confidence_threshold,
                                last_valid_reading_, new_reading);
                       consecutive_rejections_++;
                       rejection_confidence_sum_ += confidence;
                       return last_valid_reading_;
                   }
               } else {
                   ESP_LOGW(TAG, "Consistent negative reading rejected (%d/%d): %d -> %d",
                            consecutive_rejections_, config_.max_consecutive_rejections,
                            last_valid_reading_, new_reading);
                   consecutive_rejections_++;
                   rejection_confidence_sum_ += confidence;
                   return last_valid_reading_;
               }
           }

           ESP_LOGW(TAG, "Large change confirmed by %d precise consecutive readings: %d -> %d", 
                    (int)CONSISTENCY_COUNT, last_valid_reading_, new_reading);
           
           // If the deviation is huge, clear old history to prevent it from dragging us back
           if (std::abs(new_reading - last_valid_reading_) > (config_.max_absolute_diff * 5)) {
               last_good_values_count_ = 0;
           }
           consecutive_rejections_ = 0;
           rejection_confidence_sum_ = 0.0f;
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
      consecutive_rejections_ = 0;
      rejection_confidence_sum_ = 0.0f;
      return plausible_reading;
    }
  }
    
  // 2. If no good historical reading, use median of recent GOOD values
  if (last_good_values_count_ > 0) {
    int median = get_good_values_median();
    
    // Only use median if it's somewhat close to the last valid
    if (std::abs(median - last_valid_reading_) <= config_.max_absolute_diff) {
      ESP_LOGD(TAG, "Using median of last good values: %d (was: %d)", median, new_reading);
      consecutive_rejections_ = 0;
      rejection_confidence_sum_ = 0.0f;
      return median;
    }
  }
    
  // 3. Last resort: use last valid reading — this is a rejection
  consecutive_rejections_++;
  rejection_confidence_sum_ += confidence;
  
  // Self-correction for fluctuating readings:
  // When readings alternate between OCR variants (e.g. 256517/256617), the consistency 
  // check (3 identical) never passes. After enough rejections with high confidence,
  // find the consensus (most frequent) value from recent history, weighted by confidence.
  if (consecutive_rejections_ >= config_.max_consecutive_rejections) {
      float avg_conf = (consecutive_rejections_ > 0) ? 
          (rejection_confidence_sum_ / consecutive_rejections_) : 0.0f;
      if (avg_conf >= config_.high_confidence_threshold) {
          // Find the confidence-weighted best reading from recent history
          auto recent_all = history_.get_recent_readings_with_confidence(config_.max_consecutive_rejections);
          if (!recent_all.empty()) {
              // Accumulate confidence weight per unique value
              // Use simple parallel arrays to avoid std::map on ESP32
              std::vector<int> unique_vals;
              std::vector<float> conf_sums;
              std::vector<int> counts;
              
              for (auto& pair : recent_all) {
                  bool found = false;
                  for (size_t i = 0; i < unique_vals.size(); i++) {
                      if (unique_vals[i] == pair.first) {
                          conf_sums[i] += pair.second;
                          counts[i]++;
                          found = true;
                          break;
                      }
                  }
                  if (!found) {
                      unique_vals.push_back(pair.first);
                      conf_sums.push_back(pair.second);
                      counts.push_back(1);
                  }
              }
              
              // Pick value with highest total confidence weight
              int best_idx = 0;
              for (size_t i = 1; i < unique_vals.size(); i++) {
                  if (conf_sums[i] > conf_sums[best_idx]) {
                      best_idx = i;
                  }
              }
              
              int best_val = unique_vals[best_idx];
              int best_count = counts[best_idx];
              float best_conf = conf_sums[best_idx];
              
              ESP_LOGW(TAG, "Self-correcting via consensus: %d consecutive rejections (avg conf: %.2f >= %.2f). "
                       "Consensus value %d (seen %d/%d, total conf weight: %.1f). Was stuck at %d",
                       consecutive_rejections_, avg_conf, config_.high_confidence_threshold,
                       best_val, best_count, (int)recent_all.size(), best_conf, last_valid_reading_);
              consecutive_rejections_ = 0;
              rejection_confidence_sum_ = 0.0f;
              return best_val;
          }
      }
  }
  
  ESP_LOGW(TAG, "No plausible alternative found, keeping last valid: %d (Ignored: %d, rejections: %d/%d)", 
           last_valid_reading_, new_reading, consecutive_rejections_, config_.max_consecutive_rejections);
  return last_valid_reading_;
}


int ValueValidator::find_most_plausible_reading(int new_reading, const std::vector<int>& recent_readings) {
  if (recent_readings.empty()) return new_reading;
  
  // Calculate average of recent readings (use int64_t to avoid overflow for large meter values)
  int64_t sum = 0;
  for (int reading : recent_readings) {
    sum += reading;
  }
  int average = static_cast<int>(sum / static_cast<int64_t>(recent_readings.size()));
  
  // Check if average is more plausible
  if (is_digit_plausible(average, last_valid_reading_)) {
    return average;
  }
  
  // Check median using nth_element (O(N) vs O(N log N) sort)
  std::vector<int> sorted_readings = recent_readings;
  std::nth_element(sorted_readings.begin(), sorted_readings.begin() + sorted_readings.size() / 2, sorted_readings.end());
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
  this->history_.clear();
  last_valid_reading_ = 0;
  first_reading_ = true;
  first_reading_count_ = 0;
  first_reading_candidate_ = 0;
  last_good_values_count_ = 0;
  last_good_values_head_ = 0;
  consecutive_rejections_ = 0;
  rejection_confidence_sum_ = 0.0f;
  free_digit_history();
  last_valid_digits_count_ = 0;
  publish_diagnostics_("initializing");
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
  this->history_.clear();
  consecutive_rejections_ = 0;
  rejection_confidence_sum_ = 0.0f;
  ESP_LOGW(TAG, "Manually set last valid reading to: %d", value);
}

void ValueValidator::set_last_valid_reading(const std::string &value) {
  if (value.empty()) {
    ESP_LOGW(TAG, "Empty string passed to set_last_valid_reading, ignoring.");
    return;
  }
  
  for (char c : value) {
    if (!isdigit(c)) {
      ESP_LOGE(TAG, "Invalid characters in manual value string. Only digits are allowed. Got: '%s'", value.c_str());
      return;
    }
  }

  int int_val = 0;
  char* end = nullptr;
  long val = strtol(value.c_str(), &end, 10);
  if (end != value.c_str() + value.length()) {
      ESP_LOGE(TAG, "Failed to parse complete manual value string: %s", value.c_str());
      return;
  }
  int_val = static_cast<int>(val);

  // Set the integer value
  last_valid_reading_ = int_val;
  first_reading_ = false;

  // Use the STRING length for digit count, preserving leading zeros
  ensure_last_valid_digits_size(value.length());
  for (size_t i = 0; i < value.length(); i++) {
    last_valid_digits_data_[i] = value[i] - '0';
  }
  
  // Create a "fake" history for this value
  last_good_values_count_ = 0;
  for(int i=0; i<config_.smart_validation_window; i++) {
     add_good_value(int_val);
  }
  
  this->history_.clear();
  consecutive_rejections_ = 0;
  rejection_confidence_sum_ = 0.0f;
  ESP_LOGW(TAG, "Manually set last valid reading to: %d (Digits: %d, Str: %s)", int_val, static_cast<int>(value.length()), value.c_str());
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
  
  // Allocate new buffer first to avoid data loss on failure
  int *new_data = static_cast<int *>(psram_alloc(num_digits * sizeof(int)));
  
  if (new_data) {
      if (last_valid_digits_data_) free(last_valid_digits_data_);
      last_valid_digits_data_ = new_data;
      last_valid_digits_count_ = num_digits;
  } else {
      // Allocation failed — keep existing data if available
      ESP_LOGE(TAG, "Failed to allocate memory for last valid digits!");
  }
}

void ValueValidator::ensure_last_good_values_capacity(size_t capacity) {
  if (capacity == last_good_values_capacity_ && last_good_values_data_) return;
  
  // Allocate new buffer first to avoid data loss on failure
  int *new_data = static_cast<int *>(psram_alloc(capacity * sizeof(int)));
  
  if (new_data) {
      if (last_good_values_data_) free(last_good_values_data_);
      last_good_values_data_ = new_data;
      last_good_values_capacity_ = capacity;
  } else {
      ESP_LOGE(TAG, "Failed to allocate memory for last good values!");
  }
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
  
  // Copy to temp vector for nth_element
  std::vector<int> values;
  values.reserve(last_good_values_count_);
  size_t curr = (last_good_values_head_ + last_good_values_capacity_ - 1) % last_good_values_capacity_;
  for (size_t i = 0; i < last_good_values_count_; i++) {
      values.push_back(last_good_values_data_[curr]);
      if (curr == 0) curr = last_good_values_capacity_ - 1;
      else curr--;
  }
  
  std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
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
  
  digit_history_data_ = static_cast<int *>(psram_alloc(num_digits * DIGIT_HISTORY_SIZE * sizeof(int)));
  digit_history_counts_ = static_cast<uint8_t *>(psram_alloc(num_digits * sizeof(uint8_t)));
  digit_history_heads_ = static_cast<uint8_t *>(psram_alloc(num_digits * sizeof(uint8_t)));
  
  if (digit_history_data_ && digit_history_counts_ && digit_history_heads_) {
      memset(digit_history_counts_, 0, num_digits * sizeof(uint8_t));
      memset(digit_history_heads_, 0, num_digits * sizeof(uint8_t));
      digit_history_num_digits_ = num_digits;
      ESP_LOGI(TAG, "Allocated digit history for %d digits (PSRAM preferred)", static_cast<int>(num_digits));
  } else {
      ESP_LOGE(TAG, "Failed to allocate digit history!");
      free_digit_history(); // Cleanup partials
  }
}

int ValueValidator::get_stable_digit(size_t digit_index, int new_digit) {

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

bool ValueValidator::is_hallucination_pattern(const std::vector<float>& digits) const {
  if (digits.size() < 2) return false;

  // Single pass: check all-identical AND compute integer value
  int first = static_cast<int>(std::round(digits[0]));
  bool all_identical = true;
  int val = first;
  
  for (size_t i = 1; i < digits.size(); i++) {
      int d = static_cast<int>(std::round(digits[i]));
      if (d != first) all_identical = false;
      val = val * 10 + d;
  }
  
  if (!all_identical) return false;
  
  // All digits identical — check if it matches last valid reading (then it's real)
  if (!first_reading_ && val == last_valid_reading_) {
      return false;
  }

  // Otherwise, "All X's" is highly likely a hallucination
  return true;
}

void ValueValidator::publish_diagnostics_(const char* state) {
  if (rejection_count_sensor_) {
    rejection_count_sensor_->publish_state(consecutive_rejections_);
  }
  if (validator_state_sensor_) {
    validator_state_sensor_->publish_state(state);
  }
}

void ValueValidator::save_state_() {
  if (config_.persist_state && last_valid_reading_ >= 0) {
    this->pref_.save(&last_valid_reading_);
  }
}

}  // namespace value_validator
}  // namespace esphome
