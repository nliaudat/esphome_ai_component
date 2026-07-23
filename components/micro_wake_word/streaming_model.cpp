#include "streaming_model.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

static const char *const TAG = "micro_wake_word";

namespace esphome::micro_wake_word {

void WakeWordModel::log_model_config() {
  ESP_LOGCONFIG(TAG,
                "    - Wake Word: %s\n"
                "      Probability cutoff: %.2f\n"
                "      Sliding window size: %d",
                this->wake_word_.c_str(), this->probability_cutoff_ / 255.0f, this->sliding_window_size_);
}

void VADModel::log_model_config() {
  ESP_LOGCONFIG(TAG,
                "    - VAD Model\n"
                "      Probability cutoff: %.2f\n"
                "      Sliding window size: %d",
                this->probability_cutoff_ / 255.0f, this->sliding_window_size_);
}

bool StreamingModel::load_model_() {
  RAMAllocator<uint8_t> arena_allocator;

  // Allocate variable arena for MRV (MicroResourceVariables) if not yet allocated
  if (this->var_arena_ == nullptr) {
    this->var_arena_ = arena_allocator.allocate(STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    if (this->var_arena_ == nullptr) {
      ESP_LOGE(TAG, "Could not allocate the streaming model's variable tensor arena.");
      return false;
    }
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
  }

  // Probe for the actual required tensor arena size if not yet determined
  if (!this->tensor_arena_size_probed_) {
    size_t probed_size = this->probe_arena_size_();
    if (probed_size > 0) {
      ESP_LOGD(TAG, "Probed tensor arena size: %zu bytes", probed_size);
      this->tensor_arena_size_ = probed_size;
    } else {
      ESP_LOGW(TAG, "Arena size probe failed, using manifest size: %zu bytes", this->tensor_arena_size_);
    }
    this->tensor_arena_size_probed_ = true;
  }

  // Allocate tensor arena
  if (this->tensor_arena_ == nullptr) {
    this->tensor_arena_ = arena_allocator.allocate(this->tensor_arena_size_);
    if (this->tensor_arena_ == nullptr) {
      ESP_LOGE(TAG, "Could not allocate the streaming model's tensor arena.");
      return false;
    }
  }

  // Delegate model loading to tflite_micro_helper::ModelHandler with MRV support.
  // Model size is passed as tensor_arena_size_ (placeholder) since the model FlatBuffer
  // doesn't need a size to load — tflite::GetModel() reads the buffer directly.
  // model_size is only used for logging/validation in ModelHandler; the tensor arena size
  // is a non-zero placeholder that satisfies the existence check.
  model_config_.description = "micro_wake_word";
  model_config_.output_processing = "direct_class";
  model_config_.scale_factor = 1.0f;

  if (!this->model_handler_.load_model_with_arena(this->model_start_, this->tensor_arena_size_, this->tensor_arena_,
                                                  this->tensor_arena_size_, this->model_config_, this->mrv_)) {
    ESP_LOGE(TAG, "Failed to load streaming model via ModelHandler");
    return false;
  }

  // Verify input tensor dimensions (after ModelHandler has set up the interpreter)
  TfLiteTensor *input = this->model_handler_.input_tensor();
  if (input == nullptr) {
    ESP_LOGE(TAG, "Streaming model input tensor is null after loading.");
    return false;
  }

  // Dimension 2 (data[1]) will represent the first layer stride, so it may vary
  if ((input->dims->size != 3) || (input->dims->data[0] != 1) || (input->dims->data[2] != PREPROCESSOR_FEATURE_SIZE)) {
    ESP_LOGE(TAG, "Streaming model tensor input dimensions are improper.");
    return false;
  }

  if (input->type != kTfLiteInt8) {
    ESP_LOGE(TAG, "Streaming model tensor input is not int8.");
    return false;
  }

  // Verify output tensor dimensions
  TfLiteTensor *output = this->model_handler_.output_tensor();
  if (output == nullptr) {
    ESP_LOGE(TAG, "Streaming model output tensor is null after loading.");
    return false;
  }

  if ((output->dims->size != 2) || (output->dims->data[0] != 1) || (output->dims->data[1] != 1)) {
    ESP_LOGE(TAG, "Streaming model tensor output dimension is not 1x1.");
    return false;
  }

  if (output->type != kTfLiteUInt8) {
    ESP_LOGE(TAG, "Streaming model tensor output is not uint8.");
    return false;
  }

  this->loaded_ = true;
  this->reset_probabilities();
  return true;
}

size_t StreamingModel::probe_arena_size_() {
  RAMAllocator<uint8_t> arena_allocator;

  // Try with the manifest size first, then escalates to 1.5, then 2x if it fails.
  // Different platforms and different versions of the esp-nn library require different amounts of memory,
  // so the manifest size may not always be correct, and probing allows us to find the actual required size
  // for the current build and platform. Aligns test sizes to 16 bytes.
  size_t attempt_sizes[] = {(this->tensor_arena_size_ + 15) & ~15, (this->tensor_arena_size_ * 3 / 2 + 15) & ~15,
                            (this->tensor_arena_size_ * 2 + 15) & ~15};

  // Build a local temporary op resolver just for probing
  tflite::MicroMutableOpResolver<20> probe_resolver;
  if (!register_probe_ops_(probe_resolver)) {
    ESP_LOGE(TAG, "Failed to register probe ops for arena size probing");
    return 0;
  }

  for (size_t attempt_size : attempt_sizes) {
    uint8_t *probe_arena = arena_allocator.allocate(attempt_size);
    if (probe_arena == nullptr) {
      continue;
    }

    // Verify the model works at all with this arena size
    auto probe_interpreter = make_unique<tflite::MicroInterpreter>(tflite::GetModel(this->model_start_), probe_resolver,
                                                                   probe_arena, attempt_size, this->mrv_);

    if (probe_interpreter->AllocateTensors() != kTfLiteOk) {
      probe_interpreter.reset();
      arena_allocator.deallocate(probe_arena, attempt_size);
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
      continue;
    }

    // Try to shrink the arena. Start with arena_used_bytes() + 16 (rounded to 16-byte alignment).
    size_t lower = (probe_interpreter->arena_used_bytes() + 16 + 15) & ~15;
    probe_interpreter.reset();
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

    size_t upper = attempt_size;

    while (lower < upper) {
      auto test_interpreter = make_unique<tflite::MicroInterpreter>(tflite::GetModel(this->model_start_),
                                                                    probe_resolver, probe_arena, lower, this->mrv_);

      bool ok = test_interpreter->AllocateTensors() == kTfLiteOk;

      test_interpreter.reset();
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

      if (ok) {
        // Found a working size smaller than the full arena
        upper = lower + 16;  // Pad by 16 bytes to be safe for future allocations
        break;
      }

      // Try the midpoint between current attempt and full size
      lower = ((lower + upper) / 2 + 15) & ~15;
    }

    arena_allocator.deallocate(probe_arena, attempt_size);
    return upper;
  }

  return 0;
}

bool StreamingModel::register_probe_ops_(tflite::MicroMutableOpResolver<20> &op_resolver) {
  if (op_resolver.AddCallOnce() != kTfLiteOk)
    return false;
  if (op_resolver.AddVarHandle() != kTfLiteOk)
    return false;
  if (op_resolver.AddReshape() != kTfLiteOk)
    return false;
  if (op_resolver.AddReadVariable() != kTfLiteOk)
    return false;
  if (op_resolver.AddStridedSlice() != kTfLiteOk)
    return false;
  if (op_resolver.AddConcatenation() != kTfLiteOk)
    return false;
  if (op_resolver.AddAssignVariable() != kTfLiteOk)
    return false;
  if (op_resolver.AddConv2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddMul() != kTfLiteOk)
    return false;
  if (op_resolver.AddAdd() != kTfLiteOk)
    return false;
  if (op_resolver.AddMean() != kTfLiteOk)
    return false;
  if (op_resolver.AddFullyConnected() != kTfLiteOk)
    return false;
  if (op_resolver.AddLogistic() != kTfLiteOk)
    return false;
  if (op_resolver.AddQuantize() != kTfLiteOk)
    return false;
  if (op_resolver.AddDepthwiseConv2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddAveragePool2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddMaxPool2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddPad() != kTfLiteOk)
    return false;
  if (op_resolver.AddPack() != kTfLiteOk)
    return false;
  if (op_resolver.AddSplitV() != kTfLiteOk)
    return false;

  return true;
}

void StreamingModel::unload_model() {
  // Delegate to ModelHandler to free the interpreter and arena
  this->model_handler_.unload();

  // Free variable arena and MRV resources (unique to streaming)
  RAMAllocator<uint8_t> arena_allocator;

  if (this->tensor_arena_ != nullptr) {
    arena_allocator.deallocate(this->tensor_arena_, this->tensor_arena_size_);
    this->tensor_arena_ = nullptr;
  }

  if (this->var_arena_ != nullptr) {
    arena_allocator.deallocate(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->var_arena_ = nullptr;
  }

  this->mrv_ = nullptr;
  this->ma_ = nullptr;
  this->loaded_ = false;
}

bool StreamingModel::perform_streaming_inference(const int8_t features[PREPROCESSOR_FEATURE_SIZE]) {
  if (this->enabled_ && !this->loaded_) {
    // Model is enabled but isn't loaded
    if (!this->load_model_()) {
      return false;
    }
  }

  if (!this->enabled_ && this->loaded_) {
    // Model is disabled but still loaded
    this->unload_model();
    return true;
  }

  if (this->loaded_) {
    TfLiteTensor *input = this->model_handler_.input_tensor();

    uint8_t stride = input->dims->data[1];
    this->current_stride_step_ = this->current_stride_step_ % stride;

    std::memmove(
        (int8_t *) (tflite::GetTensorData<int8_t>(input)) + PREPROCESSOR_FEATURE_SIZE * this->current_stride_step_,
        features, PREPROCESSOR_FEATURE_SIZE);
    ++this->current_stride_step_;

    if (this->current_stride_step_ >= stride) {
      TfLiteStatus invoke_status = this->model_handler_.invoke();
      if (invoke_status != kTfLiteOk) {
        ESP_LOGW(TAG, "Streaming interpreter invoke failed");
        return false;
      }

      TfLiteTensor *output = this->model_handler_.output_tensor();

      ++this->last_n_index_;
      if (this->last_n_index_ == this->sliding_window_size_)
        this->last_n_index_ = 0;
      this->recent_streaming_probabilities_[this->last_n_index_] = output->data.uint8[0];  // probability;
      this->unprocessed_probability_status_ = true;
    }
    if (this->recent_streaming_probabilities_[this->last_n_index_] < this->probability_cutoff_) {
      // Only increment ignore windows if less than the probability cutoff; this forces the model to "cool-off" from a
      // previous detection and calling ``reset_probabilities`` so it avoids duplicate detections
      this->ignore_windows_ = std::min(this->ignore_windows_ + 1, 0);
    }
  }
  return true;
}

void StreamingModel::reset_probabilities() {
  for (auto &prob : this->recent_streaming_probabilities_) {
    prob = 0;
  }
  this->ignore_windows_ = -MIN_SLICES_BEFORE_DETECTION;
}

WakeWordModel::WakeWordModel(const std::string &id, const uint8_t *model_start, uint8_t default_probability_cutoff,
                             size_t sliding_window_average_size, const std::string &wake_word, size_t tensor_arena_size,
                             bool default_enabled, bool internal_only) {
  this->id_ = id;
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_average_size;
  this->recent_streaming_probabilities_.resize(sliding_window_average_size, 0);
  this->wake_word_ = wake_word;
  this->tensor_arena_size_ = tensor_arena_size;
  this->current_stride_step_ = 0;
  this->internal_only_ = internal_only;

  this->pref_ = global_preferences->make_preference<bool>(fnv1_hash(id));
  bool enabled;
  if (this->pref_.load(&enabled)) {
    // Use the enabled state loaded from flash
    this->enabled_ = enabled;
  } else {
    // If no state saved, then use the default
    this->enabled_ = default_enabled;
  }
};

void WakeWordModel::enable() {
  this->enabled_ = true;
  if (!this->internal_only_) {
    this->pref_.save(&this->enabled_);
  }
}

void WakeWordModel::disable() {
  this->enabled_ = false;
  if (!this->internal_only_) {
    this->pref_.save(&this->enabled_);
  }
}

DetectionEvent WakeWordModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.wake_word = &this->wake_word_;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if ((this->ignore_windows_ < 0) || !this->enabled_) {
    detection_event.detected = false;
    return detection_event;
  }

  uint32_t sum = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    sum += prob;
  }

  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.detected = sum > this->probability_cutoff_ * this->sliding_window_size_;

  this->unprocessed_probability_status_ = false;
  return detection_event;
}

VADModel::VADModel(const uint8_t *model_start, uint8_t default_probability_cutoff, size_t sliding_window_size,
                   size_t tensor_arena_size) {
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_size;
  this->recent_streaming_probabilities_.resize(sliding_window_size, 0);
  this->tensor_arena_size_ = tensor_arena_size;
}

DetectionEvent VADModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if (!this->enabled_) {
    // We disabled the VAD model for some reason... so we shouldn't block wake words from being detected
    detection_event.detected = true;
    return detection_event;
  }

  uint32_t sum = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    sum += prob;
  }

  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.detected = sum > (this->probability_cutoff_ * this->sliding_window_size_);

  return detection_event;
}

}  // namespace esphome::micro_wake_word

#endif  // USE_ESP32
