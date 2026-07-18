#include "model_handler.h"
#include "esp_log.h"
#include "debug_utils.h"
#include <cmath>
#include <esp_heap_caps.h>
#include <vector>
#include <algorithm>
#include <limits>

namespace esphome {
namespace tflite_micro_helper {

static const char *const TAG = "ModelHandler";

void ModelHandler::unload() {
  this->interpreter_.reset();
  this->tensor_arena_allocation_.data.reset();
  this->tensor_arena_allocation_.actual_size = 0;
  this->tensor_arena_size_requested_ = 0;

  // Reset state
  this->config_ = ModelConfig{};
  this->output_size_ = 0;
  this->tflite_model_ = nullptr;
  this->model_length_ = 0;
  this->memory_manager_ = MemoryManager();  // Reset memory manager state if needed
  this->resolver_.reset();

  ESP_LOGI(TAG, "Model unloaded and memory freed");
}

bool ModelHandler::load_model(const uint8_t *model_data, size_t model_size, const ModelConfig &config) {
  this->model_length_ = model_size;

  // Use configured arena size, or fall back to default
  if (this->tensor_arena_size_requested_ == 0) {
    this->tensor_arena_size_requested_ = 100 * 1024;  // Default 100KB
    ESP_LOGW(TAG, "Using default tensor arena size: 100KB");
  }

  // Allocate tensor arena
  this->tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(this->tensor_arena_size_requested_);
  if (!this->tensor_arena_allocation_) {
    ESP_LOGE(TAG, "Failed to allocate tensor arena");
    return false;
  }

  // Load the model using the allocated arena
  return this->load_model_with_arena(model_data, model_size, this->tensor_arena_allocation_.data.get(),
                                     this->tensor_arena_allocation_.actual_size, config);
}

bool ModelHandler::load_model_with_arena(const uint8_t *model_data, size_t model_size, uint8_t *tensor_arena,
                                         size_t tensor_arena_size, const ModelConfig &config,
                                         tflite::MicroResourceVariables *mrv) {
  if (this->debug_) {
    ESP_LOGD(TAG, "Loading model with config:");
    ESP_LOGD(TAG, "  Description: %s", config.description.c_str());
    ESP_LOGD(TAG, "  Output processing: %s", config.output_processing.c_str());
    ESP_LOGD(TAG, "Model data validation:");
    ESP_LOGD(TAG, "  Model data pointer: %p", model_data);
    ESP_LOGD(TAG, "  Model data size: %zu bytes", model_size);
  }

  if (model_data == nullptr) {
    ESP_LOGE(TAG, "Model data pointer is NULL!");
    return false;
  }

  if (model_size == 0) {
    ESP_LOGE(TAG, "Model data size is 0!");
    return false;
  }

#ifdef DEBUG_TFLITE_MICRO_HELPER
  if (model_size >= 8) {
    ESP_LOGI(TAG, "First 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X", model_data[0], model_data[1], model_data[2],
             model_data[3], model_data[4], model_data[5], model_data[6], model_data[7]);
    if (model_data[1] == 0x00 && model_data[2] == 0x00 && model_data[3] == 0x00 && model_data[4] == 0x54 &&
        model_data[5] == 0x46 && model_data[6] == 0x4C && model_data[7] == 0x33) {
      ESP_LOGI(TAG, "Valid TFLite magic number found (version byte: 0x%02X)", model_data[0]);
    } else {
      ESP_LOGE(TAG, "Invalid TFLite magic number");
      return false;
    }
  }
#endif

  this->config_ = config;

  ESP_LOGI(TAG, "Loading model from PROGMEM (%zu bytes)", model_size);

  this->tflite_model_ = tflite::GetModel(model_data);

  if (this->tflite_model_ == nullptr) {
    ESP_LOGE(TAG, "Failed to parse model - invalid data");
    return false;
  }

  if (this->tflite_model_->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Model schema version mismatch: Model has %d, Expecting %d", this->tflite_model_->version(),
             TFLITE_SCHEMA_VERSION);
    if (model_data != nullptr && model_size >= 16) {
      ESP_LOGE(TAG, "Model Header Dump (First 16 bytes):");
      ESP_LOGE(TAG, "  %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X", model_data[0],
               model_data[1], model_data[2], model_data[3], model_data[4], model_data[5], model_data[6], model_data[7],
               model_data[8], model_data[9], model_data[10], model_data[11], model_data[12], model_data[13],
               model_data[14], model_data[15]);
      if (model_data[4] == 'T' && model_data[5] == 'F' && model_data[6] == 'L' && model_data[7] == '3') {
        ESP_LOGW(TAG, "Magic number 'TFL3' is PRESENT. Issue might be alignment or genuine schema mismatch.");
      } else {
        ESP_LOGE(TAG, "Magic number 'TFL3' is MISSING! The file is likely corrupted or not a valid TFLite model.");
      }
    }
    return false;
  }

  this->resolver_ = std::make_unique<tflite::MicroMutableOpResolver<MAX_OPERATORS>>();

  if (this->debug_) {
    for (size_t i = 0; i < this->tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = this->tflite_model_->operator_codes()->Get(i);
      ESP_LOGD(TAG, "  [%d]: %d (%s)", static_cast<int>(i), op_code->builtin_code(),
               tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
  }

  std::set<tflite::BuiltinOperator> required_ops;
  for (size_t i = 0; i < this->tflite_model_->operator_codes()->size(); ++i) {
    const auto *op_code = this->tflite_model_->operator_codes()->Get(i);
    required_ops.insert(op_code->builtin_code());
  }

  if (!OpResolverManager::RegisterOps<MAX_OPERATORS>(*this->resolver_, required_ops, TAG)) {
    ESP_LOGE(TAG, "Failed to register operators");
    return false;
  }

  // Conditional interpreter construction: with MRV for streaming, without for standard
  if (mrv != nullptr) {
    this->interpreter_ = std::make_unique<tflite::MicroInterpreter>(this->tflite_model_, *this->resolver_, tensor_arena,
                                                                    tensor_arena_size, mrv);
  } else {
    this->interpreter_ = std::make_unique<tflite::MicroInterpreter>(this->tflite_model_, *this->resolver_, tensor_arena,
                                                                    tensor_arena_size);
  }

  if (this->interpreter_->AllocateTensors() != kTfLiteOk) {
    ESP_LOGE(TAG, "Failed to allocate tensors");
    ESP_LOGE(TAG, "  Tensor arena: %zu bytes (requested: %zu)", tensor_arena_size, this->tensor_arena_size_requested_);
    ESP_LOGE(TAG, "  Model size: %zu bytes", this->model_length_);
    for (size_t i = 0; i < this->tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = this->tflite_model_->operator_codes()->Get(i);
      ESP_LOGE(TAG, "  Op code [%zu]: builtin_code=%d (%s)", i, op_code->builtin_code(),
               tflite::EnumNameBuiltinOperator(op_code->builtin_code()));
    }
    for (size_t i = 0; i < this->tflite_model_->operator_codes()->size(); ++i) {
      const auto *op_code = this->tflite_model_->operator_codes()->Get(i);
      if (op_code->builtin_code() == tflite::BuiltinOperator_DELEGATE) {
        ESP_LOGE(TAG, "  *** WARNING: Model contains DELEGATE operator (index %zu)! ***", i);
        ESP_LOGE(TAG, "  *** TFLite Micro does NOT support delegates. ***");
      }
    }
#ifdef DEBUG_TFLITE_MICRO_HELPER
    size_t arena_used = this->interpreter_->arena_used_bytes();
    ESP_LOGE(TAG, "  Arena used before failure: %zu bytes", arena_used);
#endif
    return false;
  }

  if (this->tflite_model_->subgraphs()->Get(0)->operators()->size() == 0) {
    ESP_LOGE(TAG, "Model has no operators!");
    return false;
  }

  auto *input = this->input_tensor();
  if (input) {
    ESP_LOGI(TAG, "Input tensor dimensions:");
    for (int i = 0; i < input->dims->size; i++) {
      ESP_LOGI(TAG, "  Dim %d: %d", i, input->dims->data[i]);
    }
  }

  TfLiteTensor *output = this->output_tensor();
  if (output) {
    int size = 1;
    for (int i = 0; i < output->dims->size; i++) {
      size *= output->dims->data[i];
    }
    this->output_size_ = size;
  }

  ESP_LOGI(TAG, "Model loaded successfully");
  return true;
}

void ModelHandler::log_input_stats() const {
  const TfLiteTensor *input = this->input_tensor();
  if (input == nullptr)
    return;

  const int total_elements = this->get_input_width() * this->get_input_height() * this->get_input_channels();
  const int sample_size = std::min(20, total_elements);

  ESP_LOGD(TAG, "First %d %s inputs (%s):", sample_size, input->type == kTfLiteFloat32 ? "float32" : "uint8",
           this->image_config_.normalize ? "normalized" : "raw");

  if (input->type == kTfLiteFloat32) {
    const float *data = input->data.f;
    for (int i = 0; i < sample_size; i++) {
      ESP_LOGD(TAG, "  [%d]: %.4f", i, data[i]);
      if (this->image_config_.input_channels >= 3 && i % this->image_config_.input_channels == 2) {
        ESP_LOGD(TAG, "    -> %s: [%.3f, %.3f, %.3f]", this->image_config_.input_order.c_str(), data[i - 2],
                 data[i - 1], data[i]);
      }
    }
  } else {
    const uint8_t *data = input->data.uint8;
    for (int i = 0; i < sample_size; i++) {
      ESP_LOGD(TAG, "  [%d]: %u", i, data[i]);
      if (this->image_config_.input_channels >= 3 && i % this->image_config_.input_channels == 2) {
        ESP_LOGD(TAG, "    -> %s: [%u, %u, %u]", this->image_config_.input_order.c_str(), data[i - 2], data[i - 1],
                 data[i]);
      }
    }
  }
}

void ModelHandler::debug_input_pattern() const {
  const TfLiteTensor *input = this->input_tensor();
  if (!input || input->type != kTfLiteFloat32)
    return;

  const float *data = input->data.f;
  const int total_elements = input->bytes / sizeof(float);
  const int channels = this->get_input_channels();
  const int height = this->get_input_height();
  const int width = this->get_input_width();

  ESP_LOGD(TAG, "Input pattern analysis: %dx%dx%d", width, height, channels);

  float channel_sums[3] = {0};
  float channel_mins[3] = {std::numeric_limits<float>::max()};
  float channel_maxs[3] = {std::numeric_limits<float>::lowest()};
  int pixel_count = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int pos = (y * width + x) * channels;
      if (pos + 2 < total_elements) {
        for (int c = 0; c < channels; c++) {
          float val = data[pos + c];
          channel_sums[c] += val;
          channel_mins[c] = std::min(channel_mins[c], val);
          channel_maxs[c] = std::max(channel_maxs[c], val);
        }
        pixel_count++;
      }
    }
  }

  if (pixel_count > 0) {
    ESP_LOGD(TAG, "Channel statistics:");
    for (int c = 0; c < channels; c++) {
      float mean = channel_sums[c] / pixel_count;
      ESP_LOGD(TAG, "  Channel %d: min=%.3f, max=%.3f, mean=%.3f", c, channel_mins[c], channel_maxs[c], mean);
    }
  }

  bool looks_normalized_0_1 = true;
  bool looks_normalized_neg1_1 = true;
  for (int i = 0; i < total_elements; i++) {
    if (data[i] < 0.0f || data[i] > 1.0f)
      looks_normalized_0_1 = false;
    if (data[i] < -1.0f || data[i] > 1.0f)
      looks_normalized_neg1_1 = false;
  }
  ESP_LOGD(TAG, "Data range analysis:");
  ESP_LOGD(TAG, "  Looks like 0-1 normalized: %s", looks_normalized_0_1 ? "YES" : "NO");
  ESP_LOGD(TAG, "  Looks like -1 to 1 normalized: %s", looks_normalized_neg1_1 ? "YES" : "NO");
}

ProcessedOutput ModelHandler::process_output(TfLiteTensor *output_tensor) const {
  if (!output_tensor) {
    ESP_LOGE(TAG, "Null output tensor");
    return {0.0f, 0.0f};
  }
  if (output_tensor->type == kTfLiteFloat32) {
    return this->process_output(output_tensor->data.f);
  } else if (output_tensor->type == kTfLiteUInt8 || output_tensor->type == kTfLiteInt8) {
    int count = 1;
    for (int i = 0; i < output_tensor->dims->size; i++) {
      count *= output_tensor->dims->data[i];
    }
    std::vector<float> dequantized(count);
    float scale = output_tensor->params.scale;
    int32_t zero_point = output_tensor->params.zero_point;
    if (scale == 0.0f) {
      scale = 1.0f;
      ESP_LOGW(TAG, "Output tensor has scale 0.0, assuming 1.0");
    }
    if (output_tensor->type == kTfLiteUInt8) {
      for (int i = 0; i < count; i++) {
        dequantized[i] = (output_tensor->data.uint8[i] - zero_point) * scale;
      }
    } else {
      for (int i = 0; i < count; i++) {
        dequantized[i] = (output_tensor->data.int8[i] - zero_point) * scale;
      }
    }
    return this->process_output(dequantized.data());
  } else {
    ESP_LOGE(TAG, "Unsupported output tensor type: %d", output_tensor->type);
    return {0.0f, 0.0f};
  }
}

ProcessedOutput ModelHandler::process_output(const float *output_data) const {
  const int num_classes = this->output_size_;
  ProcessedOutput result = {0.0f, 0.0f};
  if (num_classes <= 0) {
    ESP_LOGE(TAG, "Invalid number of output classes: %d", num_classes);
    return result;
  }
  float min_val = output_data[0];
  float max_val = output_data[0];
  int max_idx = 0;
  for (int i = 1; i < num_classes; i++) {
    float val = output_data[i];
    if (val < min_val)
      min_val = val;
    if (val > max_val) {
      max_val = val;
      max_idx = i;
    }
  }
  if (this->config_.output_processing == "direct_class" || this->config_.output_processing == "argmax") {
    result.value = static_cast<float>(max_idx);
    result.confidence = max_val;
  } else if (this->config_.output_processing == "softmax") {
    float max_logit = *std::max_element(output_data, output_data + num_classes);
    std::vector<float> exp_vals(num_classes);
    float sum = 0.0f;
    for (int i = 0; i < num_classes; i++) {
      exp_vals[i] = expf(output_data[i] - max_logit);
      sum += exp_vals[i];
    }
    int sm_max_idx = 0;
    float sm_max_val = 0.0f;
    for (int i = 0; i < num_classes; i++) {
      float prob = exp_vals[i] / sum;
      if (prob > sm_max_val) {
        sm_max_val = prob;
        sm_max_idx = i;
      }
    }
    result.value = static_cast<float>(sm_max_idx) / this->config_.scale_factor;
    result.confidence = sm_max_val;
  } else if (this->config_.output_processing == "auto_detect") {
    // Simplified auto-detect: check if outputs look like probabilities (0-1 range)
    bool looks_like_probs = (min_val >= 0.0f && max_val <= 1.0f);
    if (looks_like_probs) {
      result.value = static_cast<float>(max_idx);
      result.confidence = max_val;
    } else {
      float max_logit = *std::max_element(output_data, output_data + num_classes);
      std::vector<float> exp_vals(num_classes);
      float sum = 0.0f;
      for (int i = 0; i < num_classes; i++) {
        exp_vals[i] = expf(output_data[i] - max_logit);
        sum += exp_vals[i];
      }
      int ad_max_idx = 0;
      float ad_max_val = 0.0f;
      for (int i = 0; i < num_classes; i++) {
        float prob = exp_vals[i] / sum;
        if (prob > ad_max_val) {
          ad_max_val = prob;
          ad_max_idx = i;
        }
      }
      result.value = static_cast<float>(ad_max_idx) / this->config_.scale_factor;
      result.confidence = ad_max_val;
    }
  } else {
    result.value = static_cast<float>(max_idx) / this->config_.scale_factor;
    result.confidence = max_val;
  }
  return result;
}

uint32_t ModelHandler::calculate_crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320;
      else
        crc >>= 1;
    }
  }
  return ~crc;
}

bool ModelHandler::verify_model_crc(const uint8_t *model_data, size_t length) {
  uint32_t crc = calculate_crc32(model_data, length);
  ESP_LOGI(TAG, "Model CRC32: 0x%08X", crc);
#ifdef MODEL_CRC32
  if (crc != MODEL_CRC32) {
    ESP_LOGE(TAG, "Model CRC32 mismatch! Expected: 0x%08X, Got: 0x%08X", MODEL_CRC32, crc);
    return false;
  }
  return true;
#else
  ESP_LOGW(TAG, "MODEL_CRC32 not defined -- skipping verification");
  return true;
#endif
}

void ModelHandler::debug_model_architecture() const {
  if (!this->tflite_model_)
    return;
  auto *subgraphs = this->tflite_model_->subgraphs();
  if (!subgraphs)
    return;
  ESP_LOGD(TAG, "Model Architecture:");
  ESP_LOGD(TAG, "  Subgraphs: %d", subgraphs->size());
  for (int i = 0; i < subgraphs->size(); i++) {
    auto *subgraph = subgraphs->Get(i);
    ESP_LOGD(TAG, "  Subgraph %d:", i);
    ESP_LOGD(TAG, "    Tensors: %d", subgraph->tensors()->size());
    ESP_LOGD(TAG, "    Operators: %d", subgraph->operators()->size());
    ESP_LOGD(TAG, "    Inputs: %d", subgraph->inputs()->size());
    ESP_LOGD(TAG, "    Outputs: %d", subgraph->outputs()->size());
  }
}

bool ModelHandler::validate_model_config() const {
  const TfLiteTensor *input = this->input_tensor();
  if (!input)
    return false;
  if (input->dims->size >= 4) {
    int height = input->dims->data[1];
    int width = input->dims->data[2];
    int channels = input->dims->data[3];
    if (this->image_config_.input_size.size() >= 2) {
      if (width != this->image_config_.input_size[0] || height != this->image_config_.input_size[1]) {
        ESP_LOGW(TAG, "Model input size mismatch! Config: %dx%d, Model: %dx%d", this->image_config_.input_size[0],
                 this->image_config_.input_size[1], width, height);
        return false;
      }
    }
    if (channels != this->image_config_.input_channels) {
      ESP_LOGW(TAG, "Model input channels mismatch! Config: %d, Model: %d", this->image_config_.input_channels,
               channels);
      return false;
    }
  }
  return true;
}

void ModelHandler::report_memory_status() {
  size_t effective_arena_size = this->tensor_arena_allocation_.actual_size;
  if (effective_arena_size == 0) {
    effective_arena_size = this->tensor_arena_size_requested_;
  }
  this->memory_manager_.report_memory_status(this->tensor_arena_size_requested_, effective_arena_size,
                                             this->get_arena_used_bytes(), this->model_length_);
}

size_t ModelHandler::probe_arena_size_(const uint8_t *model_start, size_t initial_size,
                                       const tflite::MicroMutableOpResolver<MAX_OPERATORS> &resolver) {
  const tflite::Model *model = tflite::GetModel(model_start);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "probe_arena_size_: Model schema version mismatch");
    return 0;
  }

  size_t attempt_sizes[] = {(initial_size + 15) & ~15, (initial_size * 3 / 2 + 15) & ~15,
                            (initial_size * 2 + 15) & ~15};

  for (size_t attempt_size : attempt_sizes) {
    uint8_t *probe_arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, attempt_size, MALLOC_CAP_8BIT));
    if (probe_arena == nullptr)
      continue;

    auto probe_interpreter = std::make_unique<tflite::MicroInterpreter>(model, resolver, probe_arena, attempt_size);
    if (probe_interpreter->AllocateTensors() != kTfLiteOk) {
      probe_interpreter.reset();
      heap_caps_free(probe_arena);
      continue;
    }

    size_t lower = (probe_interpreter->arena_used_bytes() + 16 + 15) & ~15;
    probe_interpreter.reset();
    size_t upper = attempt_size;

    while (lower < upper) {
      auto test_interpreter = std::make_unique<tflite::MicroInterpreter>(model, resolver, probe_arena, lower);
      bool ok = test_interpreter->AllocateTensors() == kTfLiteOk;
      test_interpreter.reset();
      if (ok) {
        upper = lower + 16;
        break;
      }
      lower = ((lower + upper) / 2 + 15) & ~15;
    }
    heap_caps_free(probe_arena);
    return upper;
  }
  return 0;
}

}  // namespace tflite_micro_helper
}  // namespace esphome