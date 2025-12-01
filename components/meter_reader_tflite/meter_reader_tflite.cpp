#include "meter_reader_tflite.h"
#include "model_config.h"
#include <sstream>
#include <iomanip>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace meter_reader_tflite {

using tflite_micro_helper::ModelConfig;
using tflite_micro_helper::ProcessedOutput;
using esp32_camera_utils::ImageProcessorConfig;
using esp32_camera_utils::ImageProcessorInputType;
using esp32_camera_utils::kInputTypeFloat32;
using esp32_camera_utils::kInputTypeUInt8;
using esp32_camera_utils::kInputTypeUnknown;

static const char *const TAG = "meter_reader_tflite";

#define DURATION_START() uint32_t start_time = millis()
#define DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)

// Uncomment to enable debug mode
#define DEBUG_METER_READER_TFLITE

void MeterReaderTFLite::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Meter Reader TFLite...");
    
    // Initialize crop zone handler with camera dimensions if available
    if (camera_width_ > 0 && camera_height_ > 0) {
        crop_zone_handler_.set_default_zone(camera_width_, camera_height_);
    }
    
    // Pass camera to utils
    if (camera_) {
        camera_utils_.set_camera(camera_);
    }
    
    // Load model
    if (!load_model()) {
        mark_failed();
        return;
    }
    
    // Initialize image processor via camera utils
    if (camera_width_ > 0 && camera_height_ > 0) {
        // Determine input type from model
        ImageProcessorInputType input_type = kInputTypeUnknown;
        TfLiteTensor* input = model_handler_.input_tensor();
        if (input->type == kTfLiteFloat32) {
            input_type = kInputTypeFloat32;
        } else if (input->type == kTfLiteUInt8 || 
                   input->type == kTfLiteInt8) {
            input_type = kInputTypeUInt8;
        }
        
        ImageProcessorConfig config;
        config.camera_width = camera_width_;
        config.camera_height = camera_height_;
        config.pixel_format = pixel_format_;
        config.model_width = model_handler_.get_input_width();
        config.model_height = model_handler_.get_input_height();
        config.model_channels = model_handler_.get_input_channels();
        config.input_type = input_type;
        config.normalize = model_handler_.get_config().normalize;
        
        camera_utils_.reinitialize_image_processor(config);
    }
    
    // Register image callback
    if (camera_) {
        camera_->add_image_callback([this](std::shared_ptr<camera::CameraImage> image) {
            if (this->frame_requested_) {
                this->pending_frame_ = image;
                this->frame_available_ = true;
                this->frame_requested_ = false;
                this->last_frame_received_ = millis();
                ESP_LOGV(TAG, "Frame received via callback");
            }
        });
    }

    // Initialize validation
    setup_output_validation();

    // Process debug image AFTER ImageProcessor is initialized
    #ifdef DEBUG_METER_READER_TFLITE
    if (debug_image_) {
        ESP_LOGI(TAG, "Processing debug image after setup completion");
        this->set_timeout(1000, [this]() { // Small delay to ensure everything is ready
            this->test_with_debug_image();
        });
    } else {
        ESP_LOGE(TAG, "No debug image set to process.");
    }
    #endif
}

void MeterReaderTFLite::setup_output_validation() {
    ValueValidator::ValidationConfig config;
    config.allow_negative_rates = allow_negative_rates_;
    config.max_absolute_diff = max_absolute_diff_;
    // config.max_rate_change = 0.15f; // Default
    
    value_validator_.set_config(config);
    value_validator_.setup();
    
    ESP_LOGD(TAG, "Output validation initialized (Max Diff: %d, Allow Negative: %s)", 
             max_absolute_diff_, allow_negative_rates_ ? "YES" : "NO");
}

void MeterReaderTFLite::update() {
    if (pause_processing_) return;

    // Check if we are already processing or have a request pending
    if (processing_frame_) {
        ESP_LOGW(TAG, "Still processing previous frame, skipping update");
        return;
    }

    if (frame_requested_) {
        ESP_LOGW(TAG, "Frame request still pending, skipping update");
        return;
    }

    // Request a new frame
    if (camera_) {
        frame_requested_ = true;
        last_request_time_ = millis();
        ESP_LOGV(TAG, "Requesting new frame");
        
        if (flash_controller_) {
            flash_controller_->initiate_capture_sequence([this]() {
                // Trigger camera if possible, otherwise wait for callback
                // this->camera_->update(); // Try this if compilation allows
            });
        } else {
             // Trigger camera if possible
             // this->camera_->update(); 
        }
    }
}

void MeterReaderTFLite::loop() {
    // 1. Handle Frame Request Timeout
    if (frame_requested_ && !frame_available_) {
        if (millis() - last_request_time_ > 5000) {
            ESP_LOGW(TAG, "Frame request timed out");
            frame_requested_ = false;
        }
    }

    // 2. Process Available Frame
    if (frame_available_) {
        process_available_frame();
    }
}

void MeterReaderTFLite::process_available_frame() {
    if (!pending_frame_) {
        frame_available_ = false;
        return;
    }

    // Move frame to local scope to free atomic/shared quickly if needed, 
    // though we process it here.
    std::shared_ptr<camera::CameraImage> frame = pending_frame_;
    pending_frame_ = nullptr;
    frame_available_ = false;

    process_full_image(frame);
}

void MeterReaderTFLite::process_full_image(std::shared_ptr<camera::CameraImage> frame) {
    if (!frame) return;
    
    processing_frame_ = true;
    DURATION_START();

    // 1. Get Crop Zones
    const auto& zones = crop_zone_handler_.get_zones();
    if (zones.empty()) {
        ESP_LOGW(TAG, "No crop zones defined. Skipping processing.");
        processing_frame_ = false;
        return;
    }

    // 2. Prepare Model Input Buffer
    TfLiteTensor* input_tensor = model_handler_.input_tensor();
    if (!input_tensor) {
        ESP_LOGE(TAG, "Model input tensor is null");
        processing_frame_ = false;
        return;
    }

    uint8_t* tensor_data = nullptr;
    if (input_tensor->type == kTfLiteFloat32) {
        tensor_data = reinterpret_cast<uint8_t*>(input_tensor->data.f);
    } else if (input_tensor->type == kTfLiteUInt8 || input_tensor->type == kTfLiteInt8) {
        tensor_data = reinterpret_cast<uint8_t*>(input_tensor->data.uint8);
    } else {
        ESP_LOGE(TAG, "Unsupported input tensor type");
        processing_frame_ = false;
        return;
    }
    
    size_t tensor_size = input_tensor->bytes;

    // 3. Process Each Zone
    std::vector<float> readings;
    std::vector<float> confidences;
    
    // Optimization: Reserve memory to avoid reallocations
    readings.reserve(zones.size());
    confidences.reserve(zones.size());
    
    ESP_LOGD(TAG, "Processing %d zones...", zones.size());

    for (size_t i = 0; i < zones.size(); i++) {
        const auto& zone = zones[i];
        
        // A. Process Image Zone directly into Model Input Tensor
        // This avoids allocating an intermediate buffer
        if (!camera_utils_.process_zone(frame, zone, tensor_data, tensor_size)) {
            ESP_LOGE(TAG, "Failed to process zone %d", i);
            readings.push_back(0.0f); // Error placeholder
            confidences.push_back(0.0f);
            continue;
        }

        // B. Run Inference
        if (model_handler_.invoke() != kTfLiteOk) {
            ESP_LOGE(TAG, "Model invocation failed for zone %d", i);
            readings.push_back(0.0f);
            confidences.push_back(0.0f);
            continue;
        }

        // C. Process Output
        // Assuming model output is [value, confidence] or similar, handled by model_handler
        TfLiteTensor* output_tensor = model_handler_.output_tensor();
        if (!output_tensor) {
             ESP_LOGE(TAG, "Output tensor is null");
             continue;
        }
        
        ProcessedOutput output = model_handler_.process_output(output_tensor->data.f);
        
        readings.push_back(output.value);
        confidences.push_back(output.confidence);
        
        ESP_LOGV(TAG, "Zone %d: Value=%.2f, Conf=%.2f", i, output.value, output.confidence);
    }

    // 4. Combine and Validate
    if (!readings.empty()) {
        float combined_value = combine_readings(readings);
        
        // Calculate average confidence
        float avg_confidence = 0.0f;
        for (float c : confidences) avg_confidence += c;
        if (!confidences.empty()) avg_confidence /= confidences.size();
        
        // Validate
        float validated_value = 0.0f;
        if (validate_and_update_reading(combined_value, avg_confidence, validated_value)) {
            // Success
            if (value_sensor_) value_sensor_->publish_state(validated_value);
            if (confidence_sensor_) confidence_sensor_->publish_state(avg_confidence);
            
            std::string log_msg = "Read: " + std::to_string(validated_value);
            if (main_logs_) main_logs_->publish_state(log_msg);
        } else {
            // Validation failed
            std::string log_msg = "Invalid: " + std::to_string(combined_value);
            if (main_logs_) main_logs_->publish_state(log_msg);
        }
        
        // Update inference logs
        if (inference_logs_) {
            std::stringstream ss;
            ss << "V:" << std::fixed << std::setprecision(1) << combined_value 
               << " C:" << std::setprecision(2) << avg_confidence 
               << " (" << millis() - start_time << "ms)";
            inference_logs_->publish_state(ss.str());
        }
    }

    processing_frame_ = false;
    DURATION_END("Full processing");
}

bool MeterReaderTFLite::validate_and_update_reading(float raw_reading, float confidence, float& validated_reading) {
    int raw_int = static_cast<int>(round(raw_reading));
    int valid_int = 0;
    
    bool is_valid = value_validator_.validate_reading(raw_int, confidence, valid_int);
    
    validated_reading = static_cast<float>(valid_int);
    return is_valid;
}

bool MeterReaderTFLite::process_model_result(float value, float confidence) {
    return false; // Unused
}

void MeterReaderTFLite::set_model(const uint8_t *model, size_t length) {
    model_ = model;
    model_length_ = length;
    ESP_LOGD(TAG, "Model set: %zu bytes", length);
}

void MeterReaderTFLite::set_camera_image_format(int width, int height, const std::string &pixel_format) {
  camera_width_ = width;
  camera_height_ = height;
  pixel_format_ = pixel_format;
  
  // Delegate to camera utils
  camera_utils_.set_camera_image_format(width, height, pixel_format);
}

float MeterReaderTFLite::combine_readings(const std::vector<float> &readings) {
    float combined_value = 0.0f;
    
    ESP_LOGI(TAG, "Processing %d readings:", readings.size());
    
    // Convert each reading to integer digit and handle wrap-around
    for (size_t i = 0; i < readings.size(); i++) {
        int digit = static_cast<int>(round(readings[i]));
        
        // Handle wrap-around for original models (like Python script)
        if (digit == 10) {
            digit = 0;
            ESP_LOGD(TAG, "Zone %d: Raw=%.1f -> Rounded=10 -> Wrapped to 0", 
                    i + 1, readings[i]);
        } else {
            ESP_LOGD(TAG, "Zone %d: Raw=%.1f -> Rounded=%d", 
                    i + 1, readings[i], digit);
        }
        
        combined_value = combined_value * 10.0f + digit;
    }
    
    // Log raw readings for debugging
    std::string readings_str;
    for (const auto& reading : readings) {
      if (!readings_str.empty()) {
        readings_str += ", ";
      }
      char buffer[16];
      snprintf(buffer, sizeof(buffer), "%.1f", reading);
      readings_str += buffer;
    }
    ESP_LOGD(TAG, "Raw readings: [%s]", readings_str.c_str());
    
    ESP_LOGI(TAG, "Final combined value: %.0f", combined_value);
    return combined_value;
}

MeterReaderTFLite::~MeterReaderTFLite() {
    // Clean up pending frame
    pending_frame_.reset();
    
    // Add memory validation
    ESP_LOGI(TAG, "Component destruction - memory cleanup");
}

void MeterReaderTFLite::print_debug_info() {
    ESP_LOGI(TAG, "--- MeterReaderTFLite Debug Info ---");
    ESP_LOGI(TAG, "  Model Loaded: %s", model_loaded_ ? "Yes" : "No");
    ESP_LOGI(TAG, "  Camera Dimensions: %dx%d", camera_width_, camera_height_);
    ESP_LOGI(TAG, "  Pixel Format: %s", pixel_format_.c_str());
    ESP_LOGI(TAG, "  Model Size: %zu bytes (%.1f KB)", model_length_, model_length_ / 1024.0f);
    ESP_LOGCONFIG(TAG, "  Confidence Threshold: %.2f", confidence_threshold_);
    ESP_LOGCONFIG(TAG, "  Tensor Arena Size: %zu", model_handler_.get_tensor_arena_size());
    ESP_LOGCONFIG(TAG, "  Camera Dimensions: %dx%d", camera_width_, camera_height_);
    ESP_LOGCONFIG(TAG, "  Pixel Format: %s", pixel_format_.c_str());
    ESP_LOGCONFIG(TAG, "  Allow Negative Rates: %s", allow_negative_rates_ ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Max Absolute Diff: %d", max_absolute_diff_);
}

void MeterReaderTFLite::set_model_config(const std::string &model_type) {
    model_type_ = model_type;
    ESP_LOGD(TAG, "Model type set to: %s", model_type_.c_str());
}

bool MeterReaderTFLite::load_model() {
    DURATION_START();
    ESP_LOGI(TAG, "Loading TFLite model...");
    
    // Get model configuration from model_config.h FIRST
    ModelConfig config;
    auto it = MODEL_CONFIGS.find(model_type_);
    if (it != MODEL_CONFIGS.end()) {
        config = it->second;
        ESP_LOGI(TAG, "Using model config: %s", config.description.c_str());
        
        // Override config with YAML requested size if set
        if (tensor_arena_size_requested_ > 0) {
            ESP_LOGI(TAG, "Overriding config tensor arena size with YAML value: %zu", tensor_arena_size_requested_);
            config.tensor_arena_size = std::to_string(tensor_arena_size_requested_) + "B";
        }
    } else {
        ESP_LOGE(TAG, "Model type '%s' not found", model_type_.c_str());
    }

    // Delegate to helper
    if (!model_handler_.load_model(model_, model_length_, config)) {
        ESP_LOGE(TAG, "Failed to load model via helper");
        return false;
    }

    ESP_LOGI(TAG, "Model loaded successfully");
    ESP_LOGI(TAG, "Input dimensions: %dx%dx%d", 
            model_handler_.get_input_width(),
            model_handler_.get_input_height(),
            model_handler_.get_input_channels());
    
    model_loaded_ = true;
    DURATION_END("load_model");
    return true;
}

void MeterReaderTFLite::basic_camera_recovery() {
    camera_utils_.basic_camera_recovery();
}

void MeterReaderTFLite::dump_config() {
    ESP_LOGCONFIG(TAG, "Meter Reader TFLite:");
    ESP_LOGCONFIG(TAG, "  Model Type: %s", model_type_.c_str());
    ESP_LOGCONFIG(TAG, "  Confidence Threshold: %.2f", confidence_threshold_);
    ESP_LOGCONFIG(TAG, "  Tensor Arena Size: %zu", model_handler_.get_tensor_arena_size());
    ESP_LOGCONFIG(TAG, "  Camera Dimensions: %dx%d", camera_width_, camera_height_);
    ESP_LOGCONFIG(TAG, "  Pixel Format: %s", pixel_format_.c_str());
    ESP_LOGCONFIG(TAG, "  Allow Negative Rates: %s", allow_negative_rates_ ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Max Absolute Diff: %d", max_absolute_diff_);
}

bool MeterReaderTFLite::reset_camera_window() {
    return camera_utils_.reset_window(camera_width_, camera_height_);
}

#ifdef DEBUG_METER_READER_TFLITE
void MeterReaderTFLite::test_with_debug_image() {
    if (!debug_image_) {
        ESP_LOGW(TAG, "No debug image available for testing");
        return;
    }
    ESP_LOGI(TAG, "Testing with debug image...");
    process_full_image(debug_image_);
}

void MeterReaderTFLite::test_with_debug_image_all_configs() {
    ESP_LOGI(TAG, "Testing with debug image (all configs) - Not implemented");
}

void MeterReaderTFLite::debug_test_with_pattern() {
    ESP_LOGI(TAG, "Testing with pattern - Not implemented");
}

void MeterReaderTFLite::set_debug_image(const uint8_t *data, size_t size) {
    if (size == 0) return;
    // Create a copy of the data
    // This is a simplified implementation, assuming JPEG or similar that needs decoding?
    // Or raw data? The original code likely handled this.
    // For now, we just log.
    ESP_LOGW(TAG, "set_debug_image called but not fully implemented");
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome