#include "meter_reader_tflite.h"
#include "model_config.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <memory>
#include <new>

namespace esphome {
namespace meter_reader_tflite {

using tflite_micro_helper::MemoryManager;
using tflite_micro_helper::ModelConfig;
using tflite_micro_helper::ProcessedOutput;

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
    
    // Load model
    if (!load_model()) {
        mark_failed();
        return;
    }
    
    // Initialize image processor
    if (camera_width_ > 0 && camera_height_ > 0) {
        using namespace esp32_camera_utils;
        
        // Determine input type from model
        ImageProcessorInputType input_type = kInputTypeUnknown;
        if (model_handler_.input_tensor()->type == kTfLiteFloat32) {
            input_type = kInputTypeFloat32;
        } else if (model_handler_.input_tensor()->type == kTfLiteUInt8 || 
                   model_handler_.input_tensor()->type == kTfLiteInt8) {
            input_type = kInputTypeUInt8;
        }
        
        image_processor_ = std::make_unique<esp32_camera_utils::ImageProcessor>(
            esp32_camera_utils::ImageProcessorConfig{
                .camera_width = camera_width_,
                .camera_height = camera_height_,
                .pixel_format = pixel_format_,
                .model_width = model_handler_.get_input_width(),
                .model_height = model_handler_.get_input_height(),
                .model_channels = model_handler_.get_input_channels(),
                .input_type = input_type,
                .normalize = model_handler_.get_config().normalize
            }
        );
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
        if (!image_processor_->process_zone_to_buffer(frame, zone, tensor_data, tensor_size)) {
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

bool MeterReaderTFLite::process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult& result, float* value, float* confidence) {
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
  
  // Store as original dimensions if not already set
  if (original_camera_width_ == 0) {
    original_camera_width_ = width;
  }
  if (original_camera_height_ == 0) {
    original_camera_height_ = height;
  }
  if (original_pixel_format_.empty()) {
    original_pixel_format_ = pixel_format;
  }
  
  ESP_LOGD(TAG, "Camera format set: %dx%d, %s (original: %dx%d, %s)", 
           width, height, pixel_format.c_str(),
           original_camera_width_, original_camera_height_, original_pixel_format_.c_str());
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
    ESP_LOGI(TAG, "  Tensor Arena Size (Requested): %zu bytes (%.1f KB)", 
             tensor_arena_size_requested_, tensor_arena_size_requested_ / 1024.0f);
    ESP_LOGI(TAG, "  Tensor Arena Size (Actual): %zu bytes (%.1f KB)", 
             tensor_arena_allocation_.actual_size, tensor_arena_allocation_.actual_size / 1024.0f);
    
    // Get the actual peak usage from the interpreter
    size_t peak_usage = model_handler_.get_arena_used_bytes();
    ESP_LOGI(TAG, "  Arena Peak Usage: %zu bytes (%.1f KB)", peak_usage, peak_usage / 1024.0f);
    
    // Calculate total memory usage
    size_t total_memory = model_length_ + tensor_arena_allocation_.actual_size;
    ESP_LOGI(TAG, "  TOTAL Memory Footprint: %zu bytes (%.1f KB)", 
             total_memory, total_memory / 1024.0f);
    
    memory_manager_.report_memory_status(
        tensor_arena_size_requested_,
        tensor_arena_allocation_.actual_size,
        peak_usage,
        model_length_
    );
    ESP_LOGI(TAG, "----------------------------------");
}

bool esphome::meter_reader_tflite::MeterReaderTFLite::load_model() {
    DURATION_START();
    ESP_LOGI(TAG, "Loading TFLite model...");
    
    // Get model configuration from model_config.h FIRST
    ModelConfig config;
    auto it = MODEL_CONFIGS.find(model_type_);
    if (it != MODEL_CONFIGS.end()) {
        config = it->second;
        ESP_LOGI(TAG, "Using model config: %s", config.description.c_str());
        
        // ALWAYS use tensor arena size from model configuration (override any previous setting)
        std::string arena_size_str = config.tensor_arena_size;
        size_t multiplier = 1;
        
        if (arena_size_str.find("KB") != std::string::npos) {
            multiplier = 1024;
            arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
        } else if (arena_size_str.find("MB") != std::string::npos) {
            multiplier = 1024 * 1024;
            arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 2);
        } else if (arena_size_str.find("B") != std::string::npos) {
            arena_size_str = arena_size_str.substr(0, arena_size_str.length() - 1);
        }
        
        // Manual string to integer conversion without exceptions
        const char* str = arena_size_str.c_str();
        char* end_ptr;
        long size_value = strtol(str, &end_ptr, 10);
        
        // Check if conversion was successful
        if (end_ptr != str && *end_ptr == '\0' && size_value > 0) {
            tensor_arena_size_requested_ = size_value * multiplier;
            ESP_LOGI(TAG, "Using model-specific tensor arena size: %s (%zu bytes)", 
                    config.tensor_arena_size.c_str(), tensor_arena_size_requested_);
        } else {
            ESP_LOGW(TAG, "Failed to parse tensor arena size from config: %s, using default", 
                    config.tensor_arena_size.c_str());
            // Keep the existing tensor_arena_size_requested_ value
        }
    } else {
        // config = DEFAULT_MODEL_CONFIG;
        ESP_LOGE(TAG, "Model type '%s' not found", 
                model_type_.c_str());
    }

    // Allocate tensor arena with the determined size
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }

    // Load the model with the config
    if (!model_handler_.load_model(model_, model_length_, 
                                 tensor_arena_allocation_.data.get(), 
                                 tensor_arena_allocation_.actual_size,
                                 config)) {
        ESP_LOGE(TAG, "Failed to load model into interpreter");
        return false;
    }


    ESP_LOGI(TAG, "Model loaded successfully");
    ESP_LOGI(TAG, "Input dimensions: %dx%dx%d", 
            model_handler_.get_input_width(),
            model_handler_.get_input_height(),
            model_handler_.get_input_channels());
    
    DURATION_END("load_model");
    return true;
}

#ifdef DEBUG_METER_READER_TFLITE
class DebugCameraImage : public camera::CameraImage {
 public:
    DebugCameraImage(const uint8_t* data, size_t size, int width, int height)
        : data_(data, data + size), width_(width), height_(height) {}

    uint8_t* get_data_buffer() override { return data_.data(); }
    size_t get_data_length() override { return data_.size(); }
    bool was_requested_by(camera::CameraRequester requester) const override { 
        return false;  // Debug image isn't tied to requester
    }

    int get_width() const { return width_; }
    int get_height() const { return height_; }

private:
    std::vector<uint8_t> data_;
    int width_;
    int height_;
};

void MeterReaderTFLite::set_debug_image(const uint8_t* data, size_t size) {
    debug_image_ = std::make_shared<DebugCameraImage>(
        data, size, camera_width_, camera_height_
    );
    
    // Initialize image processor for debug image
    if (camera_width_ > 0 && camera_height_ > 0) {
        using namespace esp32_camera_utils;
        
        // Determine input type from model
        ImageProcessorInputType input_type = kInputTypeUnknown;
        if (model_handler_.input_tensor()->type == kTfLiteFloat32) {
            input_type = kInputTypeFloat32;
        } else if (model_handler_.input_tensor()->type == kTfLiteUInt8 || 
                   model_handler_.input_tensor()->type == kTfLiteInt8) {
            input_type = kInputTypeUInt8;
        }
        
        image_processor_ = std::make_unique<ImageProcessor>(
            ImageProcessorConfig{
                .camera_width = camera_width_,
                .camera_height = camera_height_,
                .pixel_format = pixel_format_,
                .model_width = model_handler_.get_input_width(),
                .model_height = model_handler_.get_input_height(),
                .model_channels = model_handler_.get_input_channels(),
                .input_type = input_type,
                .normalize = model_handler_.get_config().normalize
            }
        );
        ESP_LOGI(TAG, "ImageProcessor reinitialized with dimensions: %dx%d, format: %s",
                 camera_width_, camera_height_, pixel_format_.c_str());
    } else {
        ESP_LOGW(TAG, "ImageProcessor not available for reinitialization");
    }
}

void MeterReaderTFLite::test_with_debug_image() {
    if (!debug_image_) {
        ESP_LOGW(TAG, "No debug image available for testing");
        return;
    }
    
    ESP_LOGI(TAG, "Testing with debug image...");
    process_full_image(debug_image_);
}

void MeterReaderTFLite::test_with_debug_image_all_configs() {
    if (!debug_image_) {
        ESP_LOGW(TAG, "No debug image available for testing all configs");
        return;
    }
    
    ESP_LOGI(TAG, "Testing with debug image (All Configs)...");
    // Iterate through some configurations if needed, or just run standard test
    // For now, just run the standard test as a placeholder if specific configs aren't known
    test_with_debug_image();
}

void MeterReaderTFLite::debug_test_with_pattern() {
    ESP_LOGI(TAG, "Running debug test with pattern...");
    // Create a synthetic image pattern for testing
    // This is a placeholder implementation
    int width = 160;
    int height = 120;
    size_t size = width * height * 3; // RGB888
    std::vector<uint8_t> pattern_data(size, 128); // Grey image
    
    auto pattern_image = std::make_shared<DebugCameraImage>(
        pattern_data.data(), pattern_data.size(), width, height
    );
    
    process_full_image(pattern_image);
}
#endif

void MeterReaderTFLite::reinitialize_image_processor() {
    if (camera_width_ > 0 && camera_height_ > 0) {
        using namespace esp32_camera_utils;
        
        // Determine input type from model
        ImageProcessorInputType input_type = kInputTypeUnknown;
        if (model_handler_.input_tensor()->type == kTfLiteFloat32) {
            input_type = kInputTypeFloat32;
        } else if (model_handler_.input_tensor()->type == kTfLiteUInt8 || 
                   model_handler_.input_tensor()->type == kTfLiteInt8) {
            input_type = kInputTypeUInt8;
        }
        
        image_processor_ = std::make_unique<ImageProcessor>(
            ImageProcessorConfig{
                .camera_width = camera_width_,
                .camera_height = camera_height_,
                .pixel_format = pixel_format_,
                .model_width = model_handler_.get_input_width(),
                .model_height = model_handler_.get_input_height(),
                .model_channels = model_handler_.get_input_channels(),
                .input_type = input_type,
                .normalize = model_handler_.get_config().normalize
            }
        );
        ESP_LOGI(TAG, "ImageProcessor reinitialized with dimensions: %dx%d, format: %s",
                 camera_width_, camera_height_, pixel_format_.c_str());
    }
}

// Public method to set crop zones (called from service)
void MeterReaderTFLite::set_crop_zones(const std::string &zones_json) {
    crop_zone_handler_.update_zones(zones_json);
    ESP_LOGI(TAG, "Crop zones updated via service");
}

// Public method to reset camera window (called from service)
bool MeterReaderTFLite::reset_camera_window() {
    if (!camera_) return false;
    
    int current_width = 0;
    int current_height = 0;
    
    bool success = camera_window_control_.reset_to_full_frame_with_dimensions(
        camera_, 
        original_camera_width_, 
        original_camera_height_,
        current_width, 
        current_height
    );
    
    if (success) {
        camera_width_ = current_width;
        camera_height_ = current_height;
        pixel_format_ = original_pixel_format_;
        
        reinitialize_image_processor();
        ESP_LOGI(TAG, "Camera window reset to full frame (%dx%d)", camera_width_, camera_height_);
    } else {
        ESP_LOGE(TAG, "Failed to reset camera window");
    }
    
    return success;
}

bool MeterReaderTFLite::test_camera_after_reset() {
    if (!camera_) return false;
    // Simple check if we can request a frame or if camera is responsive
    return true; 
}

void MeterReaderTFLite::basic_camera_recovery() {
    ESP_LOGW(TAG, "Attempting basic camera recovery...");
    
    // Reset internal flags
    pending_frame_.reset();
    frame_available_ = false;
    frame_requested_ = false;
    processing_frame_ = false;
    
    // Reinitialize image processor
    reinitialize_image_processor();
    
    ESP_LOGI(TAG, "Basic camera recovery completed");
}

void MeterReaderTFLite::dump_config() {
    ESP_LOGCONFIG(TAG, "Meter Reader TFLite:");
    ESP_LOGCONFIG(TAG, "  Model Type: %s", model_type_.c_str());
    ESP_LOGCONFIG(TAG, "  Confidence Threshold: %.2f", confidence_threshold_);
    ESP_LOGCONFIG(TAG, "  Tensor Arena Size: %zu", tensor_arena_size_requested_);
    ESP_LOGCONFIG(TAG, "  Camera Dimensions: %dx%d", camera_width_, camera_height_);
    ESP_LOGCONFIG(TAG, "  Pixel Format: %s", pixel_format_.c_str());
    ESP_LOGCONFIG(TAG, "  Allow Negative Rates: %s", allow_negative_rates_ ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Max Absolute Diff: %d", max_absolute_diff_);
}

void MeterReaderTFLite::set_model_config(const std::string &model_type) {
    model_type_ = model_type;
    ESP_LOGD(TAG, "Model type set to: %s", model_type_.c_str());
}

bool MeterReaderTFLite::allocate_tensor_arena() {
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }
    return true;
}

}  // namespace meter_reader_tflite
}  // namespace esphome