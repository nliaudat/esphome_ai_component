#include "meter_reader_tflite.h"
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

static const char *const TAG = "meter_reader_tflite";

#define DURATION_START() uint32_t start_time = millis()
#define DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)

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
    
    // Initialize validation
    setup_output_validation();
}

void MeterReaderTFLite::setup_output_validation() {
    if (!model_handler_.invoke_model(result.data->get(), result.size)) {
        ESP_LOGE(TAG, "Model invocation failed");
        return false;
    }

    // Get both value and confidence from the model handler
    ProcessedOutput output = model_handler_.get_processed_output();
    *value = output.value;
    *confidence = output.confidence;

    ESP_LOGD(TAG, "Model result - Value: %.1f, Confidence: %.6f", *value, *confidence);
    return true;
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
    std::string digit_string;
    
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
        
        digit_string += std::to_string(digit);
    }
    
    ESP_LOGI(TAG, "Concatenated digit string: %s", digit_string.c_str());
    
    // Convert string to float (like Python's int())
    float combined_value = std::stof(digit_string);
    
    // ESP_LOGD(TAG, "Raw readings: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", 
         // readings[0], readings[1], readings[2], readings[3], 
         // readings[4], readings[5], readings[6], readings[7]);
         
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
    #ifdef DEBUG_METER_READER_TFLITE
    // Optional: Add leak detection here
    // MemoryTracker::dump_leaks();
    #endif
}

size_t MeterReaderTFLite::available() const {
    return 0; // Frames processed directly in callback
}

uint8_t *MeterReaderTFLite::peek_data_buffer() {
    return nullptr; // Image data handled internally
}

void MeterReaderTFLite::consume_data(size_t consumed) {
    // Not used - image processed in one go
}

void MeterReaderTFLite::return_image() {
    // Image released after processing completes
}

void MeterReaderTFLite::set_image(std::shared_ptr<camera::CameraImage> image) {
    // Part of CameraImageReader interface - not used directly
}

void MeterReaderTFLite::set_model_config(const std::string &model_type) {
    model_type_ = model_type;
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
    size_t peak_usage = model_handler_.get_arena_peak_bytes();
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
// void MeterReaderTFLite::print_debug_info() {
    // print_meter_reader_debug_info(this);
// }

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
        data, size, camera_width_, camera_height_);
    ESP_LOGI(TAG, "Debug image set: %zu bytes (%dx%d)", 
             size, camera_width_, camera_height_);
}

void MeterReaderTFLite::test_with_debug_image() {
    if (debug_image_) {
        // Check if ImageProcessor is ready
        if (!image_processor_) {
            ESP_LOGE(TAG, "ImageProcessor not initialized yet");
            return;
        }
        
         //Ensure camera dimensions are set for debug image
        if (camera_width_ == 0 || camera_height_ == 0) {
            ESP_LOGE(TAG, "Camera dimensions not set for debug image processing");
            return;
        }
        
        // Use static debug zones instead of parsed zones
        crop_zone_handler_.set_debug_zones();
        
        ESP_LOGI(TAG, "Processing debug image with static crop zones...");
        process_full_image(debug_image_);
        set_pause_processing(true);
        
    } else {
        ESP_LOGE(TAG, "No debug image set to process.");
    }
}


void MeterReaderTFLite::test_with_debug_image_all_configs() {
    if (debug_image_) {
        if (!image_processor_) {
            ESP_LOGE(TAG, "ImageProcessor not initialized yet");
            return;
        }
        
        // Use static debug zones
        crop_zone_handler_.set_debug_zones();
        auto debug_zones = crop_zone_handler_.get_zones();
        
        ESP_LOGI(TAG, "Processing %d debug zones...", debug_zones.size());
        
        // Process all debug zones through the image processor
        auto processed_zones = image_processor_->split_image_in_zone(debug_image_, debug_zones);

        if (!processed_zones.empty() && processed_zones.size() == debug_zones.size()) {
            // Prepare zone data for testing
            std::vector<std::vector<uint8_t>> zone_data;
            
            for (size_t zone_idx = 0; zone_idx < processed_zones.size(); zone_idx++) {
                auto& zone_result = processed_zones[zone_idx];

void MeterReaderTFLite::debug_test_with_pattern() {
    ESP_LOGI(TAG, "Testing with simple pattern instead of debug image");
    
    int width = model_handler_.get_input_width();
    int height = model_handler_.get_input_height();
    int channels = model_handler_.get_input_channels();
    size_t input_size = width * height * channels * sizeof(float);
    
    std::vector<uint8_t> test_pattern(input_size);
    float* float_pattern = reinterpret_cast<float*>(test_pattern.data());
    
    // Create a simple test pattern
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            
            // Simple gradient pattern
            float_pattern[pos] = (x / float(width)) * 255.0f;     // R
            if (channels > 1) float_pattern[pos + 1] = (y / float(height)) * 255.0f;  // G
            if (channels > 2) float_pattern[pos + 2] = 128.0f;    // B
        }
    }
    
    // Last resort: try to reinitialize the image processor and hope for the best
    reinitialize_image_processor();
    
    // Reset frame states
    pending_frame_.reset();
    frame_available_.store(false);
    frame_requested_.store(false);
    processing_frame_.store(false);
    
    ESP_LOGI(TAG, "Basic camera recovery completed");
}

bool MeterReaderTFLite::camera_supports_window() const {
    if (!camera_) {
        ESP_LOGD(TAG, "Camera not initialized for window support check");
        return false;
    }
    
    bool supports = camera_window_control_.supports_window(camera_);
    ESP_LOGI(TAG, "Camera window support: %s", supports ? "YES" : "NO");
    return supports;
}

void MeterReaderTFLite::reinitialize_image_processor() {
    if (image_processor_) {
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

// force_flash_inference removed - use FlashLightController


}  // namespace meter_reader_tflite
}  // namespace esphome