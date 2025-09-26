/**
 * @file meter_reader_tflite.cpp
 * @brief Implementation of MeterReaderTFLite component for ESPHome.
 * 
 * This file contains the implementation of the meter reading component
 * that uses TensorFlow Lite Micro for digit recognition from camera images.
 */

#include "meter_reader_tflite.h"
#include "esp_log.h"
#include "debug_utils.h"
#include "model_config.h"
#include <numeric>

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

void MeterReaderTFLite::setup() {
    ESP_LOGI(TAG, "Setting up Meter Reader TFLite...");
    
    // Validate essential configuration
    if (camera_width_ == 0 || camera_height_ == 0) {
        ESP_LOGE(TAG, "Camera dimensions not set!");
        mark_failed();
        return;
    }
    
    if (!camera_) {
        ESP_LOGE(TAG, "No camera configured!");
        mark_failed();
        return;
    }
    
    // Apply crop zones from global variable if available
    crop_zone_handler_.apply_global_zones();

    // Setup camera callback with frame buffer management
    camera_->add_image_callback([this](std::shared_ptr<camera::CameraImage> image) {
        // Only accept frames if requested and not currently processing
        if (frame_requested_.load() && !processing_frame_.load()) {
            pending_frame_ = image;
            frame_available_.store(true);
            frame_requested_.store(false);
            last_frame_received_ = millis();
            ESP_LOGD(TAG, "Frame captured successfully");
        } else {
            frames_skipped_++;
        }
    });

    // Delay model loading to allow system stabilization
    ESP_LOGI(TAG, "Model loading will begin in 30 seconds...");
    this->set_timeout(30000, [this]() {
        ESP_LOGI(TAG, "Starting model loading...");
        
        if (!this->load_model()) {
            ESP_LOGE(TAG, "Failed to load model");
            this->mark_failed();
            return;
        }

        this->model_loaded_ = true;
        
        ESP_LOGI(TAG, "Setting up Image Processor...");
        image_processor_ = std::make_unique<ImageProcessor>(
            ImageProcessorConfig{camera_width_, camera_height_, pixel_format_},
            &model_handler_
        );
            
        ESP_LOGI(TAG, "Meter Reader TFLite setup complete");
        this->print_debug_info();
        
        // Process debug image AFTER ImageProcessor is initialized
        #ifdef DEBUG_METER_READER_TFLITE
        if (debug_image_) {
            ESP_LOGI(TAG, "Processing debug image after setup completion");
            this->set_timeout(1000, [this]() { // Small delay to ensure everything is ready
                this->test_with_debug_image();
            });
            
        //check all configs
        //model_handler_.debug_test_parameters(debug_image_->get_data_buffer(), debug_image_->get_data_length());
            

        } else {
            ESP_LOGE(TAG, "No debug image set to process.");
        }
        
        #endif
        
        
    });
}

void MeterReaderTFLite::update() {
    // Skip update if system not ready
    if (!model_loaded_ || !camera_) {
        ESP_LOGW(TAG, "Update skipped - system not ready");
        return;
    }

    // Reset processing state for new update cycle
    processing_frame_.store(false);
    
    // Request new frame if none available or pending
    if (!frame_available_.load() && !frame_requested_.load()) {
        frame_requested_.store(true);
        last_request_time_ = millis();
        ESP_LOGD(TAG, "Requesting new frame");
    } else if (frame_available_.load()) {
        // Process existing frame immediately
        ESP_LOGD(TAG, "Processing available frame");
        process_available_frame();
    }
}

void MeterReaderTFLite::loop() {
    // Process available frame if update() has triggered processing
    if (frame_available_.load() && !processing_frame_.load()) {
        process_available_frame();
    }

    // Handle frame request timeout (10 seconds)
    if (frame_requested_.load() && (millis() - last_request_time_ > 10000)) {
        ESP_LOGW(TAG, "Frame request timeout - no frame received in 10 seconds");
        frame_requested_.store(false);
        frames_skipped_++;
    }
}

void MeterReaderTFLite::process_available_frame() {
    processing_frame_.store(true);
    
    std::shared_ptr<camera::CameraImage> frame;
    {
        // Atomic frame retrieval
        frame = pending_frame_;
        pending_frame_.reset();
        frame_available_.store(false);
    }
    
    if (frame && frame->get_data_buffer() && frame->get_data_length() > 0) {
        ESP_LOGD(TAG, "Processing frame (%zu bytes)", frame->get_data_length());
        process_full_image(frame);
        frames_processed_++;
    } else {
        ESP_LOGE(TAG, "Invalid frame available for processing");
    }
    
    processing_frame_.store(false);
}

void MeterReaderTFLite::process_full_image(std::shared_ptr<camera::CameraImage> frame) {
    DURATION_START();
    
    // Check if processing is paused
    if (pause_processing_) {
        ESP_LOGI(TAG, "AI processing paused - skipping frame processing");
        DURATION_END("process_full_image (paused)");
        return;
    }
    
    // Validate input frame
    if (!frame || !frame->get_data_buffer() || frame->get_data_length() == 0) {
        ESP_LOGE(TAG, "Invalid frame received for processing");
        DURATION_END("process_full_image (invalid frame)");
        return;
    }
    
    // Check if ImageProcessor is ready
    if (!image_processor_) {
        ESP_LOGE(TAG, "ImageProcessor not initialized");
        DURATION_END("process_full_image (no processor)");
        return;
    }

    ESP_LOGD(TAG, "Processing frame (%zu bytes)", frame->get_data_length());

    // try {
        // Get configured crop zones or use full image
        auto zones = crop_zone_handler_.get_zones();
        if (zones.empty()) {
            zones.push_back({0, 0, camera_width_, camera_height_});
            ESP_LOGD(TAG, "No crop zones configured - using full frame");
        }

        // Process image through pipeline
        auto processed_zones = image_processor_->split_image_in_zone(frame, zones);
        
        std::vector<float> readings;
        std::vector<float> confidences;
        bool processing_success = true;

        // Process each zone through model
        for (auto& result : processed_zones) {
            float value, confidence;
            if (process_model_result(result, &value, &confidence)) {
                readings.push_back(value);
                confidences.push_back(confidence);
                
                ESP_LOGD(TAG, "Zone result - Value: %.1f, Confidence: %.2f", 
                        value, confidence);
            } else {
                ESP_LOGE(TAG, "Model result processing failed for zone");
                processing_success = false;
                break;
            }
        }

        // Publish results if successful
    if (processing_success && !readings.empty()) {
        float final_reading = combine_readings(readings);
        float avg_confidence = std::accumulate(confidences.begin(), 
                                             confidences.end(), 0.0f) / confidences.size();
        
        // Store the values for template sensors
        last_reading_ = final_reading;
        last_confidence_ = avg_confidence;
        
        ESP_LOGI(TAG, "Final reading: %.1f (avg confidence: %.2f)", 
                final_reading, avg_confidence);

        // Your existing publish_state calls (keep these if you want both methods)
        if (value_sensor_) {
            value_sensor_->publish_state(final_reading);
        }
        
        if (confidence_sensor_ != nullptr) {
            confidence_sensor_->publish_state(avg_confidence);
        }
        
        if (debug_mode_) {
            this->print_debug_info();
        }
    } else {
        ESP_LOGE(TAG, "Frame processing failed");
    }
    
    DURATION_END("process_full_image");
}

bool MeterReaderTFLite::process_model_result(const ImageProcessor::ProcessResult& result, 
                                           float* value, float* confidence) {
    // Invoke model with processed image data
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
    ESP_LOGD(TAG, "Camera format set: %dx%d, %s", width, height, pixel_format.c_str());
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

bool MeterReaderTFLite::load_model() {
    DURATION_START();
    ESP_LOGI(TAG, "Loading TFLite model...");
    
    // Get model configuration from model_config.h
    ModelConfig config;
    auto it = MODEL_CONFIGS.find(model_type_);
    if (it != MODEL_CONFIGS.end()) {
        config = it->second;
        ESP_LOGI(TAG, "Using model config: %s", config.description.c_str());
        
        // Use tensor arena size from model configuration if not explicitly set
        if (tensor_arena_size_requested_ == (50 * 1024)) { // If using default
            // Parse tensor arena size from model config string (e.g., "512KB")
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
                // Keep the default tensor_arena_size_requested_ value
            }
        }
    } else {
        config = DEFAULT_MODEL_CONFIG;
        ESP_LOGW(TAG, "Model type '%s' not found, using default config: %s", 
                model_type_.c_str(), config.description.c_str());
    }

    // Allocate tensor arena with the determined size
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }

    // Load the model
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

void MeterReaderTFLite::set_crop_zones(const std::string &zones_json) {
    ESP_LOGI(TAG, "Setting crop zones from JSON");
    crop_zone_handler_.parse_zones(zones_json);
    
    // Set default zone if none parsed
    if (crop_zone_handler_.get_zones().empty()) {
        ESP_LOGI(TAG, "No zones found in JSON, setting default zone");
        crop_zone_handler_.set_default_zone(camera_width_, camera_height_);
    }
    
    ESP_LOGI(TAG, "Configured %d crop zones", crop_zone_handler_.get_zones().size());
}

bool MeterReaderTFLite::allocate_tensor_arena() {
    ESP_LOGI(TAG, "Allocating tensor arena: %zu bytes", tensor_arena_size_requested_);
    
    tensor_arena_allocation_ = MemoryManager::allocate_tensor_arena(tensor_arena_size_requested_);
    if (!tensor_arena_allocation_) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return false;
    }
    
    ESP_LOGI(TAG, "Tensor arena allocated successfully: %zu bytes", 
            tensor_arena_allocation_.actual_size);
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
        return false;  // Debug image isn’t tied to requester
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
                zone_data.push_back(std::vector<uint8_t>(
                    zone_result.data->get(), 
                    zone_result.data->get() + zone_result.size
                ));
                
                ESP_LOGI(TAG, "Zone %d: %zu bytes processed", zone_idx + 1, zone_result.size);
            }
            
            // Test all configurations with all zones
            model_handler_.debug_test_parameters(zone_data);
        } else {
            ESP_LOGE(TAG, "Zone processing failed. Expected %d zones, got %d", 
                     debug_zones.size(), processed_zones.size());
        }
    } else {
        ESP_LOGE(TAG, "No debug image set to process.");
    }
}

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
    
    model_handler_.debug_test_parameters(test_pattern.data(), input_size);
}



void MeterReaderTFLite::set_debug_mode(bool debug_mode) {
    debug_mode_ = debug_mode;
    ESP_LOGI(TAG, "Debug mode %s", debug_mode ? "enabled" : "disabled");
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome