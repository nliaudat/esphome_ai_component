#include "meter_reader_tflite.h"
#include "model_config.h"
#include <sstream>
#include <iomanip>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace meter_reader_tflite {

using namespace esphome::esp32_camera_utils;
using namespace esphome::tflite_micro_helper;

static const char *const TAG = "meter_reader_tflite";

// #define DEBUG_METER_READER_TFLITE

#ifdef DURATION_START
#undef DURATION_START
#endif
#ifdef DURATION_END
#undef DURATION_END
#endif

#define DURATION_START() uint32_t start_time = millis()
#define DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)

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

    // Store original camera dimensions and format
    original_camera_width_ = camera_width_;
    original_camera_height_ = camera_height_;
    original_pixel_format_ = pixel_format_;

    ESP_LOGI(TAG, "Stored original camera dimensions: %dx%d, format: %s", 
             original_camera_width_, original_camera_height_, original_pixel_format_.c_str());

    // Setup output validation
    setup_output_validation();

    // Setup crop zones - the handler will manage everything
    // Note: crop_zones_global_ is passed via set_crop_zones_global() from the YAML config
    // If we have a global variable, apply its initial value
    if (crop_zone_handler_.get_crop_zones_global()) {
        ESP_LOGI(TAG, "Crop zones global variable registered, applying initial value");
        crop_zone_handler_.apply_global_zones();
    }

    // If no zones were set from global, use default
    if (crop_zone_handler_.get_zones().empty()) {
        ESP_LOGI(TAG, "No crop zones configured, using default zone");
        crop_zone_handler_.set_default_zone(camera_width_, camera_height_);
    }

    // Apply camera window configuration if provided 
    if (camera_window_configured_) {
        ESP_LOGI(TAG, "Camera window configuration found, will apply after camera setup");
        // Schedule camera window setup after a delay to ensure camera is ready
        this->set_timeout(3000, [this]() {
            ESP_LOGI(TAG, "Applying YAML camera window configuration");
            if (!this->set_camera_window(camera_window_offset_x_, camera_window_offset_y_, 
                                       camera_window_width_, camera_window_height_)) {
                ESP_LOGE(TAG, "Failed to apply camera window from YAML configuration");
            }
        });
    }

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
        
        // ADAPTATION: Use local ImageProcessor constructor
        ImageProcessorConfig config;
        config.camera_width = camera_width_;
        config.camera_height = camera_height_;
        config.pixel_format = pixel_format_;
        config.model_width = model_handler_.get_input_width();
        config.model_height = model_handler_.get_input_height();
        config.model_channels = model_handler_.get_input_channels();
        
        // Set rotation based on configuration
        switch(rotation_) {
            case 90:  config.rotation = ROTATION_90;  break;
            case 180: config.rotation = ROTATION_180; break;
            case 270: config.rotation = ROTATION_270; break;
            default:  config.rotation = ROTATION_0;   break;
        }
        
        TfLiteTensor* input = model_handler_.input_tensor();
        if (input->type == kTfLiteFloat32) {
            config.input_type = kInputTypeFloat32;
        } else {
            config.input_type = kInputTypeUInt8;
        }
        config.normalize = model_handler_.get_config().normalize;

        image_processor_ = std::make_unique<ImageProcessor>(config);

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


        } 
        // else {
        //     ESP_LOGE(TAG, "No debug image set to process.");
        // }

        #endif


    });
}

void MeterReaderTFLite::setup_output_validation() {
    ValueValidator::ValidationConfig validation_config;
    validation_config.allow_negative_rates = allow_negative_rates_;
    validation_config.max_absolute_diff = max_absolute_diff_;
    validation_config.max_rate_change = 0.15f; // 15% maximum change per reading
    validation_config.enable_smart_validation = true;
    validation_config.smart_validation_window = 5;

    output_validator_.set_config(validation_config);
    output_validator_.setup();

    ESP_LOGI(TAG, "Output validation configured - AllowNegativeRates: %s, MaxAbsoluteDiff: %d",
             allow_negative_rates_ ? "true" : "false", max_absolute_diff_);
}

bool MeterReaderTFLite::validate_and_update_reading(float raw_reading, float confidence, float& validated_reading) {
    // Convert to integer for precise validation
    int int_reading = static_cast<int>(raw_reading);
    int validated_int_reading = int_reading;

    bool is_valid = output_validator_.validate_reading(int_reading, confidence, validated_int_reading);

    // Convert back to float for sensor publishing
    validated_reading = static_cast<float>(validated_int_reading);

    return is_valid;
}

void MeterReaderTFLite::update() {

    // Check for updated crop zones from global variable using the handler
    if (crop_zone_handler_.has_global_zones_changed()) {
        ESP_LOGI(TAG, "Crop zones global variable changed, updating...");
        crop_zone_handler_.apply_global_zones();
    }

    // Skip update if system not ready
    if (!model_loaded_ || !camera_) {
        ESP_LOGW(TAG, "Update skipped - system not ready");
        return;
    }

    // Schedule flash first, then request frame after flash turns on
    if (flash_controller_) {
        // Use the external flash controller
        if (!flash_controller_->is_active() && !pause_processing_.load()) {
            ESP_LOGI(TAG, "Initiating flash capture sequence via controller");
            flash_controller_->initiate_capture_sequence([this]() {
                ESP_LOGI(TAG, "Flash controller callback: Requesting frame");
                if (!frame_available_.load() && !frame_requested_.load()) {
                    frame_requested_.store(true);
                    last_request_time_ = millis();
                }
            });

            return; // Exit update(), let controller drive the capture
        } else if (flash_controller_->is_active()) {
            ESP_LOGD(TAG, "Flash controller is active, waiting...");
            return; // Already running
        }
    } else if (flash_light_ && flash_light_enabled_ && !flash_scheduled_ && !pause_processing_.load()) {
        // Legacy internal flash handling
        // get_update_interval() already returns milliseconds!
        uint32_t update_interval_ms = get_update_interval();
        uint32_t schedule_time = update_interval_ms - flash_pre_time_;

        ESP_LOGI(TAG, "Flash scheduling: interval=%ums, pre_time=%ums, schedule_time=%ums", 
                update_interval_ms, flash_pre_time_, schedule_time);

        if (schedule_time > 0 && schedule_time < update_interval_ms) {
            // Calculate disable time here so it's in scope
            uint32_t disable_time = flash_pre_time_ + flash_post_time_;

            ESP_LOGI(TAG, "Flash scheduled: ON in %ums, OFF in %ums", 
                    schedule_time, disable_time);

            // Capture disable_time in the lambda
            this->set_timeout(schedule_time, [this, disable_time]() {
                ESP_LOGI(TAG, "Enabling flash as scheduled");
                this->enable_flash_light();

                // Request frame AFTER flash turns on (with your 7-second stabilization time)
                // Wait most of the pre_time for stabilization, then capture
                uint32_t capture_delay = flash_pre_time_ - 500; // Capture 500ms before update
                this->set_timeout(capture_delay, [this, disable_time]() {
                    ESP_LOGI(TAG, "Requesting frame after flash stabilization");
                    if (!frame_available_.load() && !frame_requested_.load()) {
                        frame_requested_.store(true);
                        last_request_time_ = millis();
                    }
                });

                // Schedule flash disable
                this->set_timeout(disable_time, [this]() {
                    ESP_LOGI(TAG, "Disabling flash as scheduled");
                    this->disable_flash_light();
                    flash_scheduled_ = false;
                });
            });
            flash_scheduled_ = true;

            // Don't request frame here - wait for flash to turn on and stabilize first
            ESP_LOGI(TAG, "Flash scheduled, frame will be requested after stabilization");
            return; // Exit update early, frame will be requested after flash turns on and stabilizes
        } else {
            ESP_LOGW(TAG, "Invalid schedule time: %ums (interval: %ums)", 
                    schedule_time, update_interval_ms);
        }
    } else {
        ESP_LOGD(TAG, "Flash not scheduled - enabled:%s scheduled:%s paused:%s", 
                flash_light_enabled_ ? "Y" : "N",
                flash_scheduled_ ? "Y" : "N",
                pause_processing_.load() ? "Y" : "N");
    }

    // If flash is not enabled or not scheduled, request frame normally
    // Reset processing state for new update cycle
    processing_frame_.store(false);

    // Request new frame if none available or pending
    if (!frame_available_.load() && !frame_requested_.load()) {
        frame_requested_.store(true);
        last_request_time_ = millis();
        ESP_LOGD(TAG, "Requesting new frame (no flash)");
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
        this->disable_flash_light();
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

        bool processing_success = true;


        // Process image through pipeline
        
        // If camera window is configured, we need to transform the global crop zones (which are based on full sensor)
        // to local window coordinates (which are based on the cropped buffer we just received)
        std::vector<CropZone> effective_zones;
        
        if (camera_window_configured_) {
            ESP_LOGD(TAG, "Transforming %d zones for camera window (offset: %d,%d, size: %dx%d)", 
                     zones.size(), camera_window_offset_x_, camera_window_offset_y_, 
                     camera_window_width_, camera_window_height_);
            
            for (const auto& zone : zones) {
                // Smart Detection of Coordinate Space
                // If the zone fits within the current window dimensions as-is, assume it is ALREADY Local.
                // If it doesn't, but fits if we subtract the offset, assume it is Global.
                
                bool is_valid_local = (zone.x2 <= camera_width_ && zone.y2 <= camera_height_);
                
                // For global check, we need to see if it makes sense as a global coordinate
                // i.e., strictly greater than offset (plus some margin to avoid ambiguity around 0)
                bool looks_like_global = (zone.x1 >= camera_window_offset_x_ || zone.y1 >= camera_window_offset_y_);

                if (is_valid_local && !looks_like_global) {
                     // Unambiguously local (e.g. x=10 with offset=928)
                     // Keep as is
                     effective_zones.push_back(zone);
                     ESP_LOGV(TAG, "  Zone [%d,%d->%d,%d] treated as Local (fits in window)", 
                              zone.x1, zone.y1, zone.x2, zone.y2);
                } 
                else if (looks_like_global) {
                    // Try transforming
                    CropZone local_zone = zone;
                    local_zone.x1 -= camera_window_offset_x_;
                    local_zone.y1 -= camera_window_offset_y_;
                    local_zone.x2 -= camera_window_offset_x_;
                    local_zone.y2 -= camera_window_offset_y_;
                    
                    // Clip
                    local_zone.x1 = std::max(0, local_zone.x1);
                    local_zone.y1 = std::max(0, local_zone.y1);
                    local_zone.x2 = std::min(camera_width_, local_zone.x2);
                    local_zone.y2 = std::min(camera_height_, local_zone.y2);

                    if (local_zone.x2 > local_zone.x1 && local_zone.y2 > local_zone.y1) {
                         effective_zones.push_back(local_zone);
                         ESP_LOGD(TAG, "  Zone mapped from Global: [%d,%d] -> Local: [%d,%d] (%dx%d)", 
                                  zone.x1, zone.y1, local_zone.x1, local_zone.y1,
                                  local_zone.x2 - local_zone.x1, local_zone.y2 - local_zone.y1);
                    } else {
                         ESP_LOGW(TAG, "  Zone [%d,%d] (Global?) falls outside camera window - skipping", 
                                  zone.x1, zone.y1);
                    }
                }
                else if (is_valid_local) {
                    // Ambiguous case, but fits as local. Default to preserving user config as local.
                    effective_zones.push_back(zone);
                    ESP_LOGD(TAG, "  Zone [%d,%d] treated as Local (Ambiguous but fits) (%dx%d)", 
                             zone.x1, zone.y1, zone.x2-zone.x1, zone.y2-zone.y1);
                }
                else {
                    ESP_LOGW(TAG, "  Zone [%d,%d->%d,%d] invalid for window %dx%d (Offset %d,%d)", 
                             zone.x1, zone.y1, zone.x2, zone.y2, 
                             camera_width_, camera_height_,
                             camera_window_offset_x_, camera_window_offset_y_);
                }
            }
        } else {
            effective_zones = zones;
            for (const auto& z : effective_zones) {
                 ESP_LOGD(TAG, "Zone Global: [%d,%d->%d,%d] Size: %dx%d", 
                          z.x1, z.y1, z.x2, z.y2, z.x2-z.x1, z.y2-z.y1);
            }
        }

        if (effective_zones.empty()) {
             ESP_LOGW(TAG, "No valid zones after window transformation");
             processing_success = false;
        }

        auto processed_zones = image_processor_->split_image_in_zone(frame, effective_zones);

        std::vector<float> readings;
        std::vector<float> confidences;


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

            // Validate the reading using output validator
            float validated_reading = final_reading;
            bool is_valid = validate_and_update_reading(final_reading, avg_confidence, validated_reading);

            // Store the values for template sensors
            last_reading_ = validated_reading;
            last_confidence_ = avg_confidence;

            ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)", 
                final_reading, validated_reading, is_valid ? "yes" : "no", 
                avg_confidence * 100.0f, confidence_threshold_ * 100.0f);


            if (inference_logs_) {
                // Publish to inference logs text sensor
                char inference_log[150];
                snprintf(inference_log, sizeof(inference_log),
                         "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)",
                         final_reading, validated_reading, is_valid ? "yes" : "no",
                         avg_confidence * 100.0f, confidence_threshold_ * 100.0f);
                inference_logs_->publish_state(inference_log);

            }

            // Only publish to sensors if confidence meets threshold AND reading is valid
            if (avg_confidence >= confidence_threshold_ && is_valid) {
                if (value_sensor_) {
                    value_sensor_->publish_state(validated_reading);
                }

                 if (confidence_sensor_ != nullptr) {
                     confidence_sensor_->publish_state(avg_confidence * 100.0f); // Convert to percentage
                 }


                ESP_LOGI(TAG, "Reading published - valid and confidence threshold met");
            } else {
                ESP_LOGW(TAG, "Reading NOT published - %s", 
                        !is_valid ? "validation failed" : "confidence below threshold");
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
    // Copy input data to model input tensor
    TfLiteTensor* input = model_handler_.input_tensor();
    if (!input) {
        ESP_LOGE(TAG, "Failed to get input tensor");
        return false;
    }

    // Ensure input size matches
    if (result.size != input->bytes) {
        ESP_LOGE(TAG, "Input size mismatch: expected %zu, got %zu", input->bytes, result.size);
        return false;
    }

    memcpy(input->data.uint8, result.data->get(), result.size);

    // Invoke model
    // Log basic input stats for debugging zero confidence
    // model_handler_.log_input_stats(); // Don't spam unless debug is high, OR if confidence is weird
    
    if (model_handler_.invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Model invocation failed");
        return false;
    }

    // Get output tensor
    TfLiteTensor* output_tensor = model_handler_.output_tensor();
    if (!output_tensor) {
        ESP_LOGE(TAG, "Failed to get output tensor");
        return false;
    }

    // Process output
    ProcessedOutput output = model_handler_.process_output(output_tensor);
    *value = output.value;
    *confidence = output.confidence;

    if (*confidence < 0.0001f) {
        // Suspiciously low confidence, dump stats
        ESP_LOGW(TAG, "Zero confidence detected. Dumping input stats:");
        model_handler_.log_input_stats();
    }

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
// void MeterReaderTFLite::print_debug_info() {
    // print_meter_reader_debug_info(this);
// }

bool MeterReaderTFLite::load_model() {
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
    if (!model_handler_.load_model_with_arena(model_, model_length_, 
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
    crop_zone_handler_.update_zones(zones_json); // This now updates both internal state AND global variable

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

void MeterReaderTFLite::enable_flash_light() {
    if (flash_controller_) {
        ESP_LOGI(TAG, "Enabling flash light (via controller)");
        if (flash_light_) {
             ESP_LOGI(TAG, "Enabling flash light directly");
             flash_auto_controlled_.store(true);
             auto call = flash_light_->turn_on();
             // call.set_brightness(1.0f); // Full brightness
             call.set_transition_length(0);
             call.perform();
        } else {
             ESP_LOGW(TAG, "Flash controller present but no direct light access for forced inference. Assuming controller handles it or light not passed to MeterReader.");
        }
    } else if (flash_light_) {
        ESP_LOGI(TAG, "Enabling flash light");
        flash_auto_controlled_.store(true);
        auto call = flash_light_->turn_on();
        // call.set_brightness(1.0f); // Full brightness
        call.set_transition_length(0);
        call.perform();
    } else {
        ESP_LOGW(TAG, "Flash light is null - cannot enable");
    }
}

bool MeterReaderTFLite::is_flash_forced_on() const {
    if (!flash_light_ || !flash_light_enabled_) {
        return false;
    }

    // If flash is on but not marked as auto-controlled, it's forced on
    bool is_currently_on = flash_light_->current_values.is_on();
    if (is_currently_on && !flash_auto_controlled_.load()) {
        return true;
    }

    return false;
}

void MeterReaderTFLite::disable_flash_light() {
    if (flash_light_ && flash_auto_controlled_.load()) {
        ESP_LOGI(TAG, "Disabling flash light");
        flash_auto_controlled_.store(false);
        auto call = flash_light_->turn_off();
        call.set_transition_length(0);
        call.perform();
    } else {
        ESP_LOGD(TAG, "Flash not auto-controlled or null - skip disable");
    }
}

/* void MeterReaderTFLite::schedule_flash_light_operations() {
    if (!flash_light_ || !flash_light_enabled_) {
        return;
    }

    // Enable flash light with a small delay to ensure it's on before capture
    this->set_timeout(50, [this]() {
        this->enable_flash_light();

        // Schedule flash disable after capture (flash_duration_ milliseconds)
        this->set_timeout(flash_duration_, [this]() {
            this->disable_flash_light();
        });
    });
} */

// Setter method for the flash light
void MeterReaderTFLite::set_flash_light(light::LightState* flash_light) {
    flash_light_ = flash_light;
    flash_light_enabled_ = (flash_light_ != nullptr);
    ESP_LOGI(TAG, "Flash light %s", flash_light_enabled_ ? "configured" : "disabled");
}

void MeterReaderTFLite::set_flash_controller(flash_light_controller::FlashLightController* controller) {
    flash_controller_ = controller;
    ESP_LOGI(TAG, "Flash controller %s", flash_controller_ ? "configured" : "disabled");
}

// set flash duration
// void MeterReaderTFLite::set_flash_duration(uint32_t duration_ms) {
    // flash_duration_ = duration_ms;
    // ESP_LOGI(TAG, "Flash duration set to %ums", duration_ms);
// }


// ###### camera parameters

bool MeterReaderTFLite::set_camera_window(int offset_x, int offset_y, int width, int height) {
    ESP_LOGI(TAG, "Setting camera window to: offset(%d,%d), size(%dx%d)", 
             offset_x, offset_y, width, height);

    // Pause processing during camera reconfiguration
    bool was_paused = get_pause_processing();
    set_pause_processing(true);

    // Clear pending frames
    {
        std::shared_ptr<camera::CameraImage> frame;
        frame = pending_frame_;
        pending_frame_.reset();
        frame_available_.store(false);
        frame_requested_.store(false);
    }

    // Use reset before setting new window
    bool success = camera_window_control_.set_window_with_reset(
        camera_, CameraWindowControl::WindowConfig{
            offset_x, offset_y, width, height, true});

    if (success) {
        // Update dimensions
        auto new_dims = camera_window_control_.update_dimensions_after_window(
            camera_, 
            CameraWindowControl::WindowConfig{offset_x, offset_y, width, height, true},
            camera_width_, camera_height_);

        camera_width_ = new_dims.first;
        camera_height_ = new_dims.second;

        // Stabilization delay
        delay(500);

        reinitialize_image_processor();

        ESP_LOGI(TAG, "Camera window set successfully to %dx%d", width, height);

        // Test the new configuration
        test_camera_after_reset();
    } else {
        ESP_LOGE(TAG, "Failed to set camera window");
        // Try to recover
        reset_camera_window();
    }

    // Restore processing state
    set_pause_processing(was_paused);

    return success;
}

/* bool MeterReaderTFLite::set_camera_window_from_crop_zones() {
    auto zones = crop_zone_handler_.get_zones();
    if (zones.empty()) {
        ESP_LOGI(TAG, "No crop zones available for camera window");
        return false;
    }

    ESP_LOGI(TAG, "Setting camera window from %d crop zones", zones.size());

    // Pause processing during camera reconfiguration
    bool was_paused = get_pause_processing();
    set_pause_processing(true);

    // Clear pending frames
    {
        std::shared_ptr<camera::CameraImage> frame;
        frame = pending_frame_;
        pending_frame_.reset();
        frame_available_.store(false);
        frame_requested_.store(false);
    }

    bool success = camera_window_control_.set_window_from_crop_zones_with_dimensions(
        camera_, zones, camera_width_, camera_height_);

    if (success) {
        // Stabilization delay
        delay(500);

        reinitialize_image_processor();

        ESP_LOGI(TAG, "Camera window set from crop zones successfully: %dx%d", 
                 camera_width_, camera_height_);

        // Test the new configuration
        test_camera_after_reset();
    } else {
        ESP_LOGE(TAG, "Failed to set camera window from crop zones");
        // Try to recover
        reset_camera_window();
    }

    // Restore processing state
    set_pause_processing(was_paused);

    return success;
} */

bool MeterReaderTFLite::reset_camera_window() {
    ESP_LOGI(TAG, "Resetting camera window to full frame...");

    // Pause processing during camera reset to prevent frame processing issues
    bool was_paused = get_pause_processing();
    set_pause_processing(true);

    // Clear any pending frames to avoid processing during reset
    {
        std::shared_ptr<camera::CameraImage> frame;
        frame = pending_frame_;
        pending_frame_.reset();
        frame_available_.store(false);
        frame_requested_.store(false);
    }

    // Method 1: Try hard reset first (most thorough)
    ESP_LOGI(TAG, "Attempting hard camera reset...");
    bool success = camera_window_control_.hard_reset_camera(camera_);

    if (!success) {
        ESP_LOGW(TAG, "Hard reset failed, trying soft reset...");
        // Method 2: Try soft reset
        success = camera_window_control_.soft_reset_camera(camera_);
    }

    if (success) {
        ESP_LOGI(TAG, "Camera reset successful, setting full frame dimensions...");

        // Reset to full frame with dimensions
        success = camera_window_control_.reset_to_full_frame_with_dimensions(
            camera_, original_camera_width_, original_camera_height_, 
            camera_width_, camera_height_);

        if (success) {
            pixel_format_ = original_pixel_format_;

            // Give camera extra time to stabilize after reset
            ESP_LOGI(TAG, "Waiting for camera stabilization after reset...");
            delay(1000); // 1 second stabilization

            reinitialize_image_processor();

            ESP_LOGI(TAG, "Camera window reset to original: %dx%d, format: %s", 
                     camera_width_, camera_height_, pixel_format_.c_str());

            // Test camera functionality
            if (test_camera_after_reset()) {
                ESP_LOGI(TAG, "Camera test after reset: SUCCESS");
            } else {
                ESP_LOGW(TAG, "Camera test after reset: WARNING - camera may need more time");
            }
        }
    }

    // Restore processing state
    set_pause_processing(was_paused);

    if (!success) {
        ESP_LOGE(TAG, "Failed to reset camera window completely");
        // Try one more time with basic reset
        basic_camera_recovery();
    }

    return success;
}

bool MeterReaderTFLite::test_camera_after_reset() {
    ESP_LOGI(TAG, "Testing camera after reset...");

    // Simple test: request a frame and see if we get one within timeout
    frame_requested_.store(true);
    uint32_t start_time = millis();
    const uint32_t timeout = 5000; // 5 second timeout

    while (millis() - start_time < timeout) {
        if (frame_available_.load()) {
            ESP_LOGI(TAG, "Camera test: Frame received successfully");
            frame_available_.store(false);
            frame_requested_.store(false);
            return true;
        }
        delay(100);
    }

    ESP_LOGW(TAG, "Camera test: No frame received within timeout");
    frame_requested_.store(false);
    return false;
}

void MeterReaderTFLite::basic_camera_recovery() {
    ESP_LOGI(TAG, "Attempting basic camera recovery...");

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
        // ADAPTATION: Use local ImageProcessor constructor
        ImageProcessorConfig config;
        config.camera_width = camera_width_;
        config.camera_height = camera_height_;
        config.pixel_format = pixel_format_;
        config.model_width = model_handler_.get_input_width();
        config.model_height = model_handler_.get_input_height();
        config.model_channels = model_handler_.get_input_channels();
        
        // Set rotation based on configuration
        switch(rotation_) {
            case 90:  config.rotation = ROTATION_90;  break;
            case 180: config.rotation = ROTATION_180; break;
            case 270: config.rotation = ROTATION_270; break;
            default:  config.rotation = ROTATION_0;   break;
        }
        
        TfLiteTensor* input = model_handler_.input_tensor();
        if (input && input->type == kTfLiteFloat32) {
            config.input_type = kInputTypeFloat32;
        } else {
            config.input_type = kInputTypeUInt8;
        }
        config.normalize = model_handler_.get_config().normalize;
        config.input_order = model_handler_.get_config().input_order;

        image_processor_ = std::make_unique<ImageProcessor>(config);
        ESP_LOGI(TAG, "ImageProcessor reinitialized with dimensions: %dx%d, format: %s, rotation: %d deg",
                 camera_width_, camera_height_, pixel_format_.c_str(), rotation_);
    } else {
        ESP_LOGW(TAG, "ImageProcessor not available for reinitialization");
    }
}

void MeterReaderTFLite::force_flash_inference() {
  // -----------------------------------------------------------------------
  // Make sure the flash hardware is configured
  // -----------------------------------------------------------------------
  if ((!flash_light_ || !flash_light_enabled_) && !flash_controller_) {
    ESP_LOGW(TAG, "Force inference requested but flash light is not configured");
    return;
  }

  // -----------------------------------------------------------------------
  // Cancel any pending frame request and clear the “paused” flag
  // -----------------------------------------------------------------------
  set_pause_processing(false);          // ensure AI is allowed to run
  frame_requested_.store(false);
  frame_available_.store(false);
  pending_frame_.reset();

  // -----------------------------------------------------------------------
  // Turn the flash on immediately
  // -----------------------------------------------------------------------
  ESP_LOGI(TAG, "Force inference: enabling flash");
  enable_flash_light();                 // protected → we are inside the class

  // -----------------------------------------------------------------------
  // After 3 seconds request a fresh frame
  // -----------------------------------------------------------------------
  const uint32_t FLASH_WAIT_MS = 3000;   // 3 seconds illumination time
  this->set_timeout(FLASH_WAIT_MS, [this]() {
    ESP_LOGI(TAG, "Force inference: requesting frame after flash warm up");

    // Request a frame (same logic that `update()` uses)
    if (!frame_requested_.load() && !frame_available_.load()) {
      frame_requested_.store(true);
      last_request_time_ = millis();
    }

    // -------------------------------------------------------------------
    // When the frame has been processed we turn the flash off.
    //     We don’t know exactly when the processing finishes, so we
    //     schedule a *short* safety timeout (500 ms) after the request.
    // -------------------------------------------------------------------
    const uint32_t SAFETY_TIMEOUT_MS = 500;
    this->set_timeout(SAFETY_TIMEOUT_MS, [this]() {
      // If the AI is still busy we simply turn the flash off – the
      // next normal update will re enable it if needed.
      ESP_LOGI(TAG, "Force inference: disabling flash");
      disable_flash_light();
    });
  });
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
            std::vector<std::vector<uint8_t>> zone_data;

            for (size_t zone_idx = 0; zone_idx < processed_zones.size(); zone_idx++) {
                auto& zone_result = processed_zones[zone_idx];
                zone_data.push_back(std::vector<uint8_t>(
                    zone_result.data->get(), 
                    zone_result.data->get() + zone_result.size
                ));

                ESP_LOGI(TAG, "Zone %d: %zu bytes processed", zone_idx + 1, zone_result.size);
            }
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

    std::vector<std::vector<uint8_t>> zone_data_pattern;
    zone_data_pattern.push_back(test_pattern);
    model_handler_.debug_test_parameters(zone_data_pattern);
}

void MeterReaderTFLite::set_debug_mode(bool debug_mode) {
    debug_mode_ = debug_mode;
    ESP_LOGI(TAG, "Debug mode %s", debug_mode ? "enabled" : "disabled");
}
#endif

}  // namespace meter_reader_tflite
}  // namespace esphome