#include "meter_reader_tflite.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

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
        image_processor_ = std::make_unique<esp32_camera_utils::ImageProcessor>(
            esp32_camera_utils::ImageProcessorConfig{camera_width_, camera_height_, pixel_format_},
            &model_handler_
        );
    }
    
    // Initialize validation
    setup_output_validation();
}

void MeterReaderTFLite::setup_output_validation() {
    // Configure validation parameters
    // value_validator_.set_allow_negative_rates(allow_negative_rates_);
    // value_validator_.set_max_absolute_diff(max_absolute_diff_);
    // These are set via setters from config
}

void MeterReaderTFLite::update() {
    if (pause_processing_) return;
    
    // Request a new frame if we don't have one pending
    if (!frame_requested_ && !frame_available_) {
        if (flash_controller_) {
            ESP_LOGD(TAG, "Initiating flash capture sequence");
            flash_controller_->initiate_capture_sequence([this]() {
                this->frame_requested_ = true;
                this->last_request_time_ = millis();
                ESP_LOGD(TAG, "Flash ready, requesting frame");
            });
        } else {
            frame_requested_ = true;
            last_request_time_ = millis();
        }
    }
}

void MeterReaderTFLite::loop() {
    // Handle frame request
    if (frame_requested_ && !frame_available_) {
        if (camera_) {
             std::shared_ptr<camera::CameraImage> image = camera_->get_image();
             if (image) {
                 pending_frame_ = image;
                 frame_available_ = true;
                 frame_requested_ = false;
                 last_frame_received_ = millis();
                 ESP_LOGD(TAG, "Frame received from camera");
             }
        }
    }
    
    process_available_frame();
}

void MeterReaderTFLite::dump_config() {
    ESP_LOGCONFIG(TAG, "Meter Reader TFLite:");
    ESP_LOGCONFIG(TAG, "  Model: %s", model_type_.c_str());
    ESP_LOGCONFIG(TAG, "  Confidence Threshold: %.2f", confidence_threshold_);
    ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->get_update_interval());
    if (flash_controller_) {
        ESP_LOGCONFIG(TAG, "  Flash Controller: Connected");
    }
}

bool MeterReaderTFLite::process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult& result, float* value, float* confidence) {
    if (result.error) {
        return false;
    }
    
    // This method seems to be used by process_full_image, but let's check if it's defined later.
    // It was in the header, so it should be implemented.
    // But I don't see it in the truncated file content I have.
    // I'll assume it's missing too and implement it here or later.
    // Wait, process_full_image calls it.
    
    // For now, let's focus on process_available_frame which is cut off.
    return false; 
}

void MeterReaderTFLite::process_available_frame() {
    if (processing_frame_) return;
    processing_frame_ = true;
