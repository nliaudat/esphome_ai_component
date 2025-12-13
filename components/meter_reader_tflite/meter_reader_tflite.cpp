#include "meter_reader_tflite.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <numeric>

#ifdef USE_WEB_SERVER
#include "esphome/components/esp32_camera_utils/preview_web_handler.h"
#endif

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

void MeterReaderTFLite::setup() {
    ESP_LOGI(TAG, "Setting up Meter Reader TFLite (Refactored)...");
    
    // 1. Initial Config
    if (camera_coord_.get_width() == 0) {
        ESP_LOGE(TAG, "Camera dimensions not set!");
        mark_failed(); return;
    }
    
    // 2. Load Model
    // Load model first to ensure we can retrieve input specifications for camera configuration.
        
    this->set_timeout(30000, [this]() {
         if (!tflite_coord_.load_model()) {
             mark_failed(); return;
         }
         
         // After model loaded, we have input specs. Update CameraCoord.
         auto spec = tflite_coord_.get_model_spec();
         camera_coord_.update_image_processor_config(
             spec.input_width, 
             spec.input_height, 
             spec.input_channels,
             spec.input_type,
             spec.normalize,
             spec.input_order
         );
         
         // Setup Web Server Preview
         #ifdef USE_WEB_SERVER
         #ifdef DEV_ENABLE_ROTATION
         if (web_server_) {
             web_server_->add_handler(new esphome::esp32_camera_utils::PreviewWebHandler([this]() {
                 return this->get_preview_image();
             }));
         }
         #endif
         #endif
    });
    
    // 3. Setup Validation
    ValueValidator::ValidationConfig val_conf;
    val_conf.allow_negative_rates = allow_negative_rates_;
    val_conf.max_absolute_diff = max_absolute_diff_;
    output_validator_.set_config(val_conf);
    output_validator_.setup();
    
    // 4. Setup Camera Callback
    // Register callback on the global camera instance.
}

void MeterReaderTFLite::set_camera(esp32_camera::ESP32Camera *camera) {
    camera_coord_.set_camera(camera);
    // Register callback
    camera->add_image_callback([this](std::shared_ptr<camera::CameraImage> img) {
        if (frame_requested_.load() && !processing_frame_.load()) {
            pending_frame_ = img;
            frame_available_.store(true);
            frame_requested_.store(false);
        }
    });
}

void MeterReaderTFLite::update() {
    if (crop_zone_handler_.has_global_zones_changed()) {
        crop_zone_handler_.apply_global_zones();
    }
    
    // Delegate flash/scheduling to FlashlightCoordinator
    if (pause_processing_) return;
    
    // The flashlight coordinator returns true if it is handling the cycle (scheduling or waiting)
    bool busy = flashlight_coord_.update_scheduling();
    
    if (!busy) {
        // Normal cycle
         if (!frame_available_ && !frame_requested_) {
             frame_requested_ = true;
         } else if (frame_available_) {
             process_available_frame();
         }
    }
}

void MeterReaderTFLite::loop() {
    if (frame_available_ && !processing_frame_) {
        process_available_frame();
    }
    // Timeout logic could go here or in a coordinator
}

void MeterReaderTFLite::process_available_frame() {
    processing_frame_ = true;
    std::shared_ptr<camera::CameraImage> frame = pending_frame_;
    pending_frame_.reset();
    frame_available_ = false;
    
    if (frame) {
        process_full_image(frame);
    }
    processing_frame_ = false;
}

void MeterReaderTFLite::process_full_image(std::shared_ptr<camera::CameraImage> frame) {
    if (pause_processing_) return;
    
    // Preview Logic (Rotation)
    #ifdef DEV_ENABLE_ROTATION
    if (generate_preview_ || request_preview_) {
         // Create rotated preview via TFLite/ImageProcessor
         // Actually TFLiteCoord owns usage of ImageProcessor static methods too? 
         // Or we use ImageProcessor static directly.
         using namespace esphome::esp32_camera_utils;
         
         // Using dimensions from camera coord
         auto preview = ImageProcessor::generate_rotated_preview(
             frame, rotation_, camera_coord_.get_width(), camera_coord_.get_height());
             
         if (preview) {
             update_preview_image(std::static_pointer_cast<RotatedPreviewImage>(preview));
         }
         
         if (request_preview_) {
             request_preview_ = false;
             return; // Skip inference
         }
    }
    #endif

    // Inference
    auto zones = crop_zone_handler_.get_zones();
    if (zones.empty()) zones.push_back({0,0, camera_coord_.get_width(), camera_coord_.get_height()});
    

    
    // Process frame -> buffers
    auto processed_buffers = camera_coord_.process_frame(frame, zones);
    
    // Buffers -> Inference
    auto results = tflite_coord_.run_inference(processed_buffers);
    
    // Collect readings
    std::vector<float> readings, confidences;
    for (const auto& res : results) {
        if (res.success) {
            readings.push_back(res.value);
            confidences.push_back(res.confidence);
        }
    }
    
    if (!readings.empty()) {
        float final_val = combine_readings(readings);
        float avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / confidences.size();
        
        float validated_val = final_val;
        bool valid = validate_and_update_reading(final_val, avg_conf, validated_val);
        
        if (valid && avg_conf >= confidence_threshold_) {
             if (value_sensor_) value_sensor_->publish_state(validated_val);
             if (confidence_sensor_) confidence_sensor_->publish_state(avg_conf * 100.0f);
        }
    }
}

void MeterReaderTFLite::set_crop_zones(const std::string &zones_json) {
    crop_zone_handler_.update_zones(zones_json);
}

// Config Delegators
void MeterReaderTFLite::set_tensor_arena_size(size_t size) { tflite_coord_.set_tensor_arena_size(size); } 
void MeterReaderTFLite::set_model_config(const std::string& type) { tflite_coord_.set_model_type(type); }
void MeterReaderTFLite::set_model(const uint8_t *model, size_t length) { tflite_coord_.set_model(model, length); }

void MeterReaderTFLite::set_camera_image_format(int w, int h, const std::string &fmt) {
    camera_coord_.set_config(w, h, fmt);
    // TFLite coord also needs to know to init processor
    camera_coord_.set_config(w, h, fmt);
    // Needed? 
    if (tflite_coord_.is_model_loaded()) {
         auto spec = tflite_coord_.get_model_spec();
         camera_coord_.update_image_processor_config(
             spec.input_width, spec.input_height, spec.input_channels,
             spec.input_type, spec.normalize, spec.input_order);
    }
}

void MeterReaderTFLite::set_flash_light(light::LightState* light) {
    flashlight_coord_.setup(this, light, nullptr);
}
void MeterReaderTFLite::set_flash_controller(flash_light_controller::FlashLightController* c) {
    flashlight_coord_.setup(this, nullptr, c);
}
void MeterReaderTFLite::set_flash_pre_time(uint32_t ms) { flashlight_coord_.set_timing(ms, 2000); } // simplified
void MeterReaderTFLite::set_flash_post_time(uint32_t ms) { flashlight_coord_.set_timing(5000, ms); }

// Preview
#ifdef DEV_ENABLE_ROTATION
void MeterReaderTFLite::take_preview_image() { capture_preview(); }
void MeterReaderTFLite::capture_preview() {
    request_preview_ = true;
    flashlight_coord_.capture_preview_sequence([this](){
        frame_requested_ = true;
    });
}
std::shared_ptr<camera::CameraImage> MeterReaderTFLite::get_preview_image() {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    return last_preview_image_;
}
void MeterReaderTFLite::update_preview_image(std::shared_ptr<camera::CameraImage> image) {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    last_preview_image_ = image;
}
#endif

// Logic Helpers
float MeterReaderTFLite::combine_readings(const std::vector<float>& readings) {
    std::string s;
    for (float r : readings) s += std::to_string((int)round(r) % 10);
    return std::stof(s);
}

bool MeterReaderTFLite::validate_and_update_reading(float raw, float conf, float& val) {
    int ival = (int)raw;
    int oval = ival;
    bool valid = output_validator_.validate_reading(ival, conf, oval);
    val = (float)oval;
    return valid;
}

// Window Control
void MeterReaderTFLite::set_camera_window_offset_x(int x) { camera_coord_.set_window_config(x, -1, -1, -1); }
void MeterReaderTFLite::set_camera_window_offset_y(int y) { camera_coord_.set_window_config(-1, y, -1, -1); }
void MeterReaderTFLite::set_camera_window_width(int w) { camera_coord_.set_window_config(-1, -1, w, -1); }
void MeterReaderTFLite::set_camera_window_height(int h) { camera_coord_.set_window_config(-1, -1, -1, h); }
void MeterReaderTFLite::set_camera_window_configured(bool c) { 
    if (c) {
        // Trigger window application logic now that configuration is complete.
        if (camera_coord_.apply_window()) {
             ESP_LOGD(TAG, "Window configuration applied successfully.");
        } else {
             ESP_LOGW(TAG, "Failed to apply window configuration.");
        }
    }
}

bool MeterReaderTFLite::reset_camera_window() {
    return camera_coord_.reset_window();
}

bool MeterReaderTFLite::set_camera_window(int offset_x, int offset_y, int width, int height) {
    return camera_coord_.set_window(offset_x, offset_y, width, height);
}

// Destructor
MeterReaderTFLite::~MeterReaderTFLite() {} 

void MeterReaderTFLite::force_flash_inference() {
    flashlight_coord_.force_inference([this](){ frame_requested_ = true; });
}

// Debug Handlers
#ifdef DEBUG_METER_READER_TFLITE
void MeterReaderTFLite::set_debug_image(const uint8_t* data, size_t size) {
    // Hardcoded dimensions for now or need to know context
    debug_coord_.set_debug_image(data, size, camera_coord_.get_width(), camera_coord_.get_height());
}
void MeterReaderTFLite::test_with_debug_image() {
    debug_coord_.run_debug_tests(tflite_coord_);
}
void MeterReaderTFLite::set_debug_mode(bool m) {
    debug_coord_.set_debug_mode(m);
}
void MeterReaderTFLite::debug_test_with_pattern() {
    debug_coord_.test_with_pattern(tflite_coord_);
}
#endif

void MeterReaderTFLite::print_debug_info() {
    debug_coord_.print_info(tflite_coord_, camera_coord_.get_width(), camera_coord_.get_height(), camera_coord_.get_format());
}

} // namespace meter_reader_tflite
} // namespace esphome
