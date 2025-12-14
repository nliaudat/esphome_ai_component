#include "meter_reader_tflite.h"
#include "esphome/core/application.h"

#include <esp_heap_caps.h>
#include <numeric>

#ifdef USE_WEB_SERVER
#include "esphome/components/esp32_camera_utils/preview_web_handler.h"
#endif

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "meter_reader_tflite";

#ifdef DEBUG_METER_READER_TFLITE
#define DURATION_START(name) uint32_t start_time = millis();
#define DURATION_END(name) ESP_LOGD(TAG, "%s took %u ms", name, millis() - start_time)
#else
#define DURATION_START(name)
#define DURATION_END(name)
#endif

void MeterReaderTFLite::setup() {
    ESP_LOGI(TAG, "Setting up Meter Reader TFLite (Refactored)...");
    
    // 1. Initial Config
    if (camera_coord_.get_width() == 0) {
        ESP_LOGE(TAG, "Camera dimensions not set!");
        mark_failed(); return;
    }
    
    // 2. Load Model
    // Load model first to ensure we can retrieve input specifications for camera configuration.
        
    // 2. Load Model
    // Load model first to ensure we can retrieve input specifications for camera configuration.
    ESP_LOGI(TAG, "Model loading will begin in 5 seconds...");
    // Reduced timeout to 5s from 30s to speed up boot availability
    this->set_timeout(5000, [this]() {
         ESP_LOGI(TAG, "Starting model loading...");
         if (!tflite_coord_.load_model()) {
             mark_failed(); return;
         }
         
         // After model loaded, we have input specs. Update CameraCoord.
         auto spec = tflite_coord_.get_model_spec();
         
         // Map input type: 
         // spec.input_type: 1=Float, 0=Uint8 (from TFLiteCoord)
         // camera/image_processor: 0=Float, 1=Uint8 (ImageProcessorInputType)
         int processor_input_type = (spec.input_type == 1) ? 0 : 1;
         
         camera_coord_.update_image_processor_config(
             spec.input_width, 
             spec.input_height, 
             spec.input_channels,
             processor_input_type,
             spec.normalize,
             spec.input_order
         );
         
         // Sync Esp32CameraUtils if present (for sensors)
         if (esp32_camera_utils_) {
              // Ensure it knows current dimensions
              esp32_camera_utils_->set_camera_image_format(
                  camera_coord_.get_width(),
                  camera_coord_.get_height(),
                  camera_coord_.get_format()
              );
              
              esp32_camera_utils::ImageProcessorConfig config;
              config.camera_width = camera_coord_.get_width();
              config.camera_height = camera_coord_.get_height();
              config.pixel_format = camera_coord_.get_format();
              config.model_width = spec.input_width;
              config.model_height = spec.input_height;
              config.model_channels = spec.input_channels;
              
              switch((int)rotation_) {
                  case 90:  config.rotation = esp32_camera_utils::ROTATION_90;  break;
                  case 180: config.rotation = esp32_camera_utils::ROTATION_180; break;
                  case 270: config.rotation = esp32_camera_utils::ROTATION_270; break;
                  default:  config.rotation = esp32_camera_utils::ROTATION_0;   break;
              }
              
              config.input_type = (esp32_camera_utils::ImageProcessorInputType)processor_input_type;
              config.normalize = spec.normalize;
              config.input_order = spec.input_order;
              
              esp32_camera_utils_->reinitialize_image_processor(config);
         }
         
          // Setup Web Server Preview
          #ifdef USE_WEB_SERVER
          #ifdef DEV_ENABLE_ROTATION
          if (web_server_) {
              web_server_->add_handler(new esphome::esp32_camera_utils::PreviewWebHandler([this]() {
                  return this->get_preview_image();
              }));
          }
          if (web_server_) {
              web_server_->add_handler(new esphome::esp32_camera_utils::PreviewWebHandler([this]() {
                  return this->get_preview_image();
              }));
          }
          #endif
          #endif

          // Print debug info on success (legacy behavior)
          #ifdef DEBUG_METER_READER_TFLITE
          this->print_debug_info();
          #endif
          
          // Publish static memory stats
          #ifdef DEBUG_METER_READER_MEMORY
          if (tensor_arena_size_sensor_) {
              // We need a getter for requested size in TFLiteCoord
              // Assuming get_tensor_arena_size_requested() exists or we add it
              tensor_arena_size_sensor_->publish_state(tflite_coord_.get_tensor_arena_size()); 
          }
          #endif
     });
    
    // 3. Setup Validation
    ValueValidator::ValidationConfig val_conf;
    val_conf.allow_negative_rates = allow_negative_rates_;
    val_conf.max_absolute_diff = max_absolute_diff_;
    output_validator_.set_config(val_conf);
    output_validator_.setup();
    
    ESP_LOGI(TAG, "Output validation configured - AllowNegativeRates: %s, MaxAbsoluteDiff: %d",
             val_conf.allow_negative_rates ? "YES" : "NO", 
             val_conf.max_absolute_diff);
    
    // 4. Setup Camera Callback
    // Register callback on the global camera instance.
    
    // 5. Setup Flashlight Coordinator Callback
    flashlight_coord_.set_request_frame_callback([this](){
        this->frame_requested_ = true;
        this->last_request_time_ = millis();
        ESP_LOGD(TAG, "Frame requested via coordinator callback");
    });
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
    ESP_LOGD(TAG, "Update triggered (Interval cycle)");
    if (crop_zone_handler_.has_global_zones_changed()) {
        crop_zone_handler_.apply_global_zones();
    }
    // Trigger updates for external camera utils sensors if available
    if (esp32_camera_utils_) {
        esp32_camera_utils_->update_memory_sensors();
    }
    
    // 2. Crop Zones Processing
    if (pause_processing_) {
        ESP_LOGD(TAG, "Processing paused, skipping update");
        return;
    }
    
    // The flashlight coordinator returns true if it is handling the cycle (scheduling or waiting)
    bool busy = flashlight_coord_.update_scheduling();
    
    if (!busy) {
        // Normal cycle
         if (!frame_available_ && !frame_requested_) {
             frame_requested_ = true;
             last_request_time_ = millis();
             ESP_LOGD(TAG, "Requesting frame (Continuous/No-Flash)");
         } else if (frame_available_) {
             process_available_frame();
         }
    }
}

void MeterReaderTFLite::loop() {
    // Watchdog: If frame requested but not arrived for 15s, reset state
    if (frame_requested_ && (millis() - last_request_time_ > 15000)) {
        ESP_LOGW(TAG, "Frame request timed out (15s)! Resetting state.");
        frame_requested_ = false;
        // Check if we need to force reset camera or just continue
    }

    if (frame_available_ && !processing_frame_) {
        process_available_frame();
    }
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

    DURATION_START("Total Processing");

    if (!tflite_coord_.is_model_loaded()) {
        ESP_LOGW(TAG, "Skipping frame - Model not loaded yet");
        return;
    }

    
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
    ESP_LOGI(TAG, "Processing Image: Found %d crop zones", zones.size());
    
    // Process frame -> buffers
    auto processed_buffers = camera_coord_.process_frame(frame, zones);
    
    // Buffers -> Inference
    
    // Capture Peak Memory State *during* processing (buffers allocated)
    #ifdef DEBUG_METER_READER_MEMORY
    if (process_free_heap_sensor_) process_free_heap_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    if (process_free_psram_sensor_) process_free_psram_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    #endif

    auto results = tflite_coord_.run_inference(processed_buffers);
    
    #ifdef DEBUG_METER_READER_MEMORY
    if (tensor_arena_used_sensor_) {
          tensor_arena_used_sensor_->publish_state(tflite_coord_.get_arena_used_bytes());
    }
    #endif

    
    // Collect readings
    std::vector<float> readings, confidences;
    int digit_index = 0;
    for (const auto& res : results) {
        if (res.success) {
            ESP_LOGD(TAG, "Digit %d: %.0f (%.2f)", digit_index, res.value, res.confidence);
            readings.push_back(res.value);
            confidences.push_back(res.confidence);
        } else {
             ESP_LOGW(TAG, "Digit %d: Failed to infer", digit_index);
        }
        digit_index++;
    }
    
    if (!readings.empty()) {
        // Use helper to combine readings and log details (matches legacy behavior)
        float final_val = combine_readings(readings);

        float avg_conf = std::accumulate(confidences.begin(), confidences.end(), 0.0f) / confidences.size();
        
        float validated_val = final_val;
        bool valid = validate_and_update_reading(final_val, avg_conf, validated_val);

        if (inference_logs_) {
             // Publish to inference logs text sensor
             char inference_log[150];
             snprintf(inference_log, sizeof(inference_log),
                      "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)",
                      final_val, validated_val, valid ? "yes" : "no",
                      avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             inference_logs_->publish_state(inference_log);
        }

        if (valid && avg_conf >= confidence_threshold_) {
             // Removed checking of inference_log char buffer availability to match legacy cleanly
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             
             if (value_sensor_) value_sensor_->publish_state(validated_val);
             if (confidence_sensor_) confidence_sensor_->publish_state(avg_conf * 100.0f);
             
             ESP_LOGI(TAG, "Reading published - valid and confidence threshold met");
        } else {
             ESP_LOGI(TAG, "Reading: %.1f -> %.1f (valid: %s, confidence: %.1f%%, threshold: %.1f%%)", 
                final_val, validated_val, valid ? "yes" : "no", 
                avg_conf * 100.0f, confidence_threshold_ * 100.0f);
             ESP_LOGW(TAG, "Reading NOT published - %s", 
                     !valid ? "validation failed" : "confidence below threshold");
        }
    }
    
    DURATION_END("Total Processing");
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
        last_request_time_ = millis();
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
    
    float combined_value = 0.0f;
    // Guaranteed to be numeric string from logic above
    combined_value = std::stof(digit_string);
    
    ESP_LOGI(TAG, "Final combined value: %.0f", combined_value);
    return combined_value;
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
    flashlight_coord_.force_inference([this](){ 
        frame_requested_ = true; 
        last_request_time_ = millis();
    });
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
