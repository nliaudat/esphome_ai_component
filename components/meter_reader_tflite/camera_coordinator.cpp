#include "camera_coordinator.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h" // For delay() if needed

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "camera_coordinator";

// Camera stabilization delays (blocking - camera must be stable before returning)
static constexpr uint32_t WINDOW_SET_STABILIZATION_MS = 500;
static constexpr uint32_t WINDOW_RESET_STABILIZATION_MS = 1000;

void CameraCoordinator::set_camera(esp32_camera::ESP32Camera* camera) {
    camera_ = camera;
}

void CameraCoordinator::set_config(int width, int height, const std::string& pixel_format) {
    current_width_ = width;
    current_height_ = height;
    current_format_ = pixel_format;
    
    // Assume initial config is "original"
    if (orig_width_ == 0) {
        orig_width_ = width;
        orig_height_ = height;
        orig_format_ = pixel_format;
    }
}

bool CameraCoordinator::supports_window() const {
     if (!camera_) return false;
     return window_control_.supports_window(camera_);
}

bool CameraCoordinator::set_window(int offset_x, int offset_y, int width, int height) {
    if (!camera_) return false;
    ESP_LOGI(TAG, "Setting camera window: off(%d,%d) size(%dx%d)", 
             offset_x, offset_y, width, height);

    bool success = window_control_.set_window_with_reset(
        camera_, 
        esp32_camera_utils::CameraWindowControl::WindowConfig{
            offset_x, offset_y, width, height, true});

    if (success) {
        auto new_dims = window_control_.update_dimensions_after_window(
            camera_, 
            esp32_camera_utils::CameraWindowControl::WindowConfig{
                offset_x, offset_y, width, height, true},
            current_width_, current_height_);
        
        current_width_ = new_dims.first;
        current_height_ = new_dims.second;
        
        // Blocking delay required: camera must stabilize before returning success
        // Cannot use set_timeout() as caller expects immediate result
        delay(WINDOW_SET_STABILIZATION_MS);
        return true;
    }
    ESP_LOGE(TAG, "Failed to set camera window");
    reset_window();
    return false;
}

bool CameraCoordinator::reset_window() {
    ESP_LOGI(TAG, "Resetting camera window to full frame");
    // Hard reset preferred
    bool success = window_control_.hard_reset_camera(camera_);
    if (!success) {
         ESP_LOGW(TAG, "Hard reset failed, trying soft reset");
         success = window_control_.soft_reset_camera(camera_);
    }

    if (success) {
         success = window_control_.reset_to_full_frame_with_dimensions(
             camera_, orig_width_, orig_height_, current_width_, current_height_);
         
         if (success) {
             current_format_ = orig_format_;
             delay(WINDOW_RESET_STABILIZATION_MS); // Blocking: camera must stabilize
         }
    }
    
    if (!success) {
        ESP_LOGE(TAG, "Failed to reset camera window completely");
        basic_recovery();
    }
    
    return success;
}

void CameraCoordinator::basic_recovery() {
     ESP_LOGI(TAG, "Executing basic camera recovery (state reset)");
     // Implement any specific camera re-init calls if exposed by esp32_camera, 
     // but mostly this just logs and maybe allows the system to try again next loop.
}

bool CameraCoordinator::test_camera_after_reset(std::atomic<bool>& frame_available, std::atomic<bool>& frame_requested) {
     frame_requested.store(true);
     uint32_t start = millis();
     while (millis() - start < 5000) {
         if (frame_available.load()) {
             frame_available.store(false);
             frame_requested.store(false);
             return true;
         }
         delay(100);
     }
     frame_requested.store(false);
     return false;
}



void CameraCoordinator::update_image_processor_config(int model_width, int model_height, int model_channels, 
                                                      int input_type, bool normalize, const std::string& input_order) {
    esp32_camera_utils::ImageProcessorConfig config;
    config.camera_width = current_width_;
    config.camera_height = current_height_;
    config.pixel_format = current_format_;
    config.model_width = model_width;
    config.model_height = model_height;
    config.model_channels = model_channels;
    
    switch((int)rotation_) {
        case 90:  config.rotation = esp32_camera_utils::ROTATION_90;  break;
        case 180: config.rotation = esp32_camera_utils::ROTATION_180; break;
        case 270: config.rotation = esp32_camera_utils::ROTATION_270; break;
        default:  config.rotation = esp32_camera_utils::ROTATION_0;   break;
    }
    
    config.input_type = (esp32_camera_utils::ImageProcessorInputType)input_type;
    config.normalize = normalize;
    config.input_order = input_order;

    image_processor_ = std::make_unique<esp32_camera_utils::ImageProcessor>(config);
    ESP_LOGI(TAG, "ImageProcessor initialized in CameraCoord: %dx%d %s -> Model %dx%d", 
             current_width_, current_height_, current_format_.c_str(), config.model_width, config.model_height);
}

std::vector<CameraCoordinator::ProcessResult> CameraCoordinator::process_frame(
      std::shared_ptr<camera::CameraImage> frame,
      const std::vector<esp32_camera_utils::CropZone>& zones) {
    if (!image_processor_) {
         ESP_LOGE(TAG, "ImageProcessor not initialized");
         return {};
    }
    return image_processor_->split_image_in_zone(frame, zones);

}

bool CameraCoordinator::apply_window() {
    if (!window_configured_) {
        ESP_LOGD(TAG, "Window not configured locally, nothing to apply");
        return true; 
    }
    return set_window(window_offset_x_, window_offset_y_, window_width_, window_height_);
}

} // namespace meter_reader_tflite
} // namespace esphome
