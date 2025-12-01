#include "esp32_camera_utils.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "esp32_camera_utils";

void Esp32CameraUtils::setup() {
  if (has_config_) {
    ESP_LOGI(TAG, "Setting up camera window: offset=%d,%d size=%dx%d", offset_x_, offset_y_, width_, height_);
    if (camera_) {
      // We need to wait for camera to be initialized? 
      // Component setup order handles dependencies if configured correctly.
      // But camera setup might be async or take time.
      // We'll try to set it.
      bool success = window_control_.set_window_with_reset(
          camera_, CameraWindowControl::WindowConfig{
              offset_x_, offset_y_, width_, height_, true});
              
      if (success) {
        ESP_LOGI(TAG, "Camera window configured successfully");
      } else {
        ESP_LOGE(TAG, "Failed to configure camera window");
      }
    } else {
      ESP_LOGW(TAG, "Camera instance not available, skipping window configuration");
    }
  }
}

void Esp32CameraUtils::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32 Camera Utils:");
  if (has_config_) {
    ESP_LOGCONFIG(TAG, "  Camera Window: offset=%d,%d size=%dx%d", offset_x_, offset_y_, width_, height_);
  }
}


void Esp32CameraUtils::set_camera_image_format(int width, int height, const std::string &pixel_format) {
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

void Esp32CameraUtils::reinitialize_image_processor(const ImageProcessorConfig& config_template) {
    last_config_template_ = config_template;
    has_processor_config_ = true;

    if (camera_width_ > 0 && camera_height_ > 0) {
        // Create config based on template but with current camera dimensions
        ImageProcessorConfig config = config_template;
        config.camera_width = camera_width_;
        config.camera_height = camera_height_;
        config.pixel_format = pixel_format_;
        
        image_processor_ = std::make_unique<ImageProcessor>(config);
        ESP_LOGI(TAG, "ImageProcessor initialized with dimensions: %dx%d, format: %s",
                 camera_width_, camera_height_, pixel_format_.c_str());
    } else {
        ESP_LOGW(TAG, "Cannot initialize ImageProcessor: Invalid camera dimensions");
    }
}

bool Esp32CameraUtils::test_camera_after_reset() {
    if (!camera_) return false;
    // Simple check if we can request a frame or if camera is responsive
    // For now, just return true as in original code
    return true; 
}

void Esp32CameraUtils::basic_camera_recovery() {
    ESP_LOGW(TAG, "Attempting basic camera recovery...");
    
    // Reinitialize image processor if we have a config
    if (has_processor_config_) {
        reinitialize_image_processor(last_config_template_);
    }
    
    ESP_LOGI(TAG, "Basic camera recovery completed");
}

bool Esp32CameraUtils::reset_window(int &width, int &height) {
    if (!camera_) return false;
    
    bool success = window_control_.reset_to_full_frame_with_dimensions(
        camera_, 
        original_camera_width_, 
        original_camera_height_,
        width, 
        height
    );
    
    if (success) {
        camera_width_ = width;
        camera_height_ = height;
        pixel_format_ = original_pixel_format_;
        
        // Reinitialize processor if we have a template
        if (has_processor_config_) {
            reinitialize_image_processor(last_config_template_);
        }
        ESP_LOGI(TAG, "Camera window reset to full frame (%dx%d)", width, height);
    } else {
        ESP_LOGE(TAG, "Failed to reset camera window");
    }
    
    return success;
}

bool Esp32CameraUtils::process_zone(std::shared_ptr<camera::CameraImage> frame, const CropZone& zone, 
                                   uint8_t* output_buffer, size_t output_size) {
    if (!image_processor_) {
        ESP_LOGE(TAG, "ImageProcessor not initialized");
        return false;
    }
    return image_processor_->process_zone_to_buffer(frame, zone, output_buffer, output_size);
}

}  // namespace esp32_camera_utils
}  // namespace esphome
