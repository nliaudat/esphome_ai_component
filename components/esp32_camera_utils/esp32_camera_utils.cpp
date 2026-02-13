#include "esp32_camera_utils.h"
#include "esphome/core/log.h"
#include <esp_heap_caps.h>
#include "debug_utils.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "esp32_camera_utils";

void Esp32CameraUtils::setup() {
  if (this->has_config_) {
    ESP_LOGI(TAG, "Setting up camera window: offset=%d,%d size=%dx%d", this->offset_x_, this->offset_y_, this->width_, this->height_);
    if (this->camera_) {
      // Attempt to set the window. If camera is not ready, this might fail or be skipped.
      // Component setup priority should normally ensure camera is initialized first.
      bool success = this->window_control_.set_window_with_reset(
          this->camera_, CameraWindowControl::WindowConfig{
              this->offset_x_, this->offset_y_, this->width_, this->height_, true});
              
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
  if (this->has_config_) {
    ESP_LOGCONFIG(TAG, "  Camera Window: offset=%d,%d size=%dx%d", this->offset_x_, this->offset_y_, this->width_, this->height_);
  }
}

bool Esp32CameraUtils::set_camera_window(int offset_x, int offset_y, int width, int height) {
    DURATION_START();
    this->set_camera_window_config(offset_x, offset_y, width, height);
    bool success = this->window_control_.set_window(this->camera_, offset_x, offset_y, width, height);
    
    if (success) {
        this->camera_width_ = width;
        this->camera_height_ = height;
        
        if (this->has_processor_config_) {
            this->reinitialize_image_processor(this->last_config_template_);
        }
        if (this->debug_) {
            ESP_LOGD(TAG, "Camera window set to %dx%d. ImageProcessor updated.", width, height);
        } else {
            ESP_LOGI(TAG, "Camera window set to %dx%d. ImageProcessor updated.", width, height);
        }
    }
    DURATION_END("set_camera_window");
    return success;
}


void Esp32CameraUtils::set_camera_image_format(int width, int height, const std::string &pixel_format) {
  this->camera_width_ = width;
  this->camera_height_ = height;
  this->pixel_format_ = pixel_format;
  
  // Store as original dimensions if not already set
  if (this->original_camera_width_ == 0) {
    this->original_camera_width_ = width;
  }
  if (this->original_camera_height_ == 0) {
    this->original_camera_height_ = height;
  }
  if (this->original_pixel_format_.empty()) {
    this->original_pixel_format_ = pixel_format;
  }
  
  ESP_LOGD(TAG, "Camera format set: %dx%d, %s (original: %dx%d, %s)", 
           width, height, pixel_format.c_str(),
           this->original_camera_width_, this->original_camera_height_, this->original_pixel_format_.c_str());
}

void Esp32CameraUtils::reinitialize_image_processor(const ImageProcessorConfig& config_template) {
    DURATION_START();
    this->last_config_template_ = config_template;
    this->has_processor_config_ = true;

    if (this->camera_width_ > 0 && this->camera_height_ > 0) {
        // Create config based on template but with current camera dimensions
        ImageProcessorConfig config = config_template;
        config.camera_width = this->camera_width_;
        config.camera_height = this->camera_height_;
        config.pixel_format = this->pixel_format_;
        
        // Apply modular configs if present
        if (this->has_scaler_config_) {
            config.scaler_width = this->scaler_width_;
            config.scaler_height = this->scaler_height_;
        }
        if (this->has_cropper_config_) {
            config.cropper_width = this->cropper_width_;
            config.cropper_height = this->cropper_height_;
            config.cropper_offset_x = this->cropper_offset_x_;
            config.cropper_offset_y = this->cropper_offset_y_;
        }
        
        this->image_processor_ = std::make_unique<ImageProcessor>(config);
        ESP_LOGI(TAG, "ImageProcessor initialized with dimensions: %dx%d, format: %s",
                 this->camera_width_, this->camera_height_, this->pixel_format_.c_str());

        #ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
        if (camera_buffer_size_sensor_ && image_processor_) {
            camera_buffer_size_sensor_->publish_state(image_processor_->get_required_buffer_size());
        }
        #endif

    } else {
        ESP_LOGW(TAG, "Cannot initialize ImageProcessor: Invalid camera dimensions");
    }
    DURATION_END("reinitialize_image_processor");
}

bool Esp32CameraUtils::test_camera_after_reset() {
    if (!this->camera_) return false;
    // Simple check if we can request a frame or if camera is responsive
    // For now, just return true as in original code
    return true; 
}

void Esp32CameraUtils::basic_camera_recovery() {
    ESP_LOGW(TAG, "Attempting basic camera recovery...");
    
    // Reinitialize image processor if we have a config
    if (this->has_processor_config_) {
        this->reinitialize_image_processor(this->last_config_template_);
    }
    
    ESP_LOGI(TAG, "Basic camera recovery completed");
}

bool Esp32CameraUtils::reset_window(int &width, int &height) {
    DURATION_START();
    if (!this->camera_) return false;
    
    bool success = this->window_control_.reset_to_full_frame_with_dimensions(
        this->camera_, 
        this->original_camera_width_, 
        this->original_camera_height_,
        width, 
        height
    );
    
    if (success) {
        this->camera_width_ = width;
        this->camera_height_ = height;
        this->pixel_format_ = this->original_pixel_format_;
        
        // Reinitialize processor if we have a template
        if (this->has_processor_config_) {
            this->reinitialize_image_processor(this->last_config_template_);
        }
        ESP_LOGI(TAG, "Camera window reset to full frame (%dx%d)", width, height);
    } else {
        ESP_LOGE(TAG, "Failed to reset camera window");
    }
    
    DURATION_END("reset_window");
    return success;
}

bool Esp32CameraUtils::process_zone(std::shared_ptr<camera::CameraImage> frame, const CropZone& zone, 
                                   uint8_t* output_buffer, size_t output_size) {
    if (!this->image_processor_) {
        ESP_LOGE(TAG, "ImageProcessor not initialized");
        return false;
    }

    #ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
    // We update sensors here too as this is the "native" path, but also allow manual updates
    if (this->camera_free_psram_sensor_) {
        this->camera_free_psram_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    #endif


    // Translate global crop zone to local window coordinates
    CropZone local_zone = zone;
    
    // If we have a window offset, adjust the coordinates
    if (this->has_config_) {
        local_zone.x1 -= this->offset_x_;
        local_zone.y1 -= this->offset_y_;
        local_zone.x2 -= this->offset_x_;
        local_zone.y2 -= this->offset_y_;
        
        // Clip to actual image dimensions to prevent out-of-bounds errors
        // This handles cases where the crop zone might slightly overlap the window edge
        // or if the window configuration is slightly out of sync.
        int img_width = this->camera_width_;
        int img_height = this->camera_height_;
        
        // Use actual frame dimensions if available and different (e.g. JPEG)
        // Note: CameraImage doesn't expose dimensions directly, so we rely on camera_width_
        // which should match the window width if configured correctly.
        // If we needed actual JPEG dims, we'd need to parse the header here or let ImageProcessor handle it.
        // For now, we assume camera_width_ is correct (window width).

        // Basic clipping
        local_zone.x1 = std::max(0, local_zone.x1);
        local_zone.y1 = std::max(0, local_zone.y1);
        local_zone.x2 = std::min(img_width, local_zone.x2);
        local_zone.y2 = std::min(img_height, local_zone.y2);
        
        // Check if the zone is valid after clipping
        if (local_zone.x2 <= local_zone.x1 || local_zone.y2 <= local_zone.y1) {
            ESP_LOGW(TAG, "Crop zone outside of current camera window (Global: %d,%d->%d,%d | Window Offset: %d,%d | Local: %d,%d->%d,%d)", 
                     zone.x1, zone.y1, zone.x2, zone.y2, 
                     this->offset_x_, this->offset_y_,
                     local_zone.x1, local_zone.y1, local_zone.x2, local_zone.y2);
            return false;
        }
    }

    return this->image_processor_->process_zone_to_buffer(frame, local_zone, output_buffer, output_size);
}

void Esp32CameraUtils::update_memory_sensors() {
    #ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
    if (this->camera_free_psram_sensor_) {
        this->camera_free_psram_sensor_->publish_state(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    if (this->camera_buffer_size_sensor_ && this->image_processor_) {
        this->camera_buffer_size_sensor_->publish_state(this->image_processor_->get_required_buffer_size());
    }
    #endif
}

}  // namespace esp32_camera_utils
}  // namespace esphome
