#include "flashlight_coordinator.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace ssocr_reader {

static const char *const TAG = "flashlight_coordinator";

void FlashlightCoordinator::setup(Component* parent, light::LightState* legacy_light, 
                                  flash_light_controller::FlashLightController* controller) {
    this->parent_ = parent;
    this->legacy_light_ = legacy_light;
    this->controller_ = controller;
}

void FlashlightCoordinator::set_timing(uint32_t pre_time, uint32_t post_time) {
    this->pre_time_ = pre_time;
    this->post_time_ = post_time;
}

void FlashlightCoordinator::set_update_interval(uint32_t interval_ms) {
    this->update_interval_ = interval_ms;
}

void FlashlightCoordinator::enable_flash() {
    if (this->controller_) {
        this->controller_->enable_flash();
    } else if (this->legacy_light_) {
        this->auto_controlled_.store(true);
        auto call = this->legacy_light_->turn_on();
        call.set_transition_length(0);
        call.perform();
    }
}

void FlashlightCoordinator::disable_flash() {
    if (this->controller_) {
        this->controller_->disable_flash();
    } else if (this->legacy_light_ && this->auto_controlled_.load()) {
        this->auto_controlled_.store(false);
        auto call = this->legacy_light_->turn_off();
        call.set_transition_length(0);
        call.perform();
    }
}

bool FlashlightCoordinator::update_scheduling() {
    // If controller is present, it handles everything
    if (this->controller_) {
        if (!this->controller_->is_active()) {
             // Not running: Start it
             ESP_LOGD(TAG, "Controller idle, initiating sequence");
             this->controller_->initiate_capture_sequence([this]() {
                 if (this->request_frame_callback_) this->request_frame_callback_();
             });
             return true; // We initiated, so we are "busy" in a sense, or rather we handled it.
        } else {
             // Already running
             ESP_LOGD(TAG, "Controller already active, skipping trigger");
             return true;
        }
    }
    
    // Legacy Logic
    if (this->legacy_light_ && !this->scheduled_) {
        uint32_t schedule_time = (this->update_interval_ > this->pre_time_) ? this->update_interval_ - this->pre_time_ : 0;
        
        if (schedule_time > 0 && this->parent_) {
            ESP_LOGI(TAG, "Scheduling flash for next cycle in %u ms", schedule_time);
            
            App.scheduler.set_timeout(this->parent_, "flash_on", schedule_time, [this]() {
                this->enable_flash();
                this->scheduled_ = true;
                
                // Then wait for pre-time to capture
                uint32_t capture_delay = (this->pre_time_ > 500) ? this->pre_time_ - 500 : 0;
                
                App.scheduler.set_timeout(this->parent_, "flash_capture", capture_delay, [this]() {
                    if (this->request_frame_callback_) this->request_frame_callback_();
                });
                
                // Then off
                uint32_t off_delay = this->pre_time_ + this->post_time_;
                App.scheduler.set_timeout(this->parent_, "flash_off", off_delay, [this]() {
                    this->disable_flash();
                    this->scheduled_ = false;
                });
            });
            return true; // Scheduled
        }
    }
    
    return false; // Not handled or no flash
}

void FlashlightCoordinator::force_inference(std::function<void()> frame_request_callback) {
    if (!this->legacy_light_ && !this->controller_) return;
    
    ESP_LOGI(TAG, "Forcing flash inference");
    this->enable_flash();
    
    if (this->parent_) {
        // Warmup 3s
        App.scheduler.set_timeout(this->parent_, "force_flash_warmup", 3000, [this, frame_request_callback]() {
             frame_request_callback();
             
             // Off after 500ms safety
             App.scheduler.set_timeout(this->parent_, "force_flash_off", 500, [this]() {
                 this->disable_flash();
             });
        });
    }
}

void FlashlightCoordinator::capture_preview_sequence(std::function<void()> frame_request_callback) {
    this->enable_flash();
    
    uint32_t warmup = this->controller_ ? this->controller_->get_flash_pre_time() : this->pre_time_;
    if (warmup < 1000) warmup = 1000;
    
    if (this->parent_) {
        App.scheduler.set_timeout(this->parent_, "preview_warmup", warmup, [this, frame_request_callback]() {
            frame_request_callback();
            
            uint32_t post = this->controller_ ? this->controller_->get_flash_post_time() : this->post_time_;
            App.scheduler.set_timeout(this->parent_, "preview_off", post, [this]() {
                this->disable_flash(); 
            });
        });
    }
}

} // namespace ssocr_reader
} // namespace esphome
