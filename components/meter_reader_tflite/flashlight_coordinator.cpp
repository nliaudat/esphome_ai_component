#include "flashlight_coordinator.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "flashlight_coordinator";

void FlashlightCoordinator::setup(Component* parent, light::LightState* legacy_light, 
                                  flash_light_controller::FlashLightController* controller) {
    parent_ = parent;
    legacy_light_ = legacy_light;
    controller_ = controller;
}

void FlashlightCoordinator::set_timing(uint32_t pre_time, uint32_t post_time) {
    pre_time_ = pre_time;
    post_time_ = post_time;
}

void FlashlightCoordinator::set_update_interval(uint32_t interval_ms) {
    update_interval_ = interval_ms;
}

void FlashlightCoordinator::enable_flash() {
    if (controller_) {
        controller_->enable_flash();
    } else if (legacy_light_) {
        auto_controlled_.store(true);
        auto call = legacy_light_->turn_on();
        call.set_transition_length(0);
        call.perform();
    }
}

void FlashlightCoordinator::disable_flash() {
    if (controller_) {
        controller_->disable_flash();
    } else if (legacy_light_ && auto_controlled_.load()) {
        auto_controlled_.store(false);
        auto call = legacy_light_->turn_off();
        call.set_transition_length(0);
        call.perform();
    }
}

bool FlashlightCoordinator::update_scheduling() {
    // If controller is present, it handles everything
    if (controller_) {
        if (!controller_->is_active()) {
             // Not running: Start it
             ESP_LOGD(TAG, "Controller idle, initiating sequence");
             controller_->initiate_capture_sequence([this]() {
                 if (request_frame_callback_) request_frame_callback_();
             });
             return true; // We initiated, so we are "busy" in a sense, or rather we handled it.
        } else {
             // Already running
             ESP_LOGD(TAG, "Controller already active, skipping trigger");
             return true;
        }
    }
    
    // Legacy Logic
    if (legacy_light_ && !scheduled_) {
        uint32_t schedule_time = (update_interval_ > pre_time_) ? update_interval_ - pre_time_ : 0;
        
        if (schedule_time > 0 && parent_) {
            ESP_LOGI(TAG, "Scheduling flash for next cycle in %u ms", schedule_time);
            
            App.scheduler.set_timeout(parent_, "flash_on", schedule_time, [this]() {
                enable_flash();
                scheduled_ = true;
                
                // Then wait for pre-time to capture
                uint32_t capture_delay = (pre_time_ > 500) ? pre_time_ - 500 : 0;
                
                App.scheduler.set_timeout(parent_, "flash_capture", capture_delay, [this]() {
                    if (request_frame_callback_) request_frame_callback_();
                });
                
                // Then off
                uint32_t off_delay = pre_time_ + post_time_;
                App.scheduler.set_timeout(parent_, "flash_off", off_delay, [this]() {
                    disable_flash();
                    scheduled_ = false;
                });
            });
            return true; // Scheduled
        }
    }
    
    return false; // Not handled or no flash
}

void FlashlightCoordinator::force_inference(std::function<void()> frame_request_callback) {
    if (!legacy_light_ && !controller_) return;
    
    ESP_LOGI(TAG, "Forcing flash inference");
    enable_flash();
    
    if (parent_) {
        // Warmup 3s
        App.scheduler.set_timeout(parent_, "force_flash_warmup", 3000, [this, frame_request_callback]() {
             frame_request_callback();
             
             // Off after 500ms safety
             App.scheduler.set_timeout(parent_, "force_flash_off", 500, [this]() {
                 disable_flash();
             });
        });
    }
}

void FlashlightCoordinator::capture_preview_sequence(std::function<void()> frame_request_callback) {
    enable_flash();
    
    uint32_t warmup = controller_ ? controller_->get_flash_pre_time() : pre_time_;
    if (warmup < 1000) warmup = 1000;
    
    if (parent_) {
        App.scheduler.set_timeout(parent_, "preview_warmup", warmup, [this, frame_request_callback]() {
            frame_request_callback();
            
            uint32_t post = controller_ ? controller_->get_flash_post_time() : post_time_;
            App.scheduler.set_timeout(parent_, "preview_off", post, [this]() {
                disable_flash(); 
            });
        });
    }
}

} // namespace meter_reader_tflite
} // namespace esphome
