#include "flash_light_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace flash_light_controller {

static const char *const TAG = "flash_light_controller";

void FlashLightController::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Flash Light Controller...");
    if (this->flash_light_) {
        ESP_LOGCONFIG(TAG, "  Flash Light: Configured");
        ESP_LOGCONFIG(TAG, "  Pre-Time: %u ms", this->flash_pre_time_);
        ESP_LOGCONFIG(TAG, "  Post-Time: %u ms", this->flash_post_time_);
    } else {
        ESP_LOGCONFIG(TAG, "  Flash Light: Not configured");
    }
}

void FlashLightController::set_flash_light(light::LightState *flash_light) {
    this->flash_light_ = flash_light;
}

void FlashLightController::initiate_capture_sequence(CaptureCallback callback) {
    if (!this->flash_light_) {
        ESP_LOGW(TAG, "No flash light configured, executing callback immediately");
        if (callback) callback();
        return;
    }

    if (this->is_active_) {
        ESP_LOGW(TAG, "Flash sequence already active, ignoring request");
        return;
    }

    this->is_active_ = true;
    if (this->debug_) {
        ESP_LOGD(TAG, "Starting flash sequence (Pre: %u ms, Post: %u ms)", this->flash_pre_time_, this->flash_post_time_);
    } else {
        ESP_LOGD(TAG, "Starting flash sequence");
    }

    this->enable_flash();

    // Schedule callback after pre-time
    this->set_timeout(this->flash_pre_time_, [this, callback]() {
        if (this->debug_) {
            ESP_LOGD(TAG, "Flash warmup complete (%u ms elapsed), executing capture callback", this->flash_pre_time_);
        } else {
            ESP_LOGD(TAG, "Flash warmup complete, executing capture callback");
        }
        if (callback) callback();

        // Schedule flash off after post-time (relative to now)
        this->set_timeout(this->flash_post_time_, [this]() {
            this->disable_flash();
            this->is_active_ = false;
            if (this->debug_) {
                ESP_LOGD(TAG, "Flash sequence complete (Post-time %u ms finished)", this->flash_post_time_);
            } else {
                ESP_LOGD(TAG, "Flash sequence complete");
            }
        });
    });
}

void FlashLightController::enable_flash() {
    if (this->flash_light_) {
        auto call = this->flash_light_->turn_on();
        call.set_transition_length(0);
        call.perform();
    }
}

void FlashLightController::disable_flash() {
    if (this->flash_light_) {
        auto call = this->flash_light_->turn_off();
        call.set_transition_length(0);
        call.perform();
    }
}

}  // namespace flash_light_controller
}  // namespace esphome
