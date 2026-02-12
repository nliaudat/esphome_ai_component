#include "flash_light_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace flash_light_controller {

static const char *const TAG = "flash_light_controller";

void FlashLightController::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Flash Light Controller...");
    if (flash_light_) {
        ESP_LOGCONFIG(TAG, "  Flash Light: Configured");
        ESP_LOGCONFIG(TAG, "  Pre-Time: %u ms", flash_pre_time_);
        ESP_LOGCONFIG(TAG, "  Post-Time: %u ms", flash_post_time_);
    } else {
        ESP_LOGCONFIG(TAG, "  Flash Light: Not configured");
    }
}

void FlashLightController::set_flash_light(light::LightState *flash_light) {
    flash_light_ = flash_light;
}

void FlashLightController::initiate_capture_sequence(CaptureCallback callback) {
    if (!flash_light_) {
        ESP_LOGW(TAG, "No flash light configured, executing callback immediately");
        if (callback) callback();
        return;
    }

    if (is_active_) {
        ESP_LOGW(TAG, "Flash sequence already active, ignoring request");
        return;
    }

    is_active_ = true;
    if (debug_) {
        ESP_LOGD(TAG, "Starting flash sequence (Pre: %d ms, Post: %d ms)", flash_pre_time_, flash_post_time_);
    } else {
        ESP_LOGD(TAG, "Starting flash sequence");
    }

    this->enable_flash();

    // Schedule callback after pre-time
    this->set_timeout(flash_pre_time_, [this, callback]() {
        if (debug_) {
            ESP_LOGD(TAG, "Flash warmup complete (%d ms elapsed), executing capture callback", flash_pre_time_);
        } else {
            ESP_LOGD(TAG, "Flash warmup complete, executing capture callback");
        }
        if (callback) callback();

        // Schedule flash off after post-time (relative to now)
        this->set_timeout(flash_post_time_, [this]() {
            this->disable_flash();
            is_active_ = false;
            if (debug_) {
                ESP_LOGD(TAG, "Flash sequence complete (Post-time %d ms finished)", flash_post_time_);
            } else {
                ESP_LOGD(TAG, "Flash sequence complete");
            }
        });
    });
}

void FlashLightController::enable_flash() {
    if (flash_light_) {
        auto call = flash_light_->turn_on();
        call.set_transition_length(0);
        call.perform();
    }
}

void FlashLightController::disable_flash() {
    if (flash_light_) {
        auto call = flash_light_->turn_off();
        call.set_transition_length(0);
        call.perform();
    }
}

}  // namespace flash_light_controller
}  // namespace esphome
