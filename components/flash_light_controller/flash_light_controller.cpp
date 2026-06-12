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
        ESP_LOGCONFIG(TAG, "  Flash Brightness: %.0f%%", this->flash_brightness_ * 100.0f);
        ESP_LOGCONFIG(TAG, "  Idle Brightness: %.0f%%", this->idle_brightness_ * 100.0f);
        // Apply idle brightness at boot so the light is at its resting level immediately.
        this->disable_flash();
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

#ifdef CONFIG_CAMERA_AF_SUPPORT
        if (this->enable_autofocus_) {
            sensor_t *s = esp_camera_sensor_get();
            if (s && esp_camera_af_is_supported(s)) {
                if (!this->af_initialized_) {
                    esp_camera_af_config_t af_cfg = {};
                    af_cfg.mode = ESP_CAMERA_AF_MODE_MANUAL;
                    af_cfg.timeout_ms = this->af_timeout_ms_;
                    if (esp_camera_af_init(s, &af_cfg) == ESP_OK) {
                        this->af_initialized_ = true;
                        ESP_LOGD(TAG, "AF firmware initialized");
                    } else {
                        ESP_LOGW(TAG, "AF firmware init failed, skipping");
                    }
                }
                if (this->af_initialized_) {
                    ESP_LOGD(TAG, "Triggering single-shot autofocus...");
                    esp_camera_af_trigger(s);
                    esp_camera_af_status_t af_status = {};
                    esp_err_t af_err = esp_camera_af_wait(s, this->af_timeout_ms_, &af_status);
                    if (af_err == ESP_OK && af_status.focused) {
                        ESP_LOGI(TAG, "Autofocus: focused");
                    } else {
                        ESP_LOGW(TAG, "Autofocus: %s (raw=0x%02x)",
                                 af_err == ESP_ERR_TIMEOUT ? "timeout" : "not focused", af_status.raw);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Autofocus requested but sensor does not support it");
            }
        }
#endif

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
        // Flash pulse uses configurable brightness and overrides idle dim state.
        call.set_brightness(this->flash_brightness_);
        call.set_rgb(1.0f, 1.0f, 1.0f);
        call.perform();
    }
}

void FlashLightController::disable_flash() {
    if (this->flash_light_) {
        if (this->idle_brightness_ > 0.0f) {
            auto call = this->flash_light_->turn_on();
            call.set_transition_length(0);
            call.set_brightness(this->idle_brightness_);
            // Force white so RGB strips actually emit light at idle.
            call.set_rgb(1.0f, 1.0f, 1.0f);
            call.perform();
        } else {
            auto call = this->flash_light_->turn_off();
            call.set_transition_length(0);
            call.perform();
        }
    }
}

}  // namespace flash_light_controller
}  // namespace esphome
