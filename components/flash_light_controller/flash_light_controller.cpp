#include "flash_light_controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace flash_light_controller {

static const char *const TAG = "flash_light_controller";

// Static-lifetime timeout names so cancel_active_sequence() can cancel them.
static constexpr const char *TIMEOUT_CAPTURE = "flash_capture";
static constexpr const char *TIMEOUT_OFF = "flash_off";

void FlashLightController::setup() {
  this->is_active_ = false;
  ESP_LOGCONFIG(TAG, "Setting up Flash Light Controller...");
  if (this->flash_light_) {
    ESP_LOGCONFIG(TAG, "  Flash Light: Configured");
    ESP_LOGCONFIG(TAG, "  Pre-Time: %u ms", this->flash_pre_time_);
    ESP_LOGCONFIG(TAG, "  Post-Time: %u ms", this->flash_post_time_);
  } else {
    ESP_LOGCONFIG(TAG, "  Flash Light: Not configured");
  }
}

void FlashLightController::initiate_capture_sequence(CaptureCallback callback) {
  if (!this->flash_light_) {
    // Config problem: always warn, regardless of debug mode.
    ESP_LOGW(TAG, "No flash light configured, executing callback immediately");
    if (this->debug_) {
      ESP_LOGD(TAG, "  (flash skipped: null flash_light_ pointer)");
    }
    if (callback)
      callback();
    return;
  }

  if (this->is_active_) {
    if (this->captured_) {
      // Capture point already passed, the flash stays on for the post time: capture right away
      if (this->debug_) {
        ESP_LOGD(TAG, "Flash sequence active (post time), executing joined capture callback now");
      }
      if (callback)
        callback();
    } else {
      if (this->debug_) {
        ESP_LOGD(TAG, "Flash sequence already active, joining it");
      }
      this->pending_callbacks_.push_back(callback);
    }
    return;
  }

  this->is_active_ = true;
  this->captured_ = false;
  this->pending_callbacks_.clear();
  this->pending_callbacks_.push_back(callback);
  if (this->debug_) {
    ESP_LOGD(TAG, "Starting flash sequence (Pre: %u ms, Post: %u ms)", this->flash_pre_time_, this->flash_post_time_);
  }

  this->enable_flash();

  // Schedule callback after pre-time (named so the sequence can be cancelled)
  this->set_timeout(TIMEOUT_CAPTURE, this->flash_pre_time_, [this]() {
    if (this->debug_) {
      ESP_LOGD(TAG, "Flash warmup complete (%u ms elapsed), executing %u capture callback(s)", this->flash_pre_time_,
               static_cast<unsigned>(this->pending_callbacks_.size()));
    }
    this->captured_ = true;
    // Moved out first: a callback may start or join a sequence and change the list
    auto callbacks = std::move(this->pending_callbacks_);
    this->pending_callbacks_.clear();
    for (auto &cb : callbacks) {
      if (cb)
        cb();
    }

    // Schedule flash off after post-time (relative to now)
    this->set_timeout(TIMEOUT_OFF, this->flash_post_time_, [this]() {
      this->disable_flash();
      this->is_active_ = false;
      if (this->debug_) {
        ESP_LOGD(TAG, "Flash sequence complete (Post-time %u ms finished)", this->flash_post_time_);
      }
    });
  });
}

void FlashLightController::cancel_active_sequence() {
  if (!this->is_active_) {
    if (this->debug_) {
      ESP_LOGD(TAG, "cancel_active_sequence() called while idle, no-op");
    }
    return;
  }
  if (this->debug_) {
    ESP_LOGD(TAG, "Cancelling active flash sequence");
  }
  this->cancel_timeout(TIMEOUT_CAPTURE);
  this->cancel_timeout(TIMEOUT_OFF);
  this->pending_callbacks_.clear();
  this->captured_ = false;
  this->disable_flash();
  this->is_active_ = false;
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
