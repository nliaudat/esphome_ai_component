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
  this->captured_ = false;
  // Two readers normally share a sequence. Reserve once so the capture path does not reallocate.
  this->pending_callbacks_.reserve(4);
  this->executing_callbacks_.reserve(4);
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
      // Capture point already passed, the flash is still on: capture right away. Restart the
      // post-time window so this callback gets the same lit capture window as the first reader.
      if (this->debug_) {
        ESP_LOGD(TAG, "Flash sequence active (post time), executing joined capture callback now");
      }
      if (callback)
        callback();
      // Restart the post-time window from now so this late capture still has a full lit window.
      // A callback may have cancelled the sequence, so only reschedule while it is still active.
      if (this->is_active_ && this->captured_)
        this->schedule_flash_off_();
    } else {
      if (this->debug_) {
        ESP_LOGD(TAG, "Flash sequence already active, joining it");
      }
      this->pending_callbacks_.push_back(std::move(callback));
    }
    return;
  }

  this->is_active_ = true;
  this->captured_ = false;
  this->pending_callbacks_.clear();
  this->pending_callbacks_.push_back(std::move(callback));
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
    // Swap instead of moving into a local: keeps both vectors' capacity for the next cycle.
    this->executing_callbacks_.clear();
    this->pending_callbacks_.swap(this->executing_callbacks_);
    for (size_t i = 0; i < this->executing_callbacks_.size(); ++i) {
      // Move the callback out first: it may cancel/replace the sequence and clear the vector.
      auto cb = std::move(this->executing_callbacks_[i]);
      if (cb)
        cb();
    }
    this->executing_callbacks_.clear();

    // A callback may have cancelled the sequence (and possibly started a new one).
    if (this->is_active_ && this->captured_)
      this->schedule_flash_off_();
  });
}

void FlashLightController::schedule_flash_off_() {
  this->cancel_timeout(TIMEOUT_OFF);
  this->set_timeout(TIMEOUT_OFF, this->flash_post_time_, [this]() {
    this->disable_flash();
    this->captured_ = false;
    this->is_active_ = false;
    if (this->debug_) {
      ESP_LOGD(TAG, "Flash sequence complete (Post-time %u ms finished)", this->flash_post_time_);
    }
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
  this->executing_callbacks_.clear();
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
