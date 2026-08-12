#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_state.h"
#include <functional>
#include <atomic>

#include "esphome/core/defines.h"

#ifdef USE_FLASH_LIGHT_CONTROLLER

namespace esphome {
namespace flash_light_controller {

class FlashLightController : public Component {
 public:
  void setup() override;
  void set_flash_light(light::LightState *flash_light) { this->flash_light_ = flash_light; }

  /// Configure pre-time in ms. Values are clamped to >= 1 so the sequence cannot collapse.
  void set_flash_pre_time(uint32_t pre_time) {
    this->flash_pre_time_ = (pre_time == 0) ? 1 : pre_time;
  }
  /// Configure post-time in ms. Values are clamped to >= 1 so the sequence cannot collapse.
  void set_flash_post_time(uint32_t post_time) {
    this->flash_post_time_ = (post_time == 0) ? 1 : post_time;
  }
  uint32_t get_flash_pre_time() const { return this->flash_pre_time_; }
  uint32_t get_flash_post_time() const { return this->flash_post_time_; }

  /// Runtime changes to pre/post time only apply to the *next* capture sequence.
  void set_debug(bool debug) { this->debug_ = debug; }

  using CaptureCallback = std::function<void()>;

  /**
   * @brief Initiates the flash sequence for image capture.
   *
   * Sequence:
   * 1. Turn on flash
   * 2. Wait for flash_pre_time_ (warmup)
   * 3. Call callback (capture image)
   * 4. Wait for flash_post_time_ (ensure light during capture)
   * 5. Turn off flash
   *
   * The callback is expected to block until capture completes. If the capture is
   * asynchronous, the consumer must hold the flash active itself for the duration
   * of the capture (see capture_preview_sequence for an example).
   *
   * @param callback Function to execute when flash is ready (usually image capture)
   */
  void initiate_capture_sequence(CaptureCallback callback);

  bool is_active() const { return this->is_active_; }

  /**
   * @brief Force-abort an in-progress or stuck capture sequence.
   *
   * Cancels the pending timeouts, turns the flash off, and releases the active
   * flag so a new sequence can start immediately. Safe to call when idle.
   */
  void cancel_active_sequence();

  void enable_flash();
  void disable_flash();

 protected:
  light::LightState *flash_light_{nullptr};
  uint32_t flash_pre_time_{5000};
  uint32_t flash_post_time_{2000};
  std::atomic<bool> is_active_{false};
  bool debug_{false};
};

}  // namespace flash_light_controller
}  // namespace esphome

#endif  // USE_FLASH_LIGHT_CONTROLLER