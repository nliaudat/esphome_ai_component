#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_state.h"
#include <functional>

namespace esphome {
namespace flash_light_controller {

class FlashLightController : public Component {
 public:
  void setup() override;
  void set_flash_light(light::LightState *flash_light);
  void set_flash_pre_time(uint32_t pre_time) { flash_pre_time_ = pre_time; }
  void set_flash_post_time(uint32_t post_time) { flash_post_time_ = post_time; }
  uint32_t get_flash_pre_time() const { return flash_pre_time_; }
  uint32_t get_flash_post_time() const { return flash_post_time_; }
  
  void set_debug(bool debug) { debug_ = debug; }
  
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
   * @param callback Function to execute when flash is ready (usually image capture)
   */
  void initiate_capture_sequence(CaptureCallback callback);
  
  bool is_active() const { return is_active_; }

  void enable_flash();
  void disable_flash();

 protected:
  light::LightState *flash_light_{nullptr};
  uint32_t flash_pre_time_{5000};
  uint32_t flash_post_time_{2000};
  bool is_active_{false};
  bool debug_{false};
};

}  // namespace flash_light_controller
}  // namespace esphome
