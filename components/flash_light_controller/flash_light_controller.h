#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_state.h"
#include <functional>
#include <atomic>

#include "esphome/core/defines.h"

#ifdef CONFIG_CAMERA_AF_SUPPORT
#include "esp_camera.h"
#include "esp_camera_af.h"
#endif

#ifdef USE_FLASH_LIGHT_CONTROLLER

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
  void set_flash_brightness(float brightness) { flash_brightness_ = brightness; }

#ifdef CONFIG_CAMERA_AF_SUPPORT
  void set_enable_autofocus(bool enable) { enable_autofocus_ = enable; }
  void set_af_timeout(uint32_t timeout_ms) { af_timeout_ms_ = timeout_ms; }
#endif
  
  void set_debug(bool debug) { debug_ = debug; }
  void set_idle_brightness(float brightness) { idle_brightness_ = brightness; }
  
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
  float flash_brightness_{1.0f};  // 1.0 = 100% flash pulse brightness
  float idle_brightness_{0.0f};  // 0.0 = fully off, >0 = dim idle (e.g. 0.1 = 10%)
  std::atomic<bool> is_active_{false};
  bool debug_{false};
#ifdef CONFIG_CAMERA_AF_SUPPORT
  bool enable_autofocus_{false};
  uint32_t af_timeout_ms_{2000};
  bool af_initialized_{false};
#endif
};

}  // namespace flash_light_controller
}  // namespace esphome

#endif  // USE_FLASH_LIGHT_CONTROLLER
