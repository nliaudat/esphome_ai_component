#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ANALOG_READER

#include "esphome/core/component.h"
#include "esphome/components/light/light_state.h"

#ifdef USE_FLASH_LIGHT_CONTROLLER
#include "esphome/components/flash_light_controller/flash_light_controller.h"
#endif

#include <atomic>
#include <functional>

namespace esphome {
namespace analog_reader {

class FlashlightCoordinator {
 public:
#ifdef USE_FLASH_LIGHT_CONTROLLER
  void setup(Component* parent, light::LightState* legacy_light, 
             flash_light_controller::FlashLightController* controller);

  // Configuration
  void set_timing(uint32_t pre_time, uint32_t post_time);
  void set_update_interval(uint32_t interval_ms);
  
  // Logic
  bool update_scheduling(); 
  
  // Actions
  void enable_flash();
  void disable_flash();
  void force_inference(std::function<void()> frame_request_callback);
  
  // Helpers
  void capture_preview_sequence(std::function<void()> frame_request_callback);
  bool is_active() const { return scheduled_ || (controller_ && controller_->is_active()); }

  // Setters required for logic
  void set_request_frame_callback(std::function<void()> cb) { request_frame_callback_ = cb; }

  void set_debug(bool debug) { debug_ = debug; }

 private:
  Component* parent_{nullptr};
  light::LightState* legacy_light_{nullptr};
  flash_light_controller::FlashLightController* controller_{nullptr};
  
  uint32_t pre_time_{5000};
  uint32_t post_time_{2000};
  uint32_t update_interval_{60000}; // Default
  
  bool scheduled_{false};
  std::atomic<bool> auto_controlled_{false}; // If we turned it on autonomously
  bool debug_{false};
  
  std::function<void()> request_frame_callback_;
  
  template<typename F>
  void schedule_timeout(uint32_t ms, F&& f);
#else
  // Dummy implementation
  void setup(Component* parent, light::LightState* legacy_light, void* controller) {}
  void set_timing(uint32_t pre_time, uint32_t post_time) {}
  void set_update_interval(uint32_t interval_ms) {}
  bool update_scheduling() { return false; }
  void enable_flash() {}
  void disable_flash() {}
  void force_inference(std::function<void()> frame_request_callback) {}
  void capture_preview_sequence(std::function<void()> frame_request_callback) {}
  bool is_active() const { return false; }
  void set_request_frame_callback(std::function<void()> cb) {}
  void set_debug(bool debug) {}
#endif
};

}  // namespace analog_reader
}  // namespace esphome

#endif // USE_ANALOG_READER
