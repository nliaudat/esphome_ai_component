#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/flash_light_controller/flash_light_controller.h"

#include <atomic>
#include <functional>

namespace esphome {
namespace ssocr_reader {

class FlashlightCoordinator {
 public:
  void setup(Component* parent, light::LightState* legacy_light, 
             flash_light_controller::FlashLightController* controller);

  // Configuration
  void set_timing(uint32_t pre_time, uint32_t post_time);
  void set_update_interval(uint32_t interval_ms);
  
  // Logic
  // Returns TRUE if flash logic is active/waiting (i.e. don't update normally)
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

 private:
  Component* parent_{nullptr};
  light::LightState* legacy_light_{nullptr};
  flash_light_controller::FlashLightController* controller_{nullptr};
  
  uint32_t pre_time_{5000};
  uint32_t post_time_{2000};
  uint32_t update_interval_{60000}; // Default
  
  bool scheduled_{false};
  std::atomic<bool> auto_controlled_{false}; // If we turned it on autonomously
  
  std::function<void()> request_frame_callback_;
  
  // Helper to schedule delayed tasks
  // Note: Coordinators aren't components, so they can't use set_timeout directly easily 
  // unless we pass the parent component or use App.scheduler.
  // We will assume parent passes a scheduler or we use Application directly.
  template<typename F>
  void schedule_timeout(uint32_t ms, F&& f);
};

}  // namespace meter_reader_tflite
}  // namespace esphome
