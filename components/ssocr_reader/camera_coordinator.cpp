#include "camera_coordinator.h"

#include "esphome/core/log.h"

#include "esphome/core/hal.h"

#include "esphome/core/application.h"  // For delay() if needed



namespace esphome {

namespace ssocr_reader {



static const char *const TAG = "camera_coordinator";



// Camera stabilization delays (blocking - camera must be stable before returning)

// Reduced from 500/1000ms after testing showed modern sensors stabilize faster

// NOTE: delay() is used here intentionally as callers (set_window/reset_window) expect
// immediate results and run in service context, not in loop(). This is a documented
// exception to §3.1 "No delay() in loop()" per .ai/instructions.md
static constexpr uint32_t WINDOW_SET_STABILIZATION_MS = 100;

static constexpr uint32_t WINDOW_RESET_STABILIZATION_MS = 200;



void CameraCoordinator::set_camera(esp32_camera::ESP32Camera *camera) { this->camera_ = camera; }



void CameraCoordinator::set_config(int width, int height, const std::string &pixel_format) {

  this->current_width_ = width;

  this->current_height_ = height;

  this->current_format_ = pixel_format;



  // Assume initial config is "original"

  if (this->orig_width_ == 0) {

    this->orig_width_ = width;

    this->orig_height_ = height;

    this->orig_format_ = pixel_format;

  }

}



bool CameraCoordinator::supports_window() const {

  if (!this->camera_)

    return false;

  return this->window_control_.supports_window(this->camera_);

}



bool CameraCoordinator::set_window(int offset_x, int offset_y, int width, int height) {

  if (!this->camera_)

    return false;

  ESP_LOGI(TAG, "Setting camera window: off(%d,%d) size(%dx%d)", offset_x, offset_y, width, height);



  bool success = this->window_control_.set_window_with_reset(

      this->camera_, esp32_camera_utils::CameraWindowControl::WindowConfig{offset_x, offset_y, width, height, true});



  if (success) {

    auto new_dims = this->window_control_.update_dimensions_after_window(

        this->camera_, esp32_camera_utils::CameraWindowControl::WindowConfig{offset_x, offset_y, width, height, true},

        this->current_width_, this->current_height_);



    this->current_width_ = new_dims.first;

    this->current_height_ = new_dims.second;



    // Blocking delay required: camera must stabilize before returning success

    // Cannot use set_timeout() as caller expects immediate result

    delay(WINDOW_SET_STABILIZATION_MS);

    return true;

  }

  ESP_LOGE(TAG, "Failed to set camera window");

  this->reset_window();

  return false;

}



bool CameraCoordinator::reset_window() {

  ESP_LOGI(TAG, "Resetting camera window to full frame");

  // Hard reset preferred

  bool success = this->window_control_.hard_reset_camera(this->camera_);

  if (!success) {

    ESP_LOGW(TAG, "Hard reset failed, trying soft reset");

    success = this->window_control_.soft_reset_camera(this->camera_);

  }



  if (success) {

    success = this->window_control_.reset_to_full_frame_with_dimensions(

        this->camera_, this->orig_width_, this->orig_height_, this->current_width_, this->current_height_);



    if (success) {

      this->current_format_ = this->orig_format_;

      delay(WINDOW_RESET_STABILIZATION_MS);  // Blocking: camera must stabilize

    }

  }



  if (!success) {

    ESP_LOGE(TAG, "Failed to reset camera window completely");

