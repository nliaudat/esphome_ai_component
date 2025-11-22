/**
 * @file meter_reader_tflite.h
 * @brief ESPHome component for meter reading using TensorFlow Lite Micro.
 * 
 * This component captures images from an ESP32 camera, processes them through
 * a TFLite model, and extracts meter readings with confidence scores.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
  bool camera_window_configured_{false};

/** ########### PRIVATE ############# **/
 private:
  std::shared_ptr<camera::CameraImage> pending_frame_{nullptr};  ///< Single frame buffer
  std::atomic<bool> frame_available_{false};    ///< Flag indicating frame available for processing
  std::atomic<bool> processing_frame_{false};   ///< Flag indicating frame processing in progress
  std::atomic<bool> frame_requested_{false};    ///< Flag indicating frame request pending
  uint32_t last_frame_received_{0};             ///< Timestamp of last received frame
  uint32_t last_request_time_{0};               ///< Timestamp of last frame request
  std::atomic<bool> pause_processing_{false};
  light::LightState* flash_light_{nullptr};  ///< Flash light component
  bool flash_light_enabled_{false};          ///< Whether flash light is enabled
  uint32_t flash_duration_{200};             ///< Flash duration in milliseconds
  std::atomic<bool> flash_auto_controlled_{false};
  uint32_t flash_pre_time_{5000};    // 5 seconds before update
  uint32_t flash_post_time_{2000};   // 2 seconds after update
  bool flash_scheduled_{false};      // Track if flash is scheduled
  
  // Original camera configuration storage
  int original_camera_width_{0};      ///< Original camera width before any window changes
  int original_camera_height_{0};     ///< Original camera height before any window changes
  std::string original_pixel_format_; ///< Original pixel format before any changes
  
  /**
   * @brief Process the next available frame in the buffer.
   */
  void process_available_frame();
  
  /**
  * @brief Control flash light around image capture
  */
  void enable_flash_light();
  bool is_flash_forced_on() const;
  void disable_flash_light();
  
  esp32_camera_utils::CameraWindowControl camera_window_control_;
  
  /**
   * @brief Reinitialize the ImageProcessor with current camera dimensions and format
   */
  void reinitialize_image_processor();
};

}  // namespace meter_reader_tflite
}  // namespace esphome