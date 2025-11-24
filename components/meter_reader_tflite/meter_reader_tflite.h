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
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/tflite_micro_helper/model_handler.h"
#include "esphome/components/tflite_micro_helper/memory_manager.h"
#include "esphome/components/esp32_camera_utils/image_processor.h"
#include "esphome/components/esp32_camera_utils/crop_zone_handler.h"
#include "esphome/components/esp32_camera_utils/camera_window_control.h"
#include "esphome/components/flash_light_controller/flash_light_controller.h"
#include "value_validator.h"
#include <atomic>

namespace esphome {
namespace meter_reader_tflite {

class MeterReaderTFLite : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  ~MeterReaderTFLite();

  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }
  void set_model(const uint8_t *model, size_t length);
  void set_confidence_threshold(float threshold) { confidence_threshold_ = threshold; }
  void set_tensor_arena_size(size_t size) { tensor_arena_size_requested_ = size; }
  
  void set_value_sensor(sensor::Sensor *sensor) { value_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { confidence_sensor_ = sensor; }
  void set_inference_logs(text_sensor::TextSensor *sensor) { inference_logs_ = sensor; }
  void set_main_logs(text_sensor::TextSensor *sensor) { main_logs_ = sensor; }
  
  #ifdef DEBUG_METER_READER_TFLITE
  void set_debug_mode(bool debug) { debug_mode_ = debug; }
  void set_debug_image(const uint8_t *data, size_t size);
  #endif
  
  // Flash light controller setter
  void set_flash_controller(flash_light_controller::FlashLightController *controller) { flash_controller_ = controller; }
  
  // Crop zones global variable setter
  void set_crop_zones_global(globals::GlobalsComponent<std::string> *crop_zones) { 
      crop_zone_handler_.set_crop_zones_global(crop_zones);
  }
  
  // Camera window configuration
  void set_camera_window_configured(bool configured) { camera_window_configured_ = configured; }
  void set_camera_window_offset_x(int offset) { camera_window_offset_x_ = offset; }
  void set_camera_window_offset_y(int offset) { camera_window_offset_y_ = offset; }
  void set_camera_window_width(int width) { camera_window_width_ = width; }
  void set_camera_window_height(int height) { camera_window_height_ = height; }
  
  // Validation configuration
  void set_allow_negative_rates(bool allow) { allow_negative_rates_ = allow; }
  void set_max_absolute_diff(int diff) { max_absolute_diff_ = diff; }
  
  // Helper to set camera format (called from codegen)
  void set_camera_image_format(int width, int height, const std::string &pixel_format);
  
  // Helper to set model type (called from codegen)
  void set_model_config(const std::string &model_type);
  
  // Public method to set crop zones (called from service)
  void set_crop_zones(const std::string &zones_json);
  
  // Public method to set camera window (called from service)
  bool set_camera_window(int offset_x, int offset_y, int width, int height) {
      return camera_window_control_.set_window(camera_, offset_x, offset_y, width, height);
  }
  
  // Public method to reset camera window (called from service)
  bool reset_camera_window();

 protected:
  esp32_camera::ESP32Camera *camera_{nullptr};
  const uint8_t *model_{nullptr};
  size_t model_length_{0};
  float confidence_threshold_{0.7f};
  size_t tensor_arena_size_requested_{0}; // Will be set from model_config.h or YAML
  
  sensor::Sensor *value_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  text_sensor::TextSensor *inference_logs_{nullptr};
  text_sensor::TextSensor *main_logs_{nullptr};
  
  bool debug_mode_{false};
  std::shared_ptr<camera::CameraImage> debug_image_{nullptr};
  
  // Helper components
  tflite_micro_helper::ModelHandler model_handler_;
  tflite_micro_helper::MemoryManager memory_manager_;
  tflite_micro_helper::MemoryManager::AllocationResult tensor_arena_allocation_;
  
  std::unique_ptr<esp32_camera_utils::ImageProcessor> image_processor_;
  esp32_camera_utils::CropZoneHandler crop_zone_handler_;
  
  // Flash controller
  flash_light_controller::FlashLightController *flash_controller_{nullptr};
  
  // Validation
  ValueValidator value_validator_;
  bool allow_negative_rates_{false};
  int max_absolute_diff_{100};
  float last_reading_{0.0f};
  float last_confidence_{0.0f};
  
  // State
  bool model_loaded_{false};
  std::string model_type_{"digit_recognizer_v3_10cls_RGB"}; // Default
  int camera_width_{0};
  int camera_height_{0};
  std::string pixel_format_{"RGB888"};
  
  // Camera window configuration storage
  bool camera_window_configured_{false};
  int camera_window_offset_x_{0};
  int camera_window_offset_y_{0};
  int camera_window_width_{0};
  int camera_window_height_{0};
  
  // Internal methods
  bool load_model();
  bool allocate_tensor_arena();
  void process_full_image(std::shared_ptr<camera::CameraImage> frame);
  bool process_model_result(const esp32_camera_utils::ImageProcessor::ProcessResult& result, float* value, float* confidence);
  float combine_readings(const std::vector<float> &readings);
  void print_debug_info();
  
  #ifdef DEBUG_METER_READER_TFLITE
  void test_with_debug_image();
  void test_with_debug_image_all_configs();
  void debug_test_with_pattern();
  #endif
  
  bool test_camera_after_reset();
  void basic_camera_recovery();
  bool camera_supports_window() const;
  
  void setup_output_validation();
  bool validate_and_update_reading(float raw_reading, float confidence, float& validated_reading);

 private:
  std::shared_ptr<camera::CameraImage> pending_frame_{nullptr};  ///< Single frame buffer
  std::atomic<bool> frame_available_{false};    ///< Flag indicating frame available for processing
  std::atomic<bool> processing_frame_{false};   ///< Flag indicating frame processing in progress
  std::atomic<bool> frame_requested_{false};    ///< Flag indicating frame request pending
  uint32_t last_frame_received_{0};             ///< Timestamp of last received frame
  uint32_t last_request_time_{0};               ///< Timestamp of last frame request
  std::atomic<bool> pause_processing_{false};
  
  void set_pause_processing(bool pause) { pause_processing_ = pause; }
  bool get_pause_processing() const { return pause_processing_; }
  
  // Original camera configuration storage
  int original_camera_width_{0};      ///< Original camera width before any window changes
  int original_camera_height_{0};     ///< Original camera height before any window changes
  std::string original_pixel_format_; ///< Original pixel format before any changes
  
  /**
   * @brief Process the next available frame in the buffer.
   */
  void process_available_frame();
  
  esp32_camera_utils::CameraWindowControl camera_window_control_;
  
  /**
   * @brief Reinitialize the ImageProcessor with current camera dimensions and format
   */
  void reinitialize_image_processor();
};

}  // namespace meter_reader_tflite
}  // namespace esphome