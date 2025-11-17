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
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/globals/globals_component.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/text_sensor/text_sensor.h"
// #include "esphome/core/application.h"
#include "model_handler.h"
#include "memory_manager.h"
#include "image_processor.h"
#include "crop_zones.h"
#include "model_config.h"
#include "camera_control.h"
#include "output_validation.h"
#include <memory>
#include <vector>
#include <string>
#include <atomic>

namespace esphome {
namespace meter_reader_tflite {

class MeterReaderTFLite : public PollingComponent, public camera::CameraImageReader {
 public:
  void setup() override;
  
  void set_crop_zones_global(globals::GlobalsComponent<std::string> *global_var) {
      crop_zone_handler_.set_crop_zones_global(global_var);
  }
  
  void update() override;
  void loop() override;
  ~MeterReaderTFLite() override;
  
  float get_setup_priority() const override { return setup_priority::LATE; }

  // CameraImageReader implementation
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

  // Configuration setters
  void set_confidence_threshold(float threshold) { confidence_threshold_ = threshold; }
  void set_tensor_arena_size(size_t size_bytes) { tensor_arena_size_requested_ = size_bytes; }
  void set_model(const uint8_t *model, size_t length);
  
  void set_value_sensor(sensor::Sensor *sensor) { value_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { confidence_sensor_ = sensor; }
  
  void set_crop_zones(const std::string &zones_json);
  void set_camera_image_format(int width, int height, const std::string &pixel_format);
  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }
  void set_model_config(const std::string &model_type);
  
  void print_debug_info();
  void report_memory_status();
  
  uint32_t get_frames_processed() const { return frames_processed_; }
  uint32_t get_frames_skipped() const { return frames_skipped_; }
  size_t get_arena_peak_bytes() const { return model_handler_.get_arena_peak_bytes(); }
  
  static void register_service(MeterReaderTFLite *comp) {
    comp->print_debug_info();
  }
  
  float combine_readings(const std::vector<float> &readings);
  float get_last_reading() const { return last_reading_; }
  float get_last_confidence() const { return last_confidence_; }
  
  float get_confidence_threshold() const { return confidence_threshold_; }
  
  // Model information getters
  int get_model_input_width() const { return model_handler_.get_input_width(); }
  int get_model_input_height() const { return model_handler_.get_input_height(); }
  int get_model_input_channels() const { return model_handler_.get_input_channels(); } 
  
  void set_pause_processing(bool pause) { 
      pause_processing_.store(pause); 
      ESP_LOGI(TAG, "AI processing %s", pause ? "PAUSED" : "RESUMED");
  }

  bool get_pause_processing() const { 
      return pause_processing_.load(); 
  }
  
  // Output validation configuration
  void set_allow_negative_rates(bool allow) { allow_negative_rates_ = allow; }
  void set_max_absolute_diff(int max_diff) { max_absolute_diff_ = max_diff; }
  
#ifdef DEBUG_METER_READER_TFLITE
  void set_debug_image(const uint8_t* data, size_t size);
  void test_with_debug_image();
  void test_with_debug_image_all_configs();
  void set_debug_mode(bool debug_mode);
  void debug_test_with_pattern();
#endif

  void set_flash_light(light::LightState* flash_light);
  void set_flash_pre_time(uint32_t pre_time) { flash_pre_time_ = pre_time; }
  void set_flash_post_time(uint32_t post_time) { flash_post_time_ = post_time; }

  // Camera window control methods
  bool set_camera_window(int offset_x, int offset_y, int width, int height);
  bool reset_camera_window();
  bool camera_supports_window() const;
  std::string get_camera_sensor_info() const;
  bool test_camera_after_reset();
  void basic_camera_recovery();
  
  // Camera window configuration setters
  void set_camera_window_offset_x(int offset_x) { camera_window_offset_x_ = offset_x; }
  void set_camera_window_offset_y(int offset_y) { camera_window_offset_y_ = offset_y; }
  void set_camera_window_width(int width) { camera_window_width_ = width; }
  void set_camera_window_height(int height) { camera_window_height_ = height; }
  void set_camera_window_configured(bool configured) { camera_window_configured_ = configured; } 
  
  void force_flash_inference();
  
  void set_inference_logs(text_sensor::TextSensor *sensor) { inference_logs_ = sensor; }
  void set_main_logs(text_sensor::TextSensor *sensor) { main_logs_ = sensor; }

/** ########### PROTECTED ############# **/
 protected:
  // sensor::Sensor *confidence_sensor_{nullptr};  ///< Sensor for confidence values
  uint32_t frames_processed_{0};                ///< Counter for successfully processed frames
  uint32_t frames_skipped_{0};                  ///< Counter for skipped frames
  
  
#ifdef DEBUG_METER_READER_TFLITE  
  std::shared_ptr<camera::CameraImage> debug_image_;
#endif
  
  /**
   * @brief Allocate tensor arena memory for TFLite model.
   * @return true if allocation successful, false otherwise
   */
  bool allocate_tensor_arena();
  
  /**
   * @brief Load and initialize the TFLite model.
   * @return true if model loaded successfully, false otherwise
   */
  bool load_model();
  
  /**
   * @brief Process a full camera image through the pipeline.
   * @param frame Shared pointer to the camera image to process
   */
  void process_full_image(std::shared_ptr<camera::CameraImage> frame);
  
  /**
   * @brief Process model inference results and extract values.
   * @param result Processed image result from ImageProcessor
   * @param value Output parameter for extracted meter value
   * @param confidence Output parameter for confidence score
   * @return true if processing successful, false otherwise
   */
  bool process_model_result(const ImageProcessor::ProcessResult& result, float* value, float* confidence);
   
  /**
   * @brief Setup output validation with configured parameters
   */
  void setup_output_validation();
  
  /**
   * @brief Validate reading using output validator
   * @param raw_reading The raw reading from model inference
   * @param confidence The confidence score for the reading
   * @param validated_reading Output parameter for validated reading
   * @return true if reading is valid, false otherwise
   */
  bool validate_and_update_reading(float raw_reading, float confidence, float& validated_reading);

  // Configuration parameters
  int camera_width_{0};                      ///< Camera image width in pixels
  int camera_height_{0};                     ///< Camera image height in pixels
  std::string pixel_format_{"RGB888"};       ///< Camera pixel format
  float confidence_threshold_{0.7f};         ///< Minimum confidence threshold for valid readings
  size_t tensor_arena_size_requested_{50 * 1024};  ///< Requested tensor arena size
  std::string model_type_{"default"};        ///< Model type identifier
  bool allow_negative_rates_{false};         ///< Whether to allow negative rate changes
  int max_absolute_diff_{100};               ///< Maximum absolute difference allowed between readings

  // State variables
  size_t tensor_arena_size_actual_{0};       ///< Actual allocated tensor arena size
  bool model_loaded_{false};                 ///< Model loading status flag
  const uint8_t *model_{nullptr};            ///< Pointer to model data
  size_t model_length_{0};                   ///< Size of model data in bytes
  sensor::Sensor *value_sensor_{nullptr};    ///< Sensor for meter values
  sensor::Sensor *confidence_sensor_{nullptr};
  
  float last_reading_{0.0f};
  float last_confidence_{0.0f};
  
  text_sensor::TextSensor *inference_logs_{nullptr};
  text_sensor::TextSensor *main_logs_{nullptr};
  
  
  esp32_camera::ESP32Camera *camera_{nullptr}; ///< Camera component reference
  bool debug_mode_ = false;                  ///< Debug mode flag

  // Component instances
  MemoryManager memory_manager_;             ///< Memory management utilities
  ModelHandler model_handler_;               ///< TFLite model handling
  std::unique_ptr<ImageProcessor> image_processor_;  ///< Image processing utilities
  CropZoneHandler crop_zone_handler_;        ///< Crop zone management
  OutputValidator output_validator_;         ///< Output validation and historical data
  MemoryManager::AllocationResult tensor_arena_allocation_;  ///< Tensor arena allocation result
  
  // Camera window configuration storage
  int camera_window_offset_x_{0};
  int camera_window_offset_y_{0};
  int camera_window_width_{0};
  int camera_window_height_{0};
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
  
  camera_control::CameraWindowControl camera_window_control_;
  
  /**
   * @brief Reinitialize the ImageProcessor with current camera dimensions and format
   */
  void reinitialize_image_processor();
};

}  // namespace meter_reader_tflite
}  // namespace esphome