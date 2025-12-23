#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/globals/globals_component.h"

// Coordinators
#include "tflite_coordinator.h"
#include "camera_coordinator.h"
#include "flashlight_coordinator.h"
#include "debug_coordinator.h"

#include "esphome/components/esp32_camera_utils/crop_zone_handler.h"
#include "esphome/components/esp32_camera_utils/esp32_camera_utils.h"
#include "value_validator.h"

#ifdef USE_WEB_SERVER
#include "esphome/components/web_server_base/web_server_base.h"
#endif

#include <memory>
#include <vector>
#include <string>
#include <atomic>

// Check for Dual Core capability
#if !defined(CONFIG_FREERTOS_UNICORE) && (portNUM_PROCESSORS > 1)
    #define SUPPORT_DOUBLE_BUFFERING
#endif

#ifdef SUPPORT_DOUBLE_BUFFERING
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

  #include "esphome/components/esp32_camera/esp32_camera.h"

namespace esphome {
namespace meter_reader_tflite {

class MeterReaderTFLite : public PollingComponent, public camera::CameraImageReader, public camera::CameraListener {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  ~MeterReaderTFLite() override;

  float get_setup_priority() const override { return setup_priority::LATE; }

  // CameraListener
  void on_camera_image(const std::shared_ptr<camera::CameraImage> &image) override;

  // CameraImageReader
  void set_image(std::shared_ptr<camera::CameraImage> image) override {};
  size_t available() const override { return 0; };
  uint8_t *peek_data_buffer() override { return nullptr; };
  void consume_data(size_t consumed) override {};
  void return_image() override {};

  // Config Setters (Delegated)
  void set_confidence_threshold(float threshold) { confidence_threshold_ = threshold; }
  void set_tensor_arena_size(size_t size_bytes); // -> TFLite
  void set_model(const uint8_t *model, size_t length); // -> TFLite
  
  void set_value_sensor(sensor::Sensor *sensor) { value_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { confidence_sensor_ = sensor; }

  void set_crop_zones(const std::string &zones_json);
  void set_crop_zones_global(globals::GlobalsComponent<std::string> *global_var) {
      crop_zone_handler_.set_crop_zones_global(global_var);
  }
  
  void set_camera_image_format(int width, int height, const std::string &pixel_format); // -> CameraCoord & TFLite
  void set_camera(camera::Camera *camera); // -> CameraCoord
  
  sensor::Sensor *get_value_sensor() const { return value_sensor_; }
  sensor::Sensor *get_confidence_sensor() const { return confidence_sensor_; }
  
  void set_model_config(const std::string &model_type); // -> TFLite
  void set_rotation(float rotation) { rotation_ = rotation; } // Storage, passed to TFLite late
  
  void set_generate_preview(bool generate) { generate_preview_ = generate; }
  void set_show_crop_areas(bool show) { show_crop_areas_ = show; }
#ifdef DEV_ENABLE_ROTATION
  void take_preview_image();
  void capture_preview();
  std::shared_ptr<camera::CameraImage> get_preview_image();
  
 private:
  void update_preview_image(std::shared_ptr<camera::CameraImage> image);
  std::shared_ptr<camera::CameraImage> last_preview_image_{nullptr};
  std::mutex preview_mutex_;
  bool request_preview_{false};
 public:
#endif

  // Debug/Reporting
  static void register_service(MeterReaderTFLite *comp) { comp->print_debug_info(); }
  void print_debug_info();
  
  // Validation
  void set_allow_negative_rates(bool allow) { allow_negative_rates_ = allow; }
  void set_max_absolute_diff(int max_diff) { max_absolute_diff_ = max_diff; }
  void set_frame_request_timeout(uint32_t ms) { frame_request_timeout_ms_ = ms; }
  void set_high_confidence_threshold(float threshold) { high_confidence_threshold_ = threshold; }

  // Pause
  void set_pause_processing(bool pause) { pause_processing_.store(pause); }
  bool get_pause_processing() const { return pause_processing_.load(); }

  // Overrides
  void set_update_interval(uint32_t ms);

  // Flashlight
  void set_flash_light(light::LightState* flash_light);
  void set_flash_controller(flash_light_controller::FlashLightController* controller);
  void set_flash_pre_time(uint32_t ms);
  void set_flash_post_time(uint32_t ms);
  void force_flash_inference(); // Service
  void set_enable_flash_calibration(bool enable) { enable_flash_calibration_ = enable; }

  // Calibration
  void start_flash_calibration();
  void update_calibration(float confidence);
  bool is_calibrating() const { return calibration_.state != FlashCalibrationHandler::IDLE; }

  // Window Control
  void set_camera_window_offset_x(int x);
  void set_camera_window_offset_y(int y);
  void set_camera_window_width(int w);
  void set_camera_window_height(int h);
  void set_camera_window_configured(bool c);
  
  bool reset_camera_window();
  bool set_camera_window(int offset_x, int offset_y, int width, int height);

  // Logs
  void set_inference_logs(text_sensor::TextSensor *sensor) { inference_logs_ = sensor; }
  void set_main_logs(text_sensor::TextSensor *main_logs) { main_logs_ = main_logs; }

  void set_esp32_camera_utils(esp32_camera_utils::Esp32CameraUtils *utils) { esp32_camera_utils_ = utils; }
  
#ifdef DEBUG_METER_READER_MEMORY
  void set_tensor_arena_size_sensor(sensor::Sensor *s) { tensor_arena_size_sensor_ = s; }
  void set_tensor_arena_used_sensor(sensor::Sensor *s) { tensor_arena_used_sensor_ = s; }
  void set_process_free_heap_sensor(sensor::Sensor *s) { process_free_heap_sensor_ = s; }
  void set_process_free_psram_sensor(sensor::Sensor *s) { process_free_psram_sensor_ = s; }
  void set_pool_job_efficiency_sensor(sensor::Sensor *s) { pool_job_efficiency_sensor_ = s; }
  void set_pool_result_efficiency_sensor(sensor::Sensor *s) { pool_result_efficiency_sensor_ = s; }
  void set_arena_efficiency_sensor(sensor::Sensor *s) { arena_efficiency_sensor_ = s; }
  void set_heap_fragmentation_sensor(sensor::Sensor *s) { heap_fragmentation_sensor_ = s; }
  void set_debug_memory_enabled(bool enabled) { debug_memory_enabled_ = enabled; }
#endif

  void set_total_inference_time_sensor(sensor::Sensor *s) { total_inference_time_sensor_ = s; }
  void set_debug_timing(bool enabled) { debug_timing_ = enabled; }


#ifdef USE_WEB_SERVER
  void set_web_server(web_server_base::WebServerBase *web_server);
#endif

#ifdef DEBUG_METER_READER_TFLITE
  void set_debug_image(const uint8_t* data, size_t size);
  void test_with_debug_image();
  void set_debug_mode(bool m);
  void debug_test_with_pattern();
#endif

 protected:
  esp32_camera_utils::Esp32CameraUtils *esp32_camera_utils_{nullptr};

  // Coordinators
  TFLiteCoordinator tflite_coord_;
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  DebugCoordinator debug_coord_;

  esp32_camera_utils::CropZoneHandler crop_zone_handler_;
  ValueValidator output_validator_;

  // State
  std::atomic<bool> pause_processing_{false};
  std::atomic<bool> frame_requested_{false};
  std::atomic<bool> frame_available_{false};
  std::atomic<bool> processing_frame_{false};
  std::shared_ptr<camera::CameraImage> pending_frame_{nullptr};
  uint32_t last_request_time_{0};
  uint32_t pending_frame_acquisition_time_{0};
  
  // Config
  float confidence_threshold_{0.85f};
  bool allow_negative_rates_{false};
  int max_absolute_diff_{100};
  uint32_t frame_request_timeout_ms_{15000};
  float high_confidence_threshold_{0.90f};
  float rotation_{0.0f};
  bool generate_preview_{false};
  bool show_crop_areas_{true};
  bool debug_memory_enabled_{false}; // Runtime flag
  bool window_active_{false};
  bool enable_flash_calibration_{false};

  // Calibration
  struct FlashCalibrationHandler {
      enum State { IDLE, CALIBRATING_PRE, CALIBRATING_POST, FINISHED };
      State state{IDLE};
      uint32_t current_pre{0};
      uint32_t current_post{0};
      uint32_t best_pre{0};
      uint32_t best_post{0};
      float baseline_confidence{0.0f};
      float best_confidence{0.0f};
      uint32_t step_start_time{0};
      
      // Configuration
      uint32_t start_pre{7000};
      uint32_t end_pre{100};
      uint32_t step_pre{500};
      
      uint32_t start_post{2000};
      uint32_t end_post{0};
      uint32_t step_post{200};
  } calibration_;

  void update_calibration(); 

  // Sensor Refs
  sensor::Sensor *value_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  text_sensor::TextSensor *inference_logs_{nullptr};

  text_sensor::TextSensor *main_logs_{nullptr};

#ifdef DEBUG_METER_READER_MEMORY
  sensor::Sensor *tensor_arena_size_sensor_{nullptr};
  sensor::Sensor *tensor_arena_used_sensor_{nullptr};
  sensor::Sensor *process_free_heap_sensor_{nullptr};
  sensor::Sensor *process_free_psram_sensor_{nullptr};
  sensor::Sensor *pool_job_efficiency_sensor_{nullptr};
  sensor::Sensor *pool_result_efficiency_sensor_{nullptr};
  sensor::Sensor *arena_efficiency_sensor_{nullptr};
  sensor::Sensor *heap_fragmentation_sensor_{nullptr};
#endif
  
  sensor::Sensor *total_inference_time_sensor_{nullptr};
  bool debug_timing_{false};


  // Helper
  void process_available_frame();
  void process_full_image(std::shared_ptr<camera::CameraImage> frame);
  float combine_readings(const std::vector<float>& readings);
  bool validate_and_update_reading(float raw, float conf, float& val);
  
#ifdef USE_WEB_SERVER
  web_server_base::WebServerBase *web_server_{nullptr};
#endif

#ifdef SUPPORT_DOUBLE_BUFFERING
  // Double Buffering / Multithreading
 public:
  struct InferenceJob {
      std::shared_ptr<camera::CameraImage> frame; // Keep managed
      std::vector<esp32_camera_utils::ImageProcessor::ProcessResult> crops;
      uint32_t start_time;
  };

  struct InferenceResult {
      std::vector<float> readings;
      std::vector<float> probabilities; // Confidence
      uint32_t inference_time;
      uint32_t total_start_time;
      #ifdef DEBUG_METER_READER_MEMORY
      size_t arena_used_bytes;
      #endif
      bool success;
  };

  QueueHandle_t input_queue_{nullptr};
  QueueHandle_t output_queue_{nullptr};
  TaskHandle_t inference_task_handle_{nullptr};
  static void inference_task(void *arg);
#endif
};

}  // namespace meter_reader_tflite
}  // namespace esphome