#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/globals/globals_component.h"
#include "esphome/core/helpers.h"

// Coordinators
#include "tflite_coordinator.h"
#include "camera_coordinator.h"
#include "flashlight_coordinator.h"
#include "debug_coordinator.h"

// Include Value Validator Coordinator
#include "value_validator_coordinator.h"

#ifdef USE_DATA_COLLECTOR
#include "esphome/components/data_collector/data_collector.h"
#endif

#include "esphome/components/esp32_camera_utils/crop_zone_handler.h"
#include "esphome/components/esp32_camera_utils/esp32_camera_utils.h"

#ifdef USE_WEB_SERVER
#include "esphome/components/web_server_base/web_server_base.h"
#endif

#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>

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

#ifdef USE_WEB_SERVER
#include "esphome/components/web_server_base/web_server_base.h"
#endif

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
  
  // Set Validator (External)
  void set_validator(value_validator::ValueValidator *v) { validation_coord_.set_validator(v); }

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

  void set_generate_preview(bool generate);
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
  // Obsolete local validation setters - removed or must be handled by external validator config
  // void set_allow_negative_rates(bool allow); 
  // void set_max_absolute_diff(int max_diff);
  // ...
  void set_frame_request_timeout(uint32_t ms) { frame_request_timeout_ms_ = ms; }
  void set_high_confidence_threshold(float threshold) { high_confidence_threshold_ = threshold; }
  void set_last_valid_value(float value);
  void set_last_valid_value(const std::string &value);

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
  void update_calibration(float confidence); // Modified signature to match logic
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
  
  #ifdef USE_DATA_COLLECTOR
  void set_data_collector(data_collector::DataCollector *collector) { data_collector_ = collector; }
  void set_collect_low_confidence(bool collect) { collect_low_confidence_ = collect; }
  #endif

  // Dynamic Resource Management
  void unload_resources();
  void reload_resources();
  void set_unload_button(button::Button *b) { unload_button_ = b; }
  void set_reload_button(button::Button *b) { reload_button_ = b; }

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
  void set_debug(bool debug);

 protected:
  esp32_camera_utils::Esp32CameraUtils *esp32_camera_utils_{nullptr};
  #ifdef USE_DATA_COLLECTOR
  data_collector::DataCollector *data_collector_{nullptr};
  #endif

  // Coordinators
  TFLiteCoordinator tflite_coord_;
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  DebugCoordinator debug_coord_;

  esp32_camera_utils::CropZoneHandler crop_zone_handler_;
  
  // Refactor: Use coordinator instead of local object
  // ValueValidator output_validator_; 
  ValueValidatorCoordinator validation_coord_;

  // State
  std::atomic<bool> pause_processing_{false};
  std::atomic<bool> frame_requested_{false};
  std::atomic<bool> frame_available_{false};
  std::atomic<bool> processing_frame_{false};
  std::shared_ptr<camera::CameraImage> pending_frame_{nullptr};
  std::mutex frame_mutex_;
  uint32_t last_request_time_{0};
  uint32_t pending_frame_acquisition_time_{0};

  // Config
  float confidence_threshold_{0.85f};
  // bool allow_negative_rates_{false}; // Moved to validator component
  // int max_absolute_diff_{100};       // Moved
  uint32_t frame_request_timeout_ms_{15000};
  float high_confidence_threshold_{0.90f}; // Threshold for 'high confidence' logging distinction
  float rotation_{0.0f};
  bool generate_preview_{false};
  bool show_crop_areas_{true};
  bool debug_{false};
  bool debug_memory_enabled_{false}; // Runtime flag
  bool window_active_{false};
  bool enable_flash_calibration_{false};

  // Data Collection
  #ifdef USE_DATA_COLLECTOR
  bool collect_low_confidence_{true};
  float low_confidence_trigger_threshold_{0.0f}; 
  
  enum CollectionState {
      COLLECTION_IDLE,
      COLLECTION_WAITING_FOR_FLASH,
      COLLECTION_WAITING_FOR_FRAME
  };
  std::atomic<CollectionState> collection_state_{COLLECTION_IDLE};
  
  // Storage for pending collection
  struct PendingCollection {
      float value;
      float confidence;
  } pending_collection_;
  
  void trigger_low_confidence_collection(float value, float confidence);
  #endif

  // bool strict_confidence_check_{false}; // Moved

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

  button::Button *unload_button_{nullptr};
  button::Button *reload_button_{nullptr};


  // Helper
  void process_available_frame();
  void process_full_image(std::shared_ptr<camera::CameraImage> frame);
  float combine_readings(const esphome::StaticVector<float, 16>& readings);
  bool validate_and_update_reading(float raw, float conf, float& val);
  bool validate_and_update_reading(const esphome::StaticVector<float, 16>& digits, const esphome::StaticVector<float, 16>& confidences, float& val);

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
      esphome::StaticVector<float, 16> readings;
      esphome::StaticVector<float, 16> probabilities; // Confidence
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

  // Sync state for safe unloading
  std::atomic<bool> is_inferencing_{false};
  std::atomic<bool> task_running_{false};

  void start_inference_pipeline();
  void stop_inference_pipeline();

#endif
};

}  // namespace meter_reader_tflite
}  // namespace esphome