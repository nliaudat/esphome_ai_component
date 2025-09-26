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
#include "model_handler.h"
#include "memory_manager.h"
#include "image_processor.h"
#include "crop_zones.h"
#include "model_config.h"
#include <memory>
#include <vector>
#include <string>
#include <atomic>

// #include <mutex> // needs CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y # for frame_mutex_ in meter_reader_tflite.*
// #include <numeric> // for std::accumulate

#define DEBUG_DURATION  ///< Enable duration debugging macros

namespace esphome {
namespace meter_reader_tflite {

/**
 * @class MeterReaderTFLite
 * @brief Main component class for meter reading using TensorFlow Lite Micro.
 * 
 * This class handles image acquisition from ESP32 camera, preprocessing,
 * TFLite model inference, and result publication to sensors.
 */
class MeterReaderTFLite : public PollingComponent, public camera::CameraImageReader {
 public:
  /**
   * @brief Initialize the component and setup camera callback.
   */
  void setup() override;
  
  // void set_crop_zones_global(GlobalVarComponent<std::string> *crop_zones_global);
  
  /**
   * @brief Periodic update called based on configured interval.
   * Requests new frame processing if system is ready.
   */
  void update() override;
  
  /**
   * @brief Main processing loop. Handles frame processing and timeout management.
   */
  void loop() override;
  
  /**
   * @brief Component destructor. Cleans up resources.
   */
  ~MeterReaderTFLite() override;
  
  /**
   * @brief Get the setup priority for this component.
   * @return setup_priority::LATE to ensure camera is initialized first
   */
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
  
  /**
   * @brief Set the value sensor for meter readings.
   * @param sensor Pointer to the sensor component
   */
  void set_value_sensor(sensor::Sensor *sensor) { value_sensor_ = sensor; }
  
  /**
   * @brief Set the confidence sensor for reading confidence scores.
   * @param sensor Pointer to the sensor component
   */
  void set_confidence_sensor(sensor::Sensor *sensor) { confidence_sensor_ = sensor; }
  
  void set_crop_zones(const std::string &zones_json);
  void set_camera_image_format(int width, int height, const std::string &pixel_format);
  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }
  void set_model_config(const std::string &model_type);
  
  /**
   * @brief Set crop zones from global string variable.
   * @param zones_str String containing crop zones configuration
   */
  void set_crop_zones_global_string(const std::string &zones_str) {
    crop_zone_handler_.set_global_zones_string(zones_str);
  }

  /**
   * @brief Print debug information about component state and memory usage.
   */
  void print_debug_info();
  
  /**
   * @brief Report current memory status and usage statistics.
   */
  void report_memory_status();
  
  /**
   * @brief Get peak memory usage from tensor arena.
   * @return Peak memory usage in bytes
   */
  // size_t get_arena_peak_bytes() const;
  
  /**
   * @brief Get the number of frames processed.
   * @return Number of frames processed
   */
  uint32_t get_frames_processed() const { return frames_processed_; }
  
  /**
   * @brief Get the number of frames skipped.
   * @return Number of frames skipped
   */
  uint32_t get_frames_skipped() const { return frames_skipped_; }
  
  /**
   * @brief Get the tensor arena peak usage.
   * @return Peak memory usage in bytes
   */
  size_t get_arena_peak_bytes() const { return model_handler_.get_arena_peak_bytes(); }
  
  /**
   * @brief Register debug service for external debugging.
   * @param comp Pointer to this component instance
   */
  static void register_service(MeterReaderTFLite *comp) {
    comp->print_debug_info();
  }
  
  /**
   * @brief Combine individual digit readings into final value.
   * @param readings Vector of individual digit readings
   * @return Combined meter reading value
   */
  float combine_readings(const std::vector<float> &readings);
  
  
  /**
   * @brief Get the last meter reading value.
   * @return Last meter reading value
   */
  float get_last_reading() const { return last_reading_; }
  
  /**
   * @brief Get the last confidence value.
   * @return Last confidence value (0.0 to 1.0)
   */
  float get_last_confidence() const { return last_confidence_; }
  
  // Model information getters
  int get_model_input_width() const { return model_handler_.get_input_width(); }
  int get_model_input_height() const { return model_handler_.get_input_height(); }
  int get_model_input_channels() const { return model_handler_.get_input_channels(); } 
  

  
#ifdef DEBUG_METER_READER_TFLITE
  void set_debug_image(const uint8_t* data, size_t size);
  void test_with_debug_image();
  void test_with_debug_image_all_configs();
  void set_debug_mode(bool debug_mode);
  void debug_test_with_pattern();
#endif

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
  
  

  // Configuration parameters
  int camera_width_{0};                      ///< Camera image width in pixels
  int camera_height_{0};                     ///< Camera image height in pixels
  std::string pixel_format_{"RGB888"};       ///< Camera pixel format
  float confidence_threshold_{0.7f};         ///< Minimum confidence threshold for valid readings
  size_t tensor_arena_size_requested_{50 * 1024};  ///< Requested tensor arena size
  std::string model_type_{"default"};        ///< Model type identifier

  // State variables
  size_t tensor_arena_size_actual_{0};       ///< Actual allocated tensor arena size
  bool model_loaded_{false};                 ///< Model loading status flag
  const uint8_t *model_{nullptr};            ///< Pointer to model data
  size_t model_length_{0};                   ///< Size of model data in bytes
  sensor::Sensor *value_sensor_{nullptr};    ///< Sensor for meter values
  sensor::Sensor *confidence_sensor_{nullptr};
  
  float last_reading_{0.0f};
  float last_confidence_{0.0f};
  
  
  esp32_camera::ESP32Camera *camera_{nullptr}; ///< Camera component reference
  bool debug_mode_ = false;                  ///< Debug mode flag

  // Component instances
  MemoryManager memory_manager_;             ///< Memory management utilities
  ModelHandler model_handler_;               ///< TFLite model handling
  std::unique_ptr<ImageProcessor> image_processor_;  ///< Image processing utilities
  CropZoneHandler crop_zone_handler_;        ///< Crop zone management
  MemoryManager::AllocationResult tensor_arena_allocation_;  ///< Tensor arena allocation result

 private:
  std::shared_ptr<camera::CameraImage> pending_frame_{nullptr};  ///< Single frame buffer
  std::atomic<bool> frame_available_{false};    ///< Flag indicating frame available for processing
  std::atomic<bool> processing_frame_{false};   ///< Flag indicating frame processing in progress
  std::atomic<bool> frame_requested_{false};    ///< Flag indicating frame request pending
  uint32_t last_frame_received_{0};             ///< Timestamp of last received frame
  uint32_t last_request_time_{0};               ///< Timestamp of last frame request
  // GlobalVarComponent<std::string> *crop_zones_global_{nullptr};
  
  /**
   * @brief Process the next available frame in the buffer.
   */
  void process_available_frame();
};

}  // namespace meter_reader_tflite
}  // namespace esphome