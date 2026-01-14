#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "camera_coordinator.h"
#include "flashlight_coordinator.h" 
#include "value_validator_coordinator.h"
#include <vector>

namespace esphome {
namespace analog_reader {

// Exported constants for algorithm implementations
extern const float kScanStartRadius;
extern const float kScanEndRadius;
extern const float kIntensityWeight;
extern const float kEdgeWeight;

enum NeedleType {
  NEEDLE_TYPE_DARK = 0,
  NEEDLE_TYPE_LIGHT = 1,
};

struct DialConfig {
  std::string id;
  NeedleType needle_type{NEEDLE_TYPE_DARK};
  std::string algorithm{"radial_profile"};  // Algorithm: radial_profile, hough_transform, template_match, auto
  float scale{1.0f};
  int crop_x{0};
  int crop_y{0};
  int crop_w{0};
  int crop_h{0};
  float min_angle{0.0f};   // Degrees
  float max_angle{360.0f}; // Degrees
  float angle_offset{0.0f}; // 0 = North, 90 = East
  float min_value{0.0f};
  float max_value{10.0f};
  bool auto_contrast{true}; // Normalization (Min-Max Stretch)
  float contrast{1.0f};      // Multiplier (1.0 = original)
  uint32_t target_color{0};  // RGB hex
  bool use_color{false};
};

class AnalogReader : public PollingComponent, public esphome::camera::CameraListener {
 public:
  ~AnalogReader();
  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { value_sensor_ = s; }
  void set_validator(value_validator::ValueValidator *v) { validation_coord_.set_validator(v); }
  
  void set_camera(esphome::camera::Camera *camera) { camera_ = camera; }
  void set_camera_image_format(int width, int height, const std::string &format) {
      img_width_ = width;
      img_height_ = height;
      pixel_format_str_ = format;
  }

  void set_pause_processing(bool paused) { paused_ = paused; }
  void set_debug(bool debug) { debug_ = debug; }
  
  void set_all_auto_contrast(bool enabled) {
      for (auto &dial : dials_) dial.auto_contrast = enabled;
  }
  
  void set_all_contrast(float contrast) {
      for (auto &dial : dials_) dial.contrast = contrast;
  }
  
  void set_all_algorithm(const std::string &algorithm) {
      for (auto &dial : dials_) dial.algorithm = algorithm;
  }
  
  void set_update_interval(uint32_t interval) override;

  void add_dial(DialConfig config) { dials_.push_back(config); }
  
  // Services
  void set_dial_range(std::string dial_id, float min_val, float max_val);
  void set_dial_angle(std::string dial_id, float min_deg, float max_deg);

 protected:
  bool paused_{false};
  // Coordinators
  // Note: We reuse CameraCoordinator from ssocr_reader/meter_reader context
  // Assuming it's available in include path (it is in same 'components' root usually or library)
  // But wait, ssocr_reader includes "camera_coordinator.h". We need to ensure we can link it.
  // We will add it to __init__.py libraries.
  CameraCoordinator camera_coord_;
  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;

  esphome::camera::Camera *camera_{nullptr};
  bool capture_next_{false};
  int img_width_{0}; 
  int img_height_{0}; 
  std::string pixel_format_str_{"JPEG"};
  
  sensor::Sensor *value_sensor_{nullptr};
  std::vector<DialConfig> dials_;
  
  // State
  bool processing_frame_{false};
  bool debug_{false};
  bool frame_requested_{false};
  uint32_t last_request_time_{0};
  
  // Async processing
  std::shared_ptr<esphome::camera::CameraImage> pending_frame_{nullptr};

  // Optimization: Pre-allocated buffer and LUTs
  std::vector<uint8_t> working_buffer_;
  std::vector<uint8_t> scratch_buffer_;
  
  // Persistent RGB buffer (Manual allocation for PSRAM control)
  uint8_t* rgb_buffer_{nullptr};
  size_t rgb_buffer_size_{0};
  bool requires_color_{false};
  
  static float sin_lut_[360];
  static float cos_lut_[360];

  void process_image(std::shared_ptr<esphome::camera::CameraImage> image);
  void process_image_from_buffer(const uint8_t* data, size_t len);
  float find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial);
  
  // Detection algorithms
  struct DetectionResult {
      float angle;
      float confidence;
      std::string algorithm;
  };
  
  DetectionResult detect_legacy(const uint8_t* img, int w, int h, const DialConfig& dial);  // Original algorithm, no preprocessing
  DetectionResult detect_radial_profile(const uint8_t* img, int w, int h, const DialConfig& dial);
  DetectionResult detect_hough_transform(const uint8_t* img, int w, int h, const DialConfig& dial);
  DetectionResult detect_template_match(const uint8_t* img, int w, int h, const DialConfig& dial);
  
  // Preprocessing
  void preprocess_image(const uint8_t* img, int w, int h, int cx, int cy, int radius, NeedleType needle_type, std::vector<uint8_t>& output);
  void apply_clahe(uint8_t* img, int w, int h, int tile_size = 8);
  void remove_background(uint8_t* img, int w, int h, int cx, int cy, int radius);
  void apply_tophat(uint8_t* img, int w, int h, int kernel_size, std::vector<uint8_t>& scratch, NeedleType needle_type); // Top-hat transform for shadow removal
  void median_filter_3x3(uint8_t* img, int w, int h);
  
  // Helpers
  float angle_to_value(float angle, const DialConfig& dial);
  void debug_dial_image(const uint8_t* img, int w, int h, float detected_angle);
  void debug_angle_calculation(float image_angle, const DialConfig& dial);
};

}  // namespace analog_reader
}  // namespace esphome
