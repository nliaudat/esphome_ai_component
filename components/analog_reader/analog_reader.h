#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

#include "flashlight_coordinator.h" 
#include "value_validator_coordinator.h"
#include <vector>
#include <mutex>
#include <atomic>

#include "esphome/core/defines.h"

#ifdef USE_ANALOG_READER

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

enum ProcessChannel {
  PROCESS_CHANNEL_GRAYSCALE = 0,
  PROCESS_CHANNEL_RED = 1,
  PROCESS_CHANNEL_GREEN = 2,
  PROCESS_CHANNEL_BLUE = 3,
};

struct DialConfig {
  std::string id = "";
  NeedleType needle_type = NEEDLE_TYPE_DARK;
  std::string algorithm = "radial_profile";  // Algorithm: radial_profile, hough_transform, template_match, auto
  float scale = 1.0f;
  int crop_x = 0;
  int crop_y = 0;
  int crop_w = 0;
  int crop_h = 0;
  float min_angle = 0.0f;   // Degrees
  float max_angle = 360.0f; // Degrees
  float angle_offset = 0.0f; // 0 = North, 90 = East
  bool clockwise = true;     // Needle sweep direction (false = counter-clockwise)
  float min_value = 0.0f;
  float max_value = 10.0f;
  bool auto_contrast = true; // Normalization (Min-Max Stretch)
  float contrast = 1.0f;      // Multiplier (1.0 = original)
  uint32_t target_color = 0;  // RGB hex
  bool use_color = false;
  // Color match tolerance (0 = legacy 0-442 linear map). When >0, the colour-distance
  // map is thresholded: pixels farther than this are pushed to flat background, which
  // removes off-colour features (e.g. black digits) that otherwise compete with the needle.
  float color_tolerance = 0.0f;
  ProcessChannel process_channel = PROCESS_CHANNEL_GRAYSCALE;
  float min_scan_radius = 0.3f; // % of radius (0.0-1.0)
  float max_scan_radius = 0.9f; // % of radius (0.0-1.0)
  float deadzone_diameter = 0.0f; // Pixels (center circle ignored by detection)
  
  // Sensors
  sensor::Sensor *value_sensor = nullptr;
  sensor::Sensor *confidence_sensor = nullptr;
  sensor::Sensor *angle_sensor = nullptr;
  
  std::vector<std::pair<float, float>> calibration_mapping; 
};

// Per-dial measured value — used for stacked-digit combination
struct DialReading { const DialConfig* dial; float value; };

class AnalogReader : public PollingComponent, public esphome::camera::CameraListener {
 public:
  ~AnalogReader();
  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { this->value_sensor_ = s; }
#ifdef USE_VALUE_VALIDATOR
  void set_validator(value_validator::ValueValidator *v) { this->validation_coord_.set_validator(v); }
#endif
  
  void set_camera(esphome::camera::Camera *camera) { this->camera_ = camera; }
  void set_camera_image_format(int width, int height, const std::string &format) {
      this->img_width_ = width;
      this->img_height_ = height;
      this->pixel_format_str_ = format;
  }

  void set_pause_processing(bool paused) { this->paused_ = paused; }
  void set_debug(bool debug) { 
      this->debug_ = debug; 
      this->flashlight_coord_.set_debug(debug);
  }
  
  void set_all_auto_contrast(bool enabled) {
      for (auto &dial : this->dials_) dial.auto_contrast = enabled;
  }
  
  void set_all_contrast(float contrast) {
      for (auto &dial : this->dials_) dial.contrast = contrast;
  }
  
  void set_all_algorithm(const std::string &algorithm) {
      for (auto &dial : this->dials_) dial.algorithm = algorithm;
  }
  
  // When enabled, dials are combined as positional digits (odometer-style) with
  // neighbour carry-correction instead of a weighted sum of raw needle values.
  void set_stacked_digits(bool enabled) { this->stacked_digits_ = enabled; }
  
  void add_dial(DialConfig config) { this->dials_.push_back(config); }
  
  void set_dial_range(const std::string &dial_id, float min_val, float max_val);
  void set_dial_angle(const std::string &dial_id, float min_deg, float max_deg);

 protected:
  // Single atomic state machine — replaces separate processing_frame_ and frame_requested_ bools
  enum class FrameState : uint8_t {
    IDLE,
    REQUESTED,
    AVAILABLE,
    PROCESSING,
    TIMEOUT
  };
  std::atomic<FrameState> frame_state_{FrameState::IDLE};
  uint32_t last_request_time_{0};

  struct DetectionResult {
      float angle;
      float confidence;
      std::string algorithm;
  };

  bool paused_{false};
  bool stacked_digits_{false};
  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;

  esphome::camera::Camera *camera_{nullptr};
  int img_width_{0}; 
  int img_height_{0}; 
  std::string pixel_format_str_{"JPEG"};
  
  sensor::Sensor *value_sensor_{nullptr};
  std::vector<DialConfig> dials_;
  
  bool debug_{false};
  
  // Async processing
  std::shared_ptr<esphome::camera::CameraImage> pending_frame_{nullptr};
  std::mutex frame_mutex_;

  // Optimization: Pre-allocated buffer and scratch spaces
  std::vector<uint8_t> working_buffer_;
  std::vector<uint8_t> scratch_buffer_;
  std::vector<uint8_t> scratch_buffer_2_;

  // Pre-allocated per-frame readings vector — avoids heap alloc in loop()
  std::vector<DialReading> readings_;
  
  // Persistent Buffer (RAII managed)
  struct FreeDeleter {
      void operator()(uint8_t* ptr) const { free(ptr); }
  };
  std::unique_ptr<uint8_t[], FreeDeleter> persistent_buffer_{nullptr};
  size_t persistent_buffer_size_{0};
  bool requires_color_{false};
  pixformat_t buffer_format_{PIXFORMAT_RGB888};
  
  static float sin_lut_[360];
  static float cos_lut_[360];

  void process_image(std::shared_ptr<esphome::camera::CameraImage> image);
  void process_image_from_buffer(const uint8_t* data, size_t len);
  [[nodiscard]] DetectionResult find_needle_angle(const uint8_t* img, int w, int h, const DialConfig& dial);
  
  DetectionResult detect_legacy(const uint8_t* img, int w, int h, const DialConfig& dial);
  DetectionResult detect_radial_profile(const uint8_t* img, int w, int h, const DialConfig& dial);
  DetectionResult detect_hough_transform(const uint8_t* img, int w, int h, const DialConfig& dial);
  DetectionResult detect_template_match(const uint8_t* img, int w, int h, const DialConfig& dial);
  
  void preprocess_image(const uint8_t* img, int w, int h, int cx, int cy, int radius, NeedleType needle_type, std::vector<uint8_t>& output);
  void apply_clahe(uint8_t* img, int w, int h, int tile_size = 8);
  void remove_background(uint8_t* img, int w, int h, int cx, int cy, int radius);
  void apply_tophat(uint8_t* img, int w, int h, int kernel_size, std::vector<uint8_t>& scratch, std::vector<uint8_t>& scratch2, NeedleType needle_type);
  void median_filter_3x3(uint8_t* img, int w, int h);
  
  [[nodiscard]] float angle_to_value(float angle, const DialConfig& dial);
  void debug_dial_image(const uint8_t* img, int w, int h, float detected_angle);
  void debug_angle_calculation(float image_angle, const DialConfig& dial);
};

}  // namespace analog_reader
}  // namespace esphome

#endif  // USE_ANALOG_READER