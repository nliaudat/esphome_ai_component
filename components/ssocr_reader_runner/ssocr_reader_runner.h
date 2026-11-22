#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SSOCR_READER_RUNNER

#include "esphome/components/camera/camera.h"
#include "esphome/components/camera/decoder.h"
#include "esphome/components/seven_segment_ocr/seven_segment_ocr.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "flashlight_coordinator.h"
#include "value_validator_coordinator.h"

#include <cstdint>
#include <memory>

namespace esphome {
namespace ssocr_reader_runner {

/// Reads a seven-segment display out of esp32_camera frames.
///
/// The upstream `ssocr_reader` decodes through `camera_decoder` and reads the
/// digits with `seven_segment_ocr`. This local counterpart keeps the optional
/// `value_validator` and `flash_light_controller` integration and publishes
/// through inline sensors.
class SSOCRReaderRunner final : public PollingComponent, public esphome::camera::CameraListener {
 public:
  SSOCRReaderRunner(esphome::camera::Camera *camera, esphome::camera::Decoder *decoder, int frame_width,
                    int frame_height, uint8_t *crop)
      : camera_(camera), decoder_(decoder), img_width_(frame_width), img_height_(frame_height), crop_(crop) {}
  ~SSOCRReaderRunner() {
    if (this->owns_crop_)
      free(this->crop_);
  }

  void setup() override;
  void update() override;
  void loop() override;
  void on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *s) { this->value_sensor_ = s; }
  void set_confidence_sensor(sensor::Sensor *s) { this->confidence_sensor_ = s; }
  void set_debug(bool debug) {
    this->debug_ = debug;
    this->flashlight_coord_.set_debug(debug);
  }
  void set_threshold_level(uint8_t level) { this->ocr_.set_threshold_level(level); }
  void set_crop(int x, int y, int width, int height) {
    this->crop_x_ = x;
    this->crop_y_ = y;
    this->crop_w_ = width;
    this->crop_h_ = height;
  }
  void set_digit_count(uint8_t count) { this->digit_count_ = count; }

#ifdef USE_VALUE_VALIDATOR
  void set_validator(value_validator::ValueValidator *v) { this->validation_coord_.set_validator(v); }
#endif
#ifdef USE_FLASH_LIGHT_CONTROLLER
  void set_flash_light_controller(flash_light_controller::FlashLightController *controller) {
    this->flash_controller_ = controller;
  }
#endif

 protected:
  static constexpr uint32_t FRAME_TIMEOUT_ID = 0;
  static constexpr uint32_t FRAME_TIMEOUT_MS = 5000;

  /// Ask the camera for a frame; also used by the flashlight scheduler.
  void request_frame_();
  void process_frame_(const std::shared_ptr<esphome::camera::CameraImage> &image);
  /// Copies the configured rectangle out of a decoded frame as one byte per pixel.
  bool crop_frame_(const uint8_t *src, const camera::CameraImageSpec &spec);

  esphome::camera::Camera *camera_{nullptr};
  esphome::camera::Decoder *decoder_{nullptr};
  int img_width_{0};
  int img_height_{0};
  uint8_t *crop_{nullptr};
  bool owns_crop_{false};
  seven_segment_ocr::SevenSegmentOCR ocr_;
  int crop_x_{0};
  int crop_y_{0};
  int crop_w_{0};
  int crop_h_{0};
  uint8_t digit_count_{6};
  bool debug_{false};
  bool frame_requested_{false};
  std::shared_ptr<esphome::camera::CameraImage> pending_frame_{nullptr};

  FlashlightCoordinator flashlight_coord_;
  ValueValidatorCoordinator validation_coord_;
  sensor::Sensor *value_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
#ifdef USE_FLASH_LIGHT_CONTROLLER
  flash_light_controller::FlashLightController *flash_controller_{nullptr};
#endif
};

}  // namespace ssocr_reader_runner
}  // namespace esphome

#endif  // USE_SSOCR_READER_RUNNER
