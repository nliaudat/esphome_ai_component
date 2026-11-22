#include "ssocr_reader_runner.h"

#include "esphome/components/camera_decoder/jpeg_grayscale.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cstring>

namespace esphome {
namespace ssocr_reader_runner {

static const char *const TAG = "ssocr_reader_runner";

void SSOCRReaderRunner::setup() {
  if (this->crop_ == nullptr || !this->ocr_.reserve(this->crop_w_)) {
    ESP_LOGE(TAG, "Could not allocate the OCR buffers");
    this->mark_failed();
    return;
  }
  this->camera_->add_listener(this);
  this->disable_loop();  // loop only runs when a frame is pending

#ifdef USE_FLASH_LIGHT_CONTROLLER
  this->flashlight_coord_.setup(this, nullptr, this->flash_controller_);
#else
  this->flashlight_coord_.setup(this, nullptr, nullptr);
#endif
  this->flashlight_coord_.set_update_interval(this->get_update_interval());
  // Called by the flash controller once the light has warmed up: request the lit frame.
  this->flashlight_coord_.set_request_frame_callback([this]() { this->request_frame_(); });
}

void SSOCRReaderRunner::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SSOCR Reader Runner:\n"
                "  Frame: %dx%d\n"
                "  Crop: x=%d, y=%d, w=%d, h=%d\n"
                "  Threshold: %u, Digits: %u\n"
                "  Update Interval: %" PRIu32 " ms\n"
                "  Flash: %s",
                this->img_width_, this->img_height_, this->crop_x_, this->crop_y_, this->crop_w_, this->crop_h_,
                this->ocr_.get_threshold_level(), this->digit_count_, this->get_update_interval(),
                this->flashlight_coord_.is_active() ? "ACTIVE" : "OFF");
  this->decoder_->dump_config();
}

void SSOCRReaderRunner::request_frame_() {
  if (this->frame_requested_)
    return;
  this->frame_requested_ = true;
  this->camera_->request_image(camera::IDLE);
  ESP_LOGD(TAG, "Requesting frame");
  this->set_timeout(FRAME_TIMEOUT_ID, FRAME_TIMEOUT_MS, [this]() {
    this->frame_requested_ = false;
    ESP_LOGW(TAG, "Frame timeout");
  });
}

void SSOCRReaderRunner::update() {
  if (this->frame_requested_)
    return;
  // The flash controller owns the timing when it is configured; it calls back
  // once the light is warm, which is when the frame should be requested.
  this->flashlight_coord_.set_update_interval(this->get_update_interval());
  if (this->flashlight_coord_.update_scheduling())
    return;
  this->request_frame_();
}

void SSOCRReaderRunner::on_camera_image(const std::shared_ptr<esphome::camera::CameraImage> &image) {
  if (!this->frame_requested_)
    return;
  this->frame_requested_ = false;
  this->cancel_timeout(FRAME_TIMEOUT_ID);
  this->pending_frame_ = image;
  this->enable_loop();
}

void SSOCRReaderRunner::loop() {
  std::shared_ptr<esphome::camera::CameraImage> frame = std::move(this->pending_frame_);
  this->disable_loop();
  if (frame) {
    this->process_frame_(frame);
  }
}

void SSOCRReaderRunner::process_frame_(const std::shared_ptr<esphome::camera::CameraImage> &image) {
  if (image->get_data_length() == 0)
    return;

  camera::CameraImageSpec spec{};
  const camera::DecoderError error = this->decoder_->decode(image->get_data_buffer(), image->get_data_length(), &spec);
  if (error != camera::DECODER_ERROR_SUCCESS) {
    ESP_LOGW(TAG, "JPEG decode failed: %s", camera::to_string(error));
    return;
  }

  camera::Buffer *decoded = this->decoder_->get_output_buffer();
  if (decoded == nullptr || decoded->get_data_buffer() == nullptr) {
    ESP_LOGW(TAG, "Decoder returned no output buffer");
    return;
  }

  // A camera_window may shrink the frame; a larger one would break the configured crop.
  if (spec.width > this->img_width_ || spec.height > this->img_height_) {
    ESP_LOGW(TAG, "Decoded dimensions larger than config: %dx%d vs %dx%d", spec.width, spec.height, this->img_width_,
             this->img_height_);
    return;
  }

  if (!this->crop_frame_(decoded->get_data_buffer(), spec)) {
    ESP_LOGE(TAG, "Could not crop %dx%d at %d,%d from the %dx%d frame", this->crop_w_, this->crop_h_, this->crop_x_,
             this->crop_y_, spec.width, spec.height);
    return;
  }

  // The engine counts every run, so a crop that contains more digits than configured is rejected.
  const seven_segment_ocr::OcrResult result =
      this->ocr_.recognize(this->crop_, this->crop_w_, this->crop_h_, this->digit_count_);
  if (!result.complete()) {
    ESP_LOGW(TAG, "Could not read %u digits (found %u, unknown %u, overflow %s)", this->digit_count_, result.digits,
             result.unknown, result.overflow ? "yes" : "no");
    return;
  }

  ESP_LOGV(TAG, "Read value=%" PRIu32 " (confidence %.2f)", result.value, result.confidence);

  // The local validator vets the whole reading; on rejection keep the last good
  // value rather than publishing an unvalidated one.
  int validated = static_cast<int>(result.value);
  if (!this->validation_coord_.validate_reading(static_cast<int>(result.value), result.confidence, validated)) {
    ESP_LOGW(TAG, "Reading %" PRIu32 " rejected by the validator", result.value);
    return;
  }

  PUBLISH_STATE(this->value_sensor_, static_cast<float>(validated));
  PUBLISH_STATE(this->confidence_sensor_, result.confidence * 100.0f);
}

bool SSOCRReaderRunner::crop_frame_(const uint8_t *src, const camera::CameraImageSpec &spec) {
  if (src == nullptr || this->crop_ == nullptr)
    return false;
  if (this->crop_x_ + this->crop_w_ > spec.width || this->crop_y_ + this->crop_h_ > spec.height)
    return false;

  const bool source_is_gray = spec.format == camera::PIXEL_FORMAT_GRAYSCALE;
  if (!source_is_gray && spec.format != camera::PIXEL_FORMAT_RGB888)
    return false;

  const size_t source_bpp = source_is_gray ? 1 : camera_decoder::RGB888_BYTES_PER_PIXEL;
  const size_t source_stride = source_bpp * spec.width;
  const uint8_t *source_row =
      src + static_cast<size_t>(this->crop_y_) * source_stride + static_cast<size_t>(this->crop_x_) * source_bpp;
  for (int y = 0; y < this->crop_h_; y++) {
    const uint8_t *row = source_row + static_cast<size_t>(y) * source_stride;
    uint8_t *dst = this->crop_ + static_cast<size_t>(y) * this->crop_w_;
    if (source_is_gray) {
      std::memcpy(dst, row, this->crop_w_);
      continue;
    }
    for (int x = 0; x < this->crop_w_; x++) {
      const uint8_t *pixel = row + static_cast<size_t>(x) * source_bpp;
      dst[x] = static_cast<uint8_t>((pixel[0] * camera_decoder::LUMA_RED + pixel[1] * camera_decoder::LUMA_GREEN +
                                     pixel[2] * camera_decoder::LUMA_BLUE) >>
                                    8);
    }
  }
  return true;
}

}  // namespace ssocr_reader_runner
}  // namespace esphome
