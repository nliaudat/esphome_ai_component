/**
 * @file image_processor.h
 * @brief Image processing utilities for cropping, scaling, and format conversion.
 * 
 * Handles JPEG decoding with optional rotation (0°, 90°, 180°, 270°),
 * raw format processing (RGB888, RGB565, Grayscale), and conversion to
 * TensorFlow Lite model input formats (float32 or uint8).
 */
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "crop_zone_handler.h"
#include "esp_jpeg_dec.h"

namespace esphome {
namespace esp32_camera_utils {

enum ImageProcessorInputType {
    kInputTypeFloat32,
    kInputTypeUInt8,
    kInputTypeUnknown
};

// Image rotation options (clockwise)
enum ImageRotation {
    ROTATION_0 = 0,      // No rotation
    ROTATION_90 = 90,    // 90° clockwise
    ROTATION_180 = 180,  // 180°
    ROTATION_270 = 270   // 270° clockwise (or 90° counter-clockwise)
};

struct ImageProcessorConfig {
  int camera_width;
  int camera_height;
  std::string pixel_format;
  ImageRotation rotation{ROTATION_0};  // Image rotation (clockwise)
  
  int model_width;
  int model_height;
  int model_channels;
  ImageProcessorInputType input_type;
  bool normalize; // For float32 conversion
  std::string input_order{"RGB"}; // "RGB" or "BGR"
  
  bool validate() const {
    return camera_width > 0 && camera_height > 0 && !pixel_format.empty() &&
           model_width > 0 && model_height > 0 && model_channels > 0;
  }
};

struct ProcessingStats {
  uint32_t total_frames{0};
  uint32_t failed_frames{0};
  uint32_t total_processing_time_ms{0};
  uint32_t jpeg_decoding_errors{0};
  
  float get_avg_processing_time() const {
    return total_frames > 0 ? (float)total_processing_time_ms / total_frames : 0.0f;
  }
  
  float get_success_rate() const {
    return total_frames > 0 ? 100.0f * (1.0f - (float)failed_frames / total_frames) : 0.0f;
  }
};

class ImageProcessor {
 public:
  // Custom deleter for tracked buffers
  struct TrackedBuffer {
      uint8_t* ptr;
      bool is_spiram;
      bool is_jpeg_aligned;
      
      TrackedBuffer(uint8_t* p = nullptr, bool spiram = false, bool aligned = false) 
          : ptr(p), is_spiram(spiram), is_jpeg_aligned(aligned) {}
          
      ~TrackedBuffer() {
          if (ptr) {
              if (is_jpeg_aligned) {
                  jpeg_free_align(ptr);
              } else if (is_spiram) {
                  heap_caps_free(ptr);
              } else {
                  delete[] ptr;
              }
          }
      }
      
      // Prevent copying
      TrackedBuffer(const TrackedBuffer&) = delete;
      TrackedBuffer& operator=(const TrackedBuffer&) = delete;
      
      // Allow moving
      TrackedBuffer(TrackedBuffer&& other) noexcept 
          : ptr(other.ptr), is_spiram(other.is_spiram), is_jpeg_aligned(other.is_jpeg_aligned) {
          other.ptr = nullptr;
      }
      
      TrackedBuffer& operator=(TrackedBuffer&& other) noexcept {
          if (this != &other) {
              if (ptr) {
                  if (is_jpeg_aligned) jpeg_free_align(ptr);
                  else if (is_spiram) heap_caps_free(ptr);
                  else delete[] ptr;
              }
              ptr = other.ptr;
              is_spiram = other.is_spiram;
              is_jpeg_aligned = other.is_jpeg_aligned;
              other.ptr = nullptr;
          }
          return *this;
      }
      
      uint8_t* get() const { return ptr; }
  };

  using UniqueBufferPtr = std::unique_ptr<TrackedBuffer>;

  struct ProcessResult {
    UniqueBufferPtr data;
    size_t size{0};
  };

  ImageProcessor(const ImageProcessorConfig &config);

  std::vector<ProcessResult> split_image_in_zone(
      std::shared_ptr<camera::CameraImage> image,
      const std::vector<CropZone> &zones);
      
  // New method for direct processing to pre-allocated buffer (zero allocation during inference)
  bool process_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);

  const ProcessingStats& get_stats() const { return stats_; }
  void reset_stats() { stats_ = ProcessingStats(); }
  
  // Helper to calculate required buffer size
  size_t get_required_buffer_size() const;

 private:
  ImageProcessorConfig config_;
  ProcessingStats stats_;
  std::mutex processing_mutex_;
  int bytes_per_pixel_{3};

  UniqueBufferPtr allocate_image_buffer(size_t size);
  
  ProcessResult process_zone(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone);
      
  bool validate_input_image(std::shared_ptr<camera::CameraImage> image) const;
  bool validate_zone(const CropZone &zone) const;
  bool validate_buffer_size(size_t required, size_t available, const char* context) const;

  // Raw processing methods
  bool process_raw_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);

  // JPEG processing methods
  bool process_jpeg_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);
      
  const char* jpeg_error_to_string(jpeg_error_t error) const;

  // Format conversion helpers
  bool process_rgb888_crop_and_scale_to_float32(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels, bool normalize);
      
  bool process_rgb888_crop_and_scale_to_uint8(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels);
      
  bool process_rgb565_crop_and_scale_to_float32(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels, bool normalize);
      
  bool process_rgb565_crop_and_scale_to_uint8(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels);
      
  bool process_grayscale_crop_and_scale_to_float32(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels, bool normalize);
      
  bool process_grayscale_crop_and_scale_to_uint8(
      const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
      uint8_t* output_buffer, int model_width, int model_height, int channels);

  // Scaling helpers
  bool scale_rgb888_to_float32(
      const uint8_t* src, int src_w, int src_h,
      uint8_t* dst, int dst_w, int dst_h, int channels, bool normalize);
      
  bool scale_rgb888_to_uint8(
      const uint8_t* src, int src_w, int src_h,
      uint8_t* dst, int dst_w, int dst_h, int channels);

  // Channel arrangement helpers
  void arrange_channels(float* output, uint8_t r, uint8_t g, uint8_t b, int output_channels, bool normalize) const;
  void arrange_channels(uint8_t* output, uint8_t r, uint8_t g, uint8_t b, int output_channels) const;

      
  // Software rotation for raw formats (when JPEG rotation not available)
  bool apply_software_rotation(
      const uint8_t* input, uint8_t* output,
      int width, int height, int bytes_per_pixel,
      ImageRotation rotation);

#ifdef DEBUG_METER_READER_TFLITE
  // Debug functions for image analysis
  void debug_log_image_stats(const uint8_t* data, size_t size, const std::string& stage);
  void debug_log_float_stats(const float* data, size_t count, const std::string& stage);
  void debug_log_image(const uint8_t* data, size_t size, int width, int height, int channels, const std::string& stage);
  void debug_log_float_image(const float* data, size_t count, int width, int height, int channels, const std::string& stage);
  void debug_log_rgb888_image(const uint8_t* data, int width, int height, const std::string& stage);
  void debug_analyze_processed_zone(const uint8_t* data, int width, int height, int channels, const std::string& zone_name);
  void debug_analyze_float_zone(const float* data, int width, int height, int channels, const std::string& zone_name, bool normalized);
  void debug_output_zone_preview(const uint8_t* data, int width, int height, int channels, const std::string& zone_name);
  void debug_output_float_preview(const float* data, int width, int height, int channels, const std::string& zone_name, bool normalized);
#endif
};

}  // namespace esp32_camera_utils
}  // namespace esphome
