/**
 * @file image_processor.h
 * @brief Image processing utilities for cropping, scaling, and format conversion.
 * 
 * Handles JPEG decoding with optional rotation (0 deg, 90 deg, 180 deg, 270 deg),
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
#include "drawing_utils.h"
#include "scaler.h"
#include "rotator.h"
#include "cropper.h"
#include "buffer_pool.h"
#include "esp_heap_caps.h"

namespace esphome {
namespace esp32_camera_utils {

enum ImageProcessorInputType {
    kInputTypeFloat32,
    kInputTypeUInt8,
    kInputTypeUnknown
};

// Rotation constants
constexpr float ROTATION_0 = 0.0f;
constexpr float ROTATION_90 = 90.0f;
constexpr float ROTATION_180 = 180.0f;
constexpr float ROTATION_270 = 270.0f;

struct ImageProcessorConfig {
  int camera_width;
  int camera_height;
  std::string pixel_format; // "RGB888", "RGB565", "GRAYSCALE", "JPEG"
  
  float rotation{0.0f};  // Image rotation in degrees (clockwise)
  
  int model_width;
  int model_height;
  int model_channels;
  ImageProcessorInputType input_type;
  bool normalize; // For float32 conversion
  std::string input_order{"RGB"}; // "RGB" or "BGR"
  
  // Custom module settings
  int scaler_width{0};
  int scaler_height{0};
  int cropper_width{0};
  int cropper_height{0};
  int cropper_offset_x{0};
  int cropper_offset_y{0};
  
  bool cache_preview_image{false}; // Optimization: Only cache master image if needed for preview
  
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
      size_t size;  // Track size for pool release
      bool is_spiram;
      bool is_jpeg_aligned;
      bool is_pooled;  // Track if buffer is from pool
      
      // Static counter for leak detection
      static std::atomic<int32_t> active_instances;

      TrackedBuffer(uint8_t* p = nullptr, bool spiram = false, bool aligned = false, bool pooled = false, size_t sz = 0) 
          : ptr(p), size(sz), is_spiram(spiram), is_jpeg_aligned(aligned), is_pooled(pooled) {
          if (p) {
              active_instances++;
          }
      }
          
      ~TrackedBuffer() {
          if (ptr) {
              active_instances--;
              if (is_pooled) {
                  // Return to buffer pool
                  BufferPool::Buffer buf{ptr, size, true};
                  ImageProcessor::buffer_pool_.release(buf);
              } else if (is_jpeg_aligned) {
                  jpeg_free_align(ptr);
              } else {
                  heap_caps_free(ptr);
              }
          }
      }
      
      
      // Prevent copying
      TrackedBuffer(const TrackedBuffer&) = delete;
      TrackedBuffer& operator=(const TrackedBuffer&) = delete;
      
      // Allow moving
      TrackedBuffer(TrackedBuffer&& other) noexcept 
          : ptr(other.ptr), size(other.size), is_spiram(other.is_spiram), 
            is_jpeg_aligned(other.is_jpeg_aligned), is_pooled(other.is_pooled) {
          // Ownership transferred, no new allocation created
          
          other.ptr = nullptr;
          other.size = 0;
          other.is_spiram = false;
          other.is_jpeg_aligned = false;
          other.is_pooled = false;
      }
      
      TrackedBuffer& operator=(TrackedBuffer&& other) noexcept {
          if (this != &other) {
              // Existing instance being overwritten - count stays same (1 destruction conceptually, 1 creation)
              // Actually, simpler: this object stays alive, just changes ownership.
              // So active_instances count does not change.
              
              if (ptr) {
                  active_instances--;
                  if (is_pooled) {
                      BufferPool::Buffer buf{ptr, size, true};
                      ImageProcessor::buffer_pool_.release(buf);
                  } else if (is_jpeg_aligned) {
                      jpeg_free_align(ptr);
                  } else {
                      heap_caps_free(ptr);
                  }
              }
              ptr = other.ptr;
              size = other.size;
              is_spiram = other.is_spiram;
              is_jpeg_aligned = other.is_jpeg_aligned;
              is_pooled = other.is_pooled;
              
              other.ptr = nullptr;
              other.size = 0;
              other.is_spiram = false;
              other.is_jpeg_aligned = false;
              other.is_pooled = false;
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

  /**
   * Process an image and split it into the requested zones (cropping + resizing).
   */
  std::vector<ProcessResult> split_image_in_zone(
      std::shared_ptr<camera::CameraImage> image,
      const std::vector<CropZone> &zones);

  /**
   * Process a single zone.
   */
  ProcessResult process_zone(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone);

  /**
   * Process a single zone directly into a provided buffer.
   */
  bool process_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);



  // Validate zone boundaries

  bool validate_zone(const CropZone &zone) const;
  
  size_t get_required_buffer_size() const;

#ifdef USE_CAMERA_DRAWING
  // Drawing primitives delegates
  void draw_pixel(uint8_t* buffer, int x, int y, int w, int h, int channels, uint16_t color) {
    DrawingUtils::draw_pixel(buffer, x, y, w, h, channels, color);
  }
  void draw_rectangle(uint8_t* buffer, int x, int y, int w, int h, int img_w, int img_h, int channels, uint16_t color) {
    DrawingUtils::draw_rectangle(buffer, x, y, w, h, img_w, img_h, channels, color);
  }
  void draw_filled_rectangle(uint8_t* buffer, int x, int y, int w, int h, int img_w, int img_h, int channels, uint16_t color) {
    DrawingUtils::draw_filled_rectangle(buffer, x, y, w, h, img_w, img_h, channels, color);
  }
  void draw_circle(uint8_t* buffer, int cx, int cy, int r, int img_w, int img_h, int channels, uint16_t color) {
    DrawingUtils::draw_circle(buffer, cx, cy, r, img_w, img_h, channels, color);
  }
  void draw_filled_circle(uint8_t* buffer, int cx, int cy, int r, int img_w, int img_h, int channels, uint16_t color) {
    DrawingUtils::draw_filled_circle(buffer, cx, cy, r, img_w, img_h, channels, color);
  }
#endif

private:
  ImageProcessorConfig config_;
  ProcessingStats stats_;
  std::mutex processing_mutex_;
  int bytes_per_pixel_{0};

  // Static buffer pool shared across all ImageProcessor instances
  static BufferPool buffer_pool_;

  // Cached last valid master image (for preview reuse)
  std::shared_ptr<camera::CameraImage> last_processed_image_;

  static UniqueBufferPtr allocate_image_buffer(size_t size);
  bool validate_buffer_size(size_t required, size_t available, const char* context) const;
  bool validate_input_image(std::shared_ptr<camera::CameraImage> image) const;
  
  // Internal processing methods
  bool process_jpeg_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);
      
  bool process_raw_zone_to_buffer(
      std::shared_ptr<camera::CameraImage> image,
      const CropZone &zone,
      uint8_t* output_buffer,
      size_t output_buffer_size);

  // Helper processing methods
  bool process_rgb888_crop_and_scale_to_float32(const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int channels, bool normalize, int src_stride_width);
  bool process_rgb888_crop_and_scale_to_uint8(const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int channels, int src_stride_width);
  
  bool process_rgb565_crop_and_scale_to_float32(const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int model_channels, bool normalize, int src_stride_width);
  bool process_rgb565_crop_and_scale_to_uint8(const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int model_channels, int src_stride_width);
  
  bool process_grayscale_crop_and_scale_to_float32(const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int model_channels, bool normalize, int src_stride_width);
  bool process_grayscale_crop_and_scale_to_uint8(const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height, uint8_t* output_buffer, int model_width, int model_height, int model_channels, int src_stride_width);

  bool scale_rgb888_to_float32(const uint8_t* src, int src_w, int src_h, uint8_t* dst, int dst_w, int dst_h, int channels, bool normalize);
  bool scale_rgb888_to_uint8(const uint8_t* src, int src_w, int src_h, uint8_t* dst, int dst_w, int dst_h, int channels);

  void arrange_channels(float* output, uint8_t r, uint8_t g, uint8_t b, int output_channels, bool normalize) const;

  const char* jpeg_error_to_string(jpeg_error_t error) const;

public:
  /**
   * Helper to parse actual dimensions from a JPEG buffer (SOF0 marker).
   */
  static bool get_jpeg_dimensions(const uint8_t *data, size_t len, int &width, int &height);

#ifdef USE_CAMERA_ROTATOR
  /**
   * Generates a rotated preview image from a source image.
   */
  static std::shared_ptr<camera::CameraImage> generate_rotated_preview(
      std::shared_ptr<camera::CameraImage> source, 
      float rotation, int width, int height);

  /**
   * Get the last fully processed (decoded & rotated) image.
   * Useful for debugging/preview without re-processing.
   */
  std::shared_ptr<camera::CameraImage> get_last_processed_image() const { return last_processed_image_; }
#endif

#if defined(USE_CAMERA_ROTATOR) || defined(DEV_ENABLE_ROTATION)
  // Software rotation for arbitrary angles
  [[nodiscard]] static bool apply_software_rotation(
      const uint8_t* input, uint8_t* output,
      int src_w, int src_h, int bytes_per_pixel,
      float rotation_deg, int& out_w, int& out_h);
#endif

  // Custom deleters for JPEG resources
  struct JpegDecoderDeleter {
    using pointer = jpeg_dec_handle_t;
    void operator()(jpeg_dec_handle_t decoder) const {
      if (decoder) {
        jpeg_dec_close(decoder);
      }
    }
  };

  struct JpegBufferDeleter {
    void operator()(uint8_t* buf) const {
      if (buf) {
        jpeg_free_align(buf);
      }
    }
  };

  using JpegBufferPtr = std::unique_ptr<uint8_t[], JpegBufferDeleter>;

  // Helper to decode JPEG to RGB888 or Gray
  [[nodiscard]] static JpegBufferPtr decode_jpeg(const uint8_t* data, size_t len, int* width, int* height, jpeg_pixel_format_t output_format = JPEG_PIXEL_FORMAT_RGB888);

  // Allocates a buffer suitable for JPEG operations (16-byte aligned)
  [[nodiscard]] static JpegBufferPtr allocate_jpeg_buffer(size_t size);

  void arrange_channels(uint8_t* output, uint8_t r, uint8_t g, uint8_t b, int output_channels) const;

#ifdef DEBUG_ESP32_CAMERA_UTILS
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

#ifdef USE_CAMERA_ROTATOR
// RotatedPreviewImage class moved here for shared usage
class RotatedPreviewImage : public camera::CameraImage {
  using UniqueBufferPtr = esphome::esp32_camera_utils::ImageProcessor::UniqueBufferPtr;
 public:
  RotatedPreviewImage(UniqueBufferPtr &&data, size_t len, int width, int height, pixformat_t format)
      : data_(std::move(data)), len_(len), width_(width), height_(height), format_(format) {}

  uint8_t *get_data_buffer() override { return data_->get(); }
  size_t get_data_length() override { return len_; }
  bool was_requested_by(camera::CameraRequester requester) const override { return true; }
  int get_width() { return width_; }
  int get_height() { return height_; }
  pixformat_t get_format() { return format_; }

 protected:
  UniqueBufferPtr data_;
  size_t len_;
  int width_;
  int height_;
  pixformat_t format_;
};
#endif

}  // namespace esp32_camera_utils
}  // namespace esphome
