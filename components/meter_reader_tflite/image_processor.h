/**
 * @file image_processor.h
 * @brief Image processing utilities for TensorFlow Lite Micro meter reader.
 * 
 * Handles image manipulation including cropping, scaling, and format conversion
 * for TFLite model input preparation.
 */

#pragma once

#include "esphome/components/camera/camera.h"
#include "crop_zones.h"
#include "model_handler.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <set>

// Use the correct include path for esp_new_jpeg
#include "esp_jpeg_dec.h"
#include "esp_jpeg_common.h"

static const char *const TAG = "ImageProcessor";

namespace esphome {
namespace meter_reader_tflite {

/**
 * @struct ImageProcessorConfig
 * @brief Configuration structure for ImageProcessor.
 * 
 * Contains camera dimensions and pixel format information required
 * for proper image processing.
 */
struct ImageProcessorConfig {
  int camera_width;          ///< Width of the camera image in pixels
  int camera_height;         ///< Height of the camera image in pixels
  std::string pixel_format;  ///< Pixel format string (e.g., "RGB888", "JPEG")
  
  /**
   * @brief Validate the configuration parameters.
   * @return true if configuration is valid, false otherwise
   */
  bool validate() const {
    if (camera_width <= 0 || camera_height <= 0) {
      return false;
    }
    
    if (pixel_format.empty()) {
      return false;
    }
    
    static const std::set<std::string> valid_formats = {
      "RGB888", "RGB565", "YUV422", "GRAYSCALE", "JPEG"
    };
    
    return valid_formats.find(pixel_format) != valid_formats.end();
  }
};

/**
 * @struct ProcessingStats
 * @brief Statistics for image processing performance monitoring.
 */
struct ProcessingStats {
  uint32_t total_frames{0};
  uint32_t failed_frames{0};
  uint32_t total_processing_time_ms{0};
  uint32_t peak_memory_usage{0};
  uint32_t jpeg_decoding_errors{0};
  uint32_t memory_allocation_errors{0};
  
  void reset() {
    total_frames = 0;
    failed_frames = 0;
    total_processing_time_ms = 0;
    peak_memory_usage = 0;
    jpeg_decoding_errors = 0;
    memory_allocation_errors = 0;
  }
  
  float get_success_rate() const {
    if (total_frames == 0) return 0.0f;
    return (static_cast<float>(total_frames - failed_frames) / total_frames) * 100.0f;
  }
  
  float get_avg_processing_time() const {
    if (total_frames == 0) return 0.0f;
    return static_cast<float>(total_processing_time_ms) / total_frames;
  }
};

/**
 * @class ImageProcessor
 * @brief Handles image preprocessing tasks for TFLite model input.
 * 
 * This class manages image manipulation including splitting images into zones,
 * JPEG decoding, scaling, and format conversion to meet model requirements.
 */
class ImageProcessor {
 public:
    /**
     * @class TrackedBuffer
     * @brief Smart pointer wrapper for dynamically allocated image buffers.
     * 
     * Manages memory allocated with heap_caps_malloc or new[] with proper
     * cleanup based on allocation type.
     */
    class TrackedBuffer {
    public:
        /**
         * @brief Construct a TrackedBuffer.
         * @param ptr Pointer to allocated memory
         * @param is_spiram True if allocated from SPIRAM
         * @param is_jpeg_aligned True if allocated with jpeg_calloc_align
         */
        TrackedBuffer(uint8_t* ptr, bool is_spiram, bool is_jpeg_aligned = false) 
            : ptr_(ptr), is_spiram_(is_spiram), is_jpeg_aligned_(is_jpeg_aligned) {}
        
        /**
         * @brief Destructor that frees memory appropriately.
         */
        ~TrackedBuffer() {
            if (ptr_) {
                if (is_jpeg_aligned_) {
                    jpeg_free_align(ptr_);
                } else if (is_spiram_) {
                    heap_caps_free(ptr_);
                } else {
                    delete[] ptr_;
                }
            }
        }
        
        uint8_t* get() { return ptr_; }                   ///< Get raw pointer
        const uint8_t* get() const { return ptr_; }       ///< Get const raw pointer
        uint8_t& operator[](size_t idx) { return ptr_[idx]; }     ///< Array access
        const uint8_t& operator[](size_t idx) const { return ptr_[idx]; } ///< Const array access
        
    private:
        uint8_t* ptr_;              ///< Pointer to buffer data
        bool is_spiram_;            ///< SPIRAM allocation flag
        bool is_jpeg_aligned_;      ///< JPEG-aligned allocation flag
    };

    /// Unique pointer type for TrackedBuffer
    using UniqueBufferPtr = std::unique_ptr<TrackedBuffer>;

    /**
     * @struct ProcessResult
     * @brief Result structure for processed image data.
     */
    struct ProcessResult {
        UniqueBufferPtr data;  ///< Processed image data
        size_t size;           ///< Size of processed data in bytes
        
        ProcessResult() : data(nullptr), size(0) {}  ///< Default constructor
        ProcessResult(UniqueBufferPtr&& ptr, size_t sz)  ///< Parameterized constructor
            : data(std::move(ptr)), size(sz) {}
        
        operator bool() const { return static_cast<bool>(data); }
    };

    /**
     * @brief Construct an ImageProcessor.
     * @param config Image processor configuration
     * @param model_handler Pointer to ModelHandler for model dimensions
     */
    ImageProcessor(const ImageProcessorConfig &config, ModelHandler* model_handler);

    /**
     * @brief Split image into crop zones and process each zone.
     * @param image Camera image to process
     * @param zones Vector of crop zones to extract
     * @return Vector of processed image results for each zone
     */
    std::vector<ProcessResult> split_image_in_zone(
        std::shared_ptr<camera::CameraImage> image,
        const std::vector<CropZone> &zones = {});

    /**
     * @brief Process zone directly into pre-allocated buffer.
     * @param image Camera image to process
     * @param zone Crop zone to extract
     * @param output_buffer Pre-allocated output buffer
     * @param output_buffer_size Size of output buffer
     * @return true if successful, false otherwise
     */
    bool process_zone_to_buffer(
        std::shared_ptr<camera::CameraImage> image,
        const CropZone &zone,
        uint8_t* output_buffer,
        size_t output_buffer_size);

    /**
     * @brief Get processing statistics.
     * @return Current processing statistics
     */
    const ProcessingStats& get_stats() const { return stats_; }
    
    /**
     * @brief Reset processing statistics.
     */
    void reset_stats() { stats_.reset(); }
    
    /**
     * @brief Validate input image for processing.
     * @param image Camera image to validate
     * @return true if image is valid, false otherwise
     */
    bool validate_input_image(std::shared_ptr<camera::CameraImage> image) const;

 private:
    ProcessResult process_zone(
        std::shared_ptr<camera::CameraImage> image,
        const CropZone &zone);

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

    bool validate_zone(const CropZone &zone) const;
    
    bool validate_buffer_size(size_t required, size_t available, const char* context) const;
    
    const char* jpeg_error_to_string(jpeg_error_t error) const;
    
    // RGB888 processing functions
    bool process_rgb888_crop_and_scale_to_float32(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels,
        bool normalize);

    bool process_rgb888_crop_and_scale_to_uint8(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels);

    // RGB565 processing functions
    bool process_rgb565_crop_and_scale_to_float32(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels,
        bool normalize);

    bool process_rgb565_crop_and_scale_to_uint8(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels);

    // Grayscale processing functions
    bool process_grayscale_crop_and_scale_to_float32(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels,
        bool normalize);

    bool process_grayscale_crop_and_scale_to_uint8(
        const uint8_t* input_data, const CropZone &zone,
        int crop_width, int crop_height,
        uint8_t* output_buffer, int model_width, int model_height, int model_channels);

    // Scaling functions
    bool scale_rgb888_to_float32(
        const uint8_t* input_data, int input_width, int input_height,
        uint8_t* output_buffer, int output_width, int output_height, int output_channels,
        bool normalize);

    bool scale_rgb888_to_uint8(
        const uint8_t* input_data, int input_width, int input_height,
        uint8_t* output_buffer, int output_width, int output_height, int output_channels);

    UniqueBufferPtr allocate_image_buffer(size_t size);
    
    ImageProcessorConfig config_;  ///< Processor configuration
    ModelHandler* model_handler_;  ///< Model handler reference
    int bytes_per_pixel_;          ///< Bytes per pixel for current format
    ProcessingStats stats_;        ///< Processing statistics
    mutable std::mutex processing_mutex_; ///< Thread safety mutex
    
    
    /**
     * @brief Arranges RGB channels according to the configured input order for float output.
     * @param output Pointer to output buffer
     * @param r Red channel value
     * @param g Green channel value
     * @param b Blue channel value
     * @param output_channels Number of output channels
     * @param normalize Whether to normalize values (0-1 range)
     */
    void arrange_channels(float* output, uint8_t r, uint8_t g, uint8_t b, 
                         int output_channels, bool normalize) const;

    /**
     * @brief Arranges RGB channels according to the configured input order for uint8 output.
     * @param output Pointer to output buffer
     * @param r Red channel value
     * @param g Green channel value
     * @param b Blue channel value
     * @param output_channels Number of output channels
     */
    void arrange_channels(uint8_t* output, uint8_t r, uint8_t g, uint8_t b, 
                         int output_channels) const;
    
#ifdef DEBUG_METER_READER_TFLITE
    void debug_log_image(const uint8_t* data, size_t size, 
                        int width, int height, int channels,
                        const std::string& stage);
    void debug_log_float_image(const float* data, size_t count,
                              int width, int height, int channels,
                              const std::string& stage);
    void debug_log_rgb888_image(const uint8_t* data, 
                               int width, int height,
                               const std::string& stage);
    void debug_log_image_stats(const uint8_t* data, size_t size,
                              const std::string& stage);
    void debug_log_float_stats(const float* data, size_t count,
                              const std::string& stage);
                              
    void debug_analyze_processed_zone(const uint8_t* data, 
                                                 int width, int height, 
                                                 int channels,
                                                 const std::string& zone_name);
                                                 
    void debug_output_zone_preview(const uint8_t* data,
                                              int width, int height,
                                              int channels,
                                              const std::string& zone_name);
                                              
    void debug_analyze_float_zone(const float* data, 
                                             int width, int height, 
                                             int channels,
                                             const std::string& zone_name,
                                             bool normalized);
                                              
    void debug_output_float_preview(const float* data,
                                               int width, int height,
                                               int channels,
                                               const std::string& zone_name,
                                               bool normalized);
                                              

    
#endif
    
    
};

}  // namespace meter_reader_tflite
}  // namespace esphome
