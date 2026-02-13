/**
 * @file image_processor.cpp
 * @brief Implements the ImageProcessor class for handling image manipulation
 *        such as cropping, scaling, and format conversion for TensorFlow Lite.
 * 
 * Supports JPEG decoding with optional rotation (0 deg, 90 deg, 180 deg, 270 deg),
 * raw format processing (RGB888, RGB565, Grayscale), and conversion to
 * model input formats (float32 or uint8).
 */

// Prevent old JPEG headers from being included
#define ESP_JPEG_DEC_H
#define ESP_JPEG_COMMON_H

// Include esp_new_jpeg FIRST, before any ESPHome camera headers
#include <cstdint>
#include "esp_jpeg_dec.h"
#include "esp_jpeg_common.h"

// Then include other headers
#include "image_processor.h"
#include <algorithm>
#include <cmath>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

namespace esphome {
namespace esp32_camera_utils {

static const char *const TAG = "ImageProcessor";

// Static buffer pool instance (shared across all ImageProcessor instances)
BufferPool ImageProcessor::buffer_pool_;

// Initialize static member
std::atomic<int32_t> ImageProcessor::TrackedBuffer::active_instances{0};

// Duration logging macros for performance profiling
#ifdef DURATION_START
#undef DURATION_START
#endif
#define DURATION_START() uint32_t duration_start_ = millis()
#ifdef DURATION_END
#undef DURATION_END
#endif
#define DURATION_END(func) ESP_LOGD(TAG, "%s duration: %lums", func, millis() - duration_start_)

// Debug macros for image processing analysis (restored from legacy code)
#if defined(DEBUG_ESP32_CAMERA_UTILS) || defined(DEBUG_METER_READER_TFLITE)
#define DEBUG_ZONE_INFO(zone, crop_w, crop_h, model_w, model_h) \
    ESP_LOGI(TAG, "ZONE: [%d,%d,%d,%d] -> %dx%d -> %dx%d", \
             zone.x1, zone.y1, zone.x2, zone.y2, \
             crop_w, crop_h, model_w, model_h)

#define DEBUG_FIRST_PIXELS(data, count, channels) \
    do { \
        ESP_LOGI(TAG, "FIRST_PIXELS:"); \
        for (int i = 0; i < std::min(5, count); i += channels) { \
            if (i + channels - 1 < count) { \
                ESP_LOGI(TAG, "  Pixel %d: ", i/channels); \
                for (int c = 0; c < channels; c++) { \
                    ESP_LOGI(TAG, "    Ch%d: %.1f", c, data[i + c]); \
                } \
            } \
        } \
    } while (0)

#define DEBUG_CHANNEL_ORDER(data, count, channels) \
    do { \
        if (channels >= 3) { \
            ESP_LOGI(TAG, "CHANNEL_ORDER_TEST:"); \
            ESP_LOGI(TAG, "  First pixel: %.1f, %.1f, %.1f", \
                     data[0], data[1], data[2]); \
            /* Simple heuristic for BGR vs RGB detection */ \
            if (data[0] > data[2]) { \
                ESP_LOGI(TAG, "  -> Likely BGR order (first > last)"); \
            } else if (data[2] > data[0]) { \
                ESP_LOGI(TAG, "  -> Likely RGB order (last > first)"); \
            } else { \
                ESP_LOGI(TAG, "  -> Channel order unclear"); \
            } \
        } \
    } while (0)
#endif


// JPEG error code to string conversion
const char* ImageProcessor::jpeg_error_to_string(jpeg_error_t error) const {
    switch(error) {
        case JPEG_ERR_OK:            return "OK";
        case JPEG_ERR_FAIL:          return "Device error or wrong termination of input stream";
        case JPEG_ERR_NO_MEM:        return "Insufficient memory for the image";
        case JPEG_ERR_NO_MORE_DATA:  return "Input data is not enough";
        case JPEG_ERR_INVALID_PARAM: return "Parameter error";
        case JPEG_ERR_BAD_DATA:      return "Data format error (may be damaged data)";
        case JPEG_ERR_UNSUPPORT_FMT: return "Right format but not supported";
        case JPEG_ERR_UNSUPPORT_STD: return "Not supported JPEG standard";
        default:                     return "Unknown error";
    }
}

// -------------------------------------------------------------------------
// Static Helpers
// -------------------------------------------------------------------------

bool ImageProcessor::get_jpeg_dimensions(const uint8_t *data, size_t len, int &width, int &height) {
    if (!data || len < 10) return false;
    
    // Simple JPEG generic parser to find SOF0 (Start Of Frame 0) marker 0xFFC0
    // Based on standard JPEG structure
    for (size_t i = 0; i < len - 9; i++) {
        if (data[i] == 0xFF && data[i+1] == 0xC0) {
             // Found SOF0
             // Skip marker (2) + length (2) + precision (1) to get height (2) + width (2)
             // Height is at offset 5, Width at offset 7 from marker start (0xFF)
             int h_high = data[i+5];
             int h_low = data[i+6];
             int w_high = data[i+7];
             int w_low = data[i+8];
             
             height = (h_high << 8) | h_low;
             width = (w_high << 8) | w_low;
             return true;
        }
    }
    return false;
}


ImageProcessor::JpegBufferPtr ImageProcessor::decode_jpeg(const uint8_t* data, size_t len, int* width, int* height, jpeg_pixel_format_t output_format, jpeg_rotate_t rotate) {
    if (!data || len == 0) return nullptr;

    int w, h;
    if (!get_jpeg_dimensions(data, len, w, h)) {
         return nullptr;
    }
    if (width) *width = w;
    if (height) *height = h;

    if (rotate == JPEG_ROTATE_90D || rotate == JPEG_ROTATE_270D) {
        if (width) *width = h;
        if (height) *height = w;
    }

    jpeg_dec_config_t decode_config = DEFAULT_JPEG_DEC_CONFIG();
    decode_config.output_type = output_format;
    decode_config.scale.width = w;
    decode_config.scale.height = h;
    decode_config.clipper.width = w;
    decode_config.clipper.height = h;
    decode_config.rotate = rotate;
    decode_config.block_enable = false;

    jpeg_dec_handle_t decoder_handle = nullptr;
    if (jpeg_dec_open(&decode_config, &decoder_handle) != JPEG_ERR_OK) {
        return nullptr;
    }
    // RAII for decoder
    std::unique_ptr<struct jpeg_dec_s, JpegDecoderDeleter> decoder(decoder_handle);

    static bool version_logged = false;
    if (!version_logged) {
        // ESP_LOGI(TAG, "Using ESP_NEW_JPEG Library Version: %s", ESP_JPEG_VERION);
        version_logged = true;
    }

    int channels = 3;
    if (output_format == JPEG_PIXEL_FORMAT_GRAY) {
        channels = 1;
    } else if (output_format == JPEG_PIXEL_FORMAT_RGB565_LE || output_format == JPEG_PIXEL_FORMAT_RGB565_BE) {
        channels = 2; // Actually 2 bytes, but distinct format
    }
    
    size_t out_size = w * h * channels;
    uint8_t* raw_buf = static_cast<uint8_t*>(jpeg_calloc_align(out_size, 16));
    if (!raw_buf) {
        ESP_LOGE(TAG, "CRITICAL: Failed to allocate JPEG output buffer (aligned) of size %zu", out_size);
        return nullptr;
    }
    // RAII for buffer
    JpegBufferPtr out_buf(raw_buf);

    jpeg_dec_io_t io = {0};
    io.inbuf = const_cast<uint8_t*>(data);
    io.inbuf_len = len;
    io.inbuf_remain = len;
    io.outbuf = out_buf.get();
    io.out_size = out_size;

    jpeg_dec_header_info_t header_info;
    if (jpeg_dec_parse_header(decoder_handle, &io, &header_info) != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "JPEG Decode Error: Header parse failed");
        return nullptr;
    }

    if (jpeg_dec_process(decoder_handle, &io) != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "JPEG Decode Error: Process failed");
        return nullptr;
    }

    // Fix for ESP32 JPEG decoder outputting BGR (Little Endian RGB) instead of RGB.
    // The ESP32 hardware/software JPEG decoder typically outputs pixels in Little Endian format.
    // For 24-bit RGB, this results in the byte order B-G-R in memory.
    // Since our downstream processing (and the web preview) expects standard R-G-B order,
    // we must perform an in-place swap of the Red (0) and Blue (2) channels.
    // Source: Common ESP32 camera/JPEG behavior observed in ESP-IDF and Arduino implementations.
    // Fix for ESP32 JPEG decoder outputting BGR (Little Endian RGB) instead of RGB.
    // DISABLING THIS FIX as it seems to cause incorrect BGR output observed by user. 
    // Hypothesis: Decoder now outputs RGB correctly, so this swap breaks it.
    /*
    #ifdef ESP32
    if (output_format == JPEG_PIXEL_FORMAT_RGB888) {
        // Iterate through pixels and swap R (index 0) and B (index 2)
        // BGR (Input) -> RGB (Output)
        uint8_t* pixels = out_buf.get();
        for (size_t i = 0; i < out_size; i += 3) {
            uint8_t temp = pixels[i];     // B (from BGR)
            pixels[i] = pixels[i+2];      // Move R to index 0
            pixels[i+2] = temp;           // Move B to index 2
        }
    } else if (output_format == JPEG_PIXEL_FORMAT_RGB565_LE || output_format == JPEG_PIXEL_FORMAT_RGB565_BE) {
        // Fix for RGB565 as well
        uint16_t* pixels = (uint16_t*)out_buf.get();
        size_t count = out_size / 2;
        for (size_t i = 0; i < count; i++) {
             uint16_t p = pixels[i];
             // Swap Red and Blue in 565 format (R=top 5, B=bottom 5)
             // RRRRR GGGGGG BBBBB
             // Mask R: 0xF800, Mask B: 0x001F
             pixels[i] = (p & 0x07E0) | ((p & 0xF800) >> 11) | ((p & 0x001F) << 11);
        }
    }
    #endif
    */

    return out_buf;
}

#ifdef USE_CAMERA_ROTATOR
std::shared_ptr<camera::CameraImage> ImageProcessor::generate_rotated_preview(
    std::shared_ptr<camera::CameraImage> source, 
    float rotation, int width, int height) {
    
    if (!source) return nullptr;

    const uint8_t *jpeg_data = source->get_data_buffer();
    size_t jpeg_size = source->get_data_length();
    
    // 1. Determine dimensions (prefer JPEG header)
    int src_w = width; 
    int src_h = height; 
    int jpeg_w, jpeg_h;
    if (get_jpeg_dimensions(jpeg_data, jpeg_size, jpeg_w, jpeg_h)) {
        src_w = jpeg_w;
        src_h = jpeg_h;
    } 

    // 2. Determine Rotation Optimization
    // Check for standard angles to use decoder rotation (Hardware/Optimized)
    jpeg_rotate_t hw_rotation = JPEG_ROTATE_0D;
    bool software_rotation_needed = false;
    
    float rot = rotation;
    while (rot < 0) rot += 360.0f;
    while (rot >= 360.0f) rot -= 360.0f;
    
    if (std::abs(rot) < 0.1f) {
         hw_rotation = JPEG_ROTATE_0D;
    } else if (std::abs(rot - 90.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_90D;
    } else if (std::abs(rot - 180.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_180D;
    } else if (std::abs(rot - 270.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_270D;
    } else {
         software_rotation_needed = true;
    }

    // 3. Decode with optimization
    int dec_w = 0, dec_h = 0;
    // We use RGB888 for preview
    JpegBufferPtr full_image_buf = decode_jpeg(jpeg_data, jpeg_size, &dec_w, &dec_h, JPEG_PIXEL_FORMAT_RGB888, hw_rotation);
    if (!full_image_buf) {
        ESP_LOGW(TAG, "Preview: Failed to decode input image");
        return nullptr;
    }
    
    // 4. Handle Software Rotation if needed
    if (software_rotation_needed && std::abs(rotation) > 0.01f) {
         // Calculate new dimensions
         int new_w = 0, new_h = 0;
         Rotator::get_rotated_dimensions(dec_w, dec_h, rotation, new_w, new_h);
         
         size_t out_size = new_w * new_h * 3;
         JpegBufferPtr rot_buf = allocate_jpeg_buffer(out_size);
         
         if (rot_buf) {
            bool rot_success = Rotator::perform_rotation(
                full_image_buf.get(), rot_buf.get(),
                dec_w, dec_h, 3,
                rotation, new_w, new_h
            );
            
             if (rot_success) {
                 full_image_buf = std::move(rot_buf);
                 dec_w = new_w;
                 dec_h = new_h;
             } else {
                  ESP_LOGE(TAG, "Preview: Zone rotation failed");
                  return nullptr;
             }
         } else {
              ESP_LOGE(TAG, "Preview: Zone rotation malloc failed");
              return nullptr;
         }
    }

    // 5. Wrap in RotatedPreviewImage (Copy to TrackedBuffer)
    size_t out_size = dec_w * dec_h * 3;
    UniqueBufferPtr out_buffer = allocate_image_buffer(out_size);
    if (!out_buffer) return nullptr;
    memcpy(out_buffer->get(), full_image_buf.get(), out_size);
    
    return std::shared_ptr<RotatedPreviewImage>(new RotatedPreviewImage(
        std::move(out_buffer), out_size,
        dec_w, dec_h,
        PIXFORMAT_RGB888
    ));
}
#endif

bool ImageProcessor::validate_buffer_size(size_t required, size_t available, const char* context) const {
    if (available < required) {
        ESP_LOGE(TAG, "Buffer too small for %s: need %zu bytes, have %zu bytes", 
                context, required, available);
        return false;
    }
    return true;
}

bool ImageProcessor::validate_input_image(std::shared_ptr<camera::CameraImage> image) const {
    if (!image) {
        ESP_LOGE(TAG, "Null image pointer");
        return false;
    }
    
    if (!image->get_data_buffer()) {
        ESP_LOGE(TAG, "Image data buffer is null");
        return false;
    }
    
    if (image->get_data_length() == 0) {
        ESP_LOGE(TAG, "Image data length is zero");
        return false;
    }
    
    if (this->config_.pixel_format == "JPEG") {
        if (image->get_data_length() < 2) {
            ESP_LOGE(TAG, "JPEG data too short");
            return false;
        }
        
        const uint8_t* data = image->get_data_buffer();
        if (data[0] != 0xFF || data[1] != 0xD8) {
            ESP_LOGW(TAG, "Invalid JPEG signature: 0x%02X 0x%02X", data[0], data[1]);
        }
    }
    
    return true;
}

CropZone adjust_zone_for_jpeg(const CropZone &zone, int max_width, int max_height) {
    CropZone adjusted = zone;
    
    int width = zone.x2 - zone.x1;
    int adjusted_width = (width + 4) / 8 * 8;
    if (adjusted_width > width) {
        adjusted_width -= 8;
    }
    adjusted_width = std::max(8, adjusted_width);
    
    int height = zone.y2 - zone.y1;
    int adjusted_height = (height + 4) / 8 * 8;
    if (adjusted_height > height) {
        adjusted_height -= 8;
    }
    adjusted_height = std::max(8, adjusted_height);
    
    adjusted.x1 = zone.x1 + (width - adjusted_width) / 2;
    adjusted.y1 = zone.y1 + (height - adjusted_height) / 2;
    adjusted.x2 = adjusted.x1 + adjusted_width;
    adjusted.y2 = adjusted.y1 + adjusted_height;
    
    adjusted.x1 = std::max(0, adjusted.x1);
    adjusted.y1 = std::max(0, adjusted.y1);
    adjusted.x2 = std::min(max_width, adjusted.x2);
    adjusted.y2 = std::min(max_height, adjusted.y2);
    
    int final_width = adjusted.x2 - adjusted.x1;
    int final_height = adjusted.y2 - adjusted.y1;
    final_width = (final_width / 8) * 8;
    final_height = (final_height / 8) * 8;
    adjusted.x2 = adjusted.x1 + final_width;
    adjusted.y2 = adjusted.y1 + final_height;
    
    return adjusted;
}

ImageProcessor::ImageProcessor(const ImageProcessorConfig &config)
  : config_(config) {
  
  if (!this->config_.validate()) {
    ESP_LOGE(TAG, "Invalid image processor configuration");
    ESP_LOGE(TAG, "  Camera: %dx%d, Format: %s", 
             this->config_.camera_width, this->config_.camera_height, this->config_.pixel_format.c_str());
  }
  
  #ifdef USE_CAMERA_ROTATOR
  if (this->config_.rotation != ROTATION_0) {
    ESP_LOGI(TAG, "Image rotation enabled: %d degrees clockwise", static_cast<int>(this->config_.rotation));
  }
  #endif

  if (this->config_.pixel_format == "RGB888") {
    this->bytes_per_pixel_ = 3;
  } else if (this->config_.pixel_format == "RGB565") {
    this->bytes_per_pixel_ = 2;
  } else if (this->config_.pixel_format == "YUV422") {
    this->bytes_per_pixel_ = 2;
  } else if (this->config_.pixel_format == "GRAYSCALE") {
    this->bytes_per_pixel_ = 1;
  } else if (this->config_.pixel_format == "JPEG") {
    this->bytes_per_pixel_ = 3;
  } else {
    ESP_LOGE(TAG, "Unsupported pixel format: %s", this->config_.pixel_format.c_str());
    this->bytes_per_pixel_ = 3;
  }
}


ImageProcessor::UniqueBufferPtr ImageProcessor::allocate_image_buffer(size_t size) {
    // Use buffer pool for small/medium buffers (model inputs)
    // Direct allocation for very large buffers (full frames)
    static constexpr size_t POOL_SIZE_THRESHOLD = 50 * 1024; // 50KB
    
    if (size <= POOL_SIZE_THRESHOLD) {
        // Try buffer pool first
        auto pooled_buffer = buffer_pool_.acquire(size);
        
        if (pooled_buffer.data) {
            // Successfully got buffer from pool
            // TrackedBuffer(ptr, spiram, jpeg_aligned, pooled, size)
            if (TrackedBuffer::active_instances > 5) {
                // Log on allocation if debug is enabled to track memory usage.
                #ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
                ESP_LOGD(TAG, "Allocated pooled buffer. Active TrackedBuffers: %d", TrackedBuffer::active_instances.load() + 1);
                #endif
            }
            return UniqueBufferPtr(new TrackedBuffer(pooled_buffer.data, false, false, true, pooled_buffer.size));
        }
        // Pool acquisition failed, fall through to direct allocation
        ESP_LOGW(TAG, "Buffer pool acquisition failed for %zu bytes, using direct allocation", size);
    }
    
    // Direct allocation for large buffers or pool failure
    uint8_t* ptr = nullptr;
    bool spiram = false;
    
    // Try SPIRAM first if size is significant
    if (size > 1024) {
        ptr = (uint8_t*)heap_caps_aligned_alloc(64, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ptr) spiram = true;
    }
    
    // Fallback to internal/default RAM
    if (!ptr) {
        ptr = (uint8_t*)heap_caps_aligned_alloc(64, size, MALLOC_CAP_8BIT);
        spiram = false;
    }
    
    if (!ptr) {
        ESP_LOGE(TAG, "CRITICAL: Failed to allocate image buffer of size %zu (Direct & Pool failed)", size);
        return nullptr;
    }
    
    // TrackedBuffer(ptr, spiram, jpeg_aligned, pooled, size)
    #ifdef DEBUG_ESP32_CAMERA_UTILS_MEMORY
    ESP_LOGD(TAG, "Allocated direct buffer. Active TrackedBuffers: %d", TrackedBuffer::active_instances.load() + 1);
    #endif
    return UniqueBufferPtr(new TrackedBuffer(ptr, spiram, false, false, size));
}

ImageProcessor::JpegBufferPtr ImageProcessor::allocate_jpeg_buffer(size_t size) {
    // We use jpeg_calloc_align because the deleter (JpegBufferDeleter) calls jpeg_free_align.
    // 16-byte alignment is standard for ESP JPEG lib.
    uint8_t* ptr = static_cast<uint8_t*>(jpeg_calloc_align(size, 16));
    return JpegBufferPtr(ptr);
}

size_t ImageProcessor::get_required_buffer_size() const {
    if (this->config_.input_type == kInputTypeFloat32) {
        uint64_t size = static_cast<uint64_t>(this->config_.model_width) * this->config_.model_height * this->config_.model_channels * sizeof(float);
        if (size > SIZE_MAX) return 0;
        return static_cast<size_t>(size);
    } else if (this->config_.input_type == kInputTypeUInt8) {
        uint64_t size = static_cast<uint64_t>(this->config_.model_width) * this->config_.model_height * this->config_.model_channels;
        if (size > SIZE_MAX) return 0;
        return static_cast<size_t>(size);
    }
    return 0;
}

std::vector<ImageProcessor::ProcessResult> ImageProcessor::split_image_in_zone(
    std::shared_ptr<camera::CameraImage> image,
    const std::vector<CropZone> &zones) {
    
    if (!this->validate_input_image(image)) {
        this->stats_.failed_frames++;
        return {};
    }
    
    // Create a span from the image data
    std::span<uint8_t> data_span(image->get_data_buffer(), image->get_data_length());
    
    // Call the span-based implementation (assuming dimensions are needed, we extract them here or pass them)
    // Note: The span version needs width/height. We can try to extract them if it's JPEG, or use config/image props.
    // For JPEG, width/height in config might be camera dims.
    int w = this->config_.camera_width;
    int h = this->config_.camera_height;
    
    // Try to get actual dims from JPEG header if available
    if (this->config_.pixel_format == "JPEG") {
         get_jpeg_dimensions(image->get_data_buffer(), image->get_data_length(), w, h);
    }
    
    return this->split_image_in_zone(data_span, w, h, zones);
}

std::vector<ImageProcessor::ProcessResult> ImageProcessor::split_image_in_zone(
    std::span<uint8_t> image_data, int width, int height,
    const std::vector<CropZone> &zones) {
   
  std::lock_guard<std::mutex> lock(this->processing_mutex_);
  std::vector<ProcessResult> results;
  
  this->stats_.total_frames++;
  uint32_t start_time = millis();
  
  if (image_data.empty()) {
    this->stats_.failed_frames++;
    return results;
  }


  // Optimization: Decode and Rotate ONCE if multiple zones exist
  // Only applies if format is JPEG (ProcessRaw is already efficient-ish, though strict raw doesn't allocate intermediate usually)
  
  std::vector<CropZone> effective_zones = zones;
  if (effective_zones.empty()) {
      // Create a full-frame zone if none provided
      int w = this->config_.camera_width;
      int h = this->config_.camera_height;
      if (this->config_.pixel_format == "JPEG") {
        get_jpeg_dimensions(image_data.data(), image_data.size(), w, h);
      }
      effective_zones.push_back({0, 0, w, h});
  }

  // Intermediate buffer state
  JpegBufferPtr master_decoded_buffer; // Holds decoded JPEG or rotated raw
  int master_width = width;
  int master_height = height;
  int master_channels = 3; 

  bool use_master_buffer = false;
  
  // Memory Pressure Detection - Check available memory before deciding strategy
  static constexpr size_t SPIRAM_RESERVE_THRESHOLD = 512 * 1024;  // 512KB reserve for PSRAM boards
  static constexpr size_t INTERNAL_RESERVE_THRESHOLD = 100 * 1024; // 100KB reserve for non-PSRAM boards
  
  size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  
  // Log memory state for debugging invisible failures
  ESP_LOGI(TAG, "Image Processing Start. PSRAM Free: %zu KB (Threshold: %zu KB). Format: %s", 
           free_spiram/1024, SPIRAM_RESERVE_THRESHOLD/1024, this->config_.pixel_format.c_str());

  bool low_memory = false;
  if (free_spiram > 0) {
      // PSRAM available - check if below threshold
      low_memory = (free_spiram < SPIRAM_RESERVE_THRESHOLD);
      if (low_memory) {
          ESP_LOGW(TAG, "Low PSRAM detected, using zone-by-zone processing");
      }
  } else {
      // No PSRAM - check internal RAM (ESP32 without PSRAM)
      low_memory = (free_internal < INTERNAL_RESERVE_THRESHOLD);
      if (low_memory) {
          ESP_LOGW(TAG, "Low internal RAM detected, using zone-by-zone processing");
      }
  }
  
  // Decide if we should decode once
  // Use master buffer for JPEG if memory is available, otherwise process zone-by-zone
  if (this->config_.pixel_format == "JPEG" && !low_memory) {
      use_master_buffer = true;
      
      const uint8_t *jpeg_data = image_data.data();
      size_t jpeg_size = image_data.size();

      // 1. Determine format and rotation optimization
      jpeg_pixel_format_t decode_format = JPEG_PIXEL_FORMAT_RGB888;
      int decode_channels = 3;
      if (this->config_.pixel_format == "GRAYSCALE") {
          decode_format = JPEG_PIXEL_FORMAT_GRAY;
          decode_channels = 1;
      }
      
      // Check for standard angles to use decoder rotation (Hardware/Optimized)
      jpeg_rotate_t hw_rotation = JPEG_ROTATE_0D;
      bool software_rotation_needed = false;
      
      // Normalize rotation for check
      float rot = this->config_.rotation;
      while (rot < 0) rot += 360.0f;
      while (rot >= 360.0f) rot -= 360.0f;
      
      if (std::abs(rot) < 0.1f) {
           hw_rotation = JPEG_ROTATE_0D;
      } else if (std::abs(rot - 90.0f) < 0.1f) {
           hw_rotation = JPEG_ROTATE_90D;
      } else if (std::abs(rot - 180.0f) < 0.1f) {
           hw_rotation = JPEG_ROTATE_180D;
      } else if (std::abs(rot - 270.0f) < 0.1f) {
           hw_rotation = JPEG_ROTATE_270D;
      } else {
           // arbitrary angle, use software rotation
           software_rotation_needed = true;
      }
      
      // 2. Decode (with optional rotation)
      int dec_w = 0, dec_h = 0;
      master_decoded_buffer = decode_jpeg(jpeg_data, jpeg_size, &dec_w, &dec_h, decode_format, hw_rotation);
      
      if (!master_decoded_buffer) {
          ESP_LOGE(TAG, "CRITICAL: Image Decoding Failed (Master Buffer). Check heap/format.");
          this->stats_.jpeg_decoding_errors++;
          this->stats_.failed_frames++;
          return results;
      }
      master_width = dec_w;
      master_height = dec_h;
      master_channels = decode_channels;

      // 3. Rotate Master if needed (Software fallback for fine angles)
      #ifdef USE_CAMERA_ROTATOR
      if (software_rotation_needed && std::abs(this->config_.rotation) > 0.01f) {
           // ... (Existing software rotation logic) ...
           float rads = this->config_.rotation * static_cast<float>(M_PI) / 180.0f;
           float abs_cos = std::abs(std::cos(rads));
           float abs_sin = std::abs(std::sin(rads));
           int rot_w = static_cast<int>(master_width * abs_cos + master_height * abs_sin);
           int rot_h = static_cast<int>(master_width * abs_sin + master_height * abs_cos);
           
           size_t rot_size = rot_w * rot_h * master_channels;
           
           // We use jpeg_calloc_align for consistency since master_decoded_buffer is JpegBufferPtr (uptr with free_align)
           // Actually decode_jpeg returns a smart pointer compatible with jpeg_free_align.
           uint8_t* raw_rot = static_cast<uint8_t*>(jpeg_calloc_align(rot_size, 16));
           if (raw_rot) {
               bool rot_success = Rotator::perform_rotation(
                   master_decoded_buffer.get(), raw_rot,
                   master_width, master_height, master_channels,
                   this->config_.rotation, rot_w, rot_h
               );
               
               if (rot_success) {
                   // Replace master buffer with rotated version
                   master_decoded_buffer.reset(raw_rot);
                   master_width = rot_w;
                   master_height = rot_h;
               } else {
                   // Rotation failed, free raw_rot (managed by uptr if we assigned it, but we didn't yet)
                   jpeg_free_align(raw_rot);
                   ESP_LOGE(TAG, "Master rotation failed");
                    // Rotation failed. Fail safe to prevent invalid image usage.
                    return results;
               }
           } else {
               ESP_LOGE(TAG, "Failed to allocate master rotation buffer");
               return results;
           }
      }
      #endif
  }

  // PREVIEW CACHING LOGIC
  const uint8_t* processing_source_ptr = nullptr;
  if (use_master_buffer && master_decoded_buffer) {
        // Fix BGR -> RGB unconditionally for 3-channel
        // Since we disabled the internal fix in decode_jpeg, we know we have specific BGR output to correct.
        if (master_channels == 3) {
            // ESP_LOGD(TAG, "ImageProcessor: Executing BGR->RGB Swap on %dx%d image", master_width, master_height);
            uint8_t* raw_buf = master_decoded_buffer.get();
            size_t pixel_count = master_width * master_height;
            for (size_t i = 0; i < pixel_count; i++) {
                size_t idx = i * 3;
                uint8_t temp = raw_buf[idx];
                raw_buf[idx] = raw_buf[idx + 2];
                raw_buf[idx + 2] = temp;
            }
        } else {
             // ESP_LOGW(TAG, "ImageProcessor: Skipping BGR->RGB Swap. Channels=%d", master_channels);
        }

        if (this->config_.cache_preview_image) {
            uint8_t* raw = master_decoded_buffer.release();
            // Wrap in TrackedBuffer (jpeg aligned)
            // Constructor: ptr, spiram, jpeg_aligned, pooled, size
            // Note: size=0 is safe for free (jpeg_free_align doesn't need it), but checking track usage might be off.
            // Using 0 for now as we don't easily know alloc size unless we track it from decode/alloc.
            // Actually decode_jpeg allocates size = w*h*c roughly.
            TrackedBuffer* tb = new TrackedBuffer(raw, false, true, false, 0);
            UniqueBufferPtr uptr(tb);
            
            size_t buf_len = master_width * master_height * master_channels;
            
            // Create preview image
             std::shared_ptr<RotatedPreviewImage> cached_img = std::make_shared<RotatedPreviewImage>(
                std::move(uptr), 
                buf_len,
                master_width, master_height,
                (master_channels == 1) ? PIXFORMAT_GRAYSCALE : PIXFORMAT_RGB888 
            );
            this->last_processed_image_ = cached_img;
            processing_source_ptr = raw; // Still valid, owned by cached_img
            // ESP_LOGD(TAG, "ImageProcessor: Cached preview image. Ptr: %p", (void*)cached_img.get());
        } else {
             ESP_LOGD(TAG, "ImageProcessor: Preview caching DISABLED. Releasing buffer.");
             processing_source_ptr = master_decoded_buffer.get();
        }
  }

  // Process Zones from Master Buffer (or Source if not JPEG/Mastered)
  bool all_zones_successful = true;
  for (const auto& zone : effective_zones) {
      ProcessResult result;
      bool crop_success = false;
      
      size_t required_size = get_required_buffer_size();
      if (required_size == 0) continue;
      
      UniqueBufferPtr out_buf = allocate_image_buffer(required_size);
      if (!out_buf) {
          ESP_LOGE(TAG, "Failed to allocate buffer of size %zu for zone [%d,%d,%d,%d]", 
                   required_size, zone.x1, zone.y1, zone.x2, zone.y2);
          all_zones_successful = false;
          continue;
      }

      if (use_master_buffer) {
          // Validate effective zone against master dimensions
          if (zone.x1 >= 0 && zone.y1 >= 0 && zone.x2 <= master_width && zone.y2 <= master_height) {
              // Extract from RAW master buffer
              // We need a helper that takes raw buffer + dims instead of CameraImage
              // We can adapt 'process_raw_zone_to_buffer' logic here inline or extract method
              
              int stride = master_width;
              int crop_w = zone.x2 - zone.x1;
              int crop_h = zone.y2 - zone.y1;
              
              if (master_channels == 1) {
                  // Grayscale
                   if (this->config_.input_type == kInputTypeFloat32) {
                        crop_success = this->process_grayscale_crop_and_scale_to_float32(
                            processing_source_ptr, zone, crop_w, crop_h,
                            out_buf->get(), this->config_.model_width, this->config_.model_height,
                            this->config_.model_channels, this->config_.normalize, stride);
                   } else {
                        crop_success = this->process_grayscale_crop_and_scale_to_uint8(
                            processing_source_ptr, zone, crop_w, crop_h,
                            out_buf->get(), this->config_.model_width, this->config_.model_height,
                            this->config_.model_channels, stride);
                        if (!crop_success) {
                             ESP_LOGE(TAG, "process_grayscale... failed for zone [%d,%d,%d,%d]", 
                                      zone.x1, zone.y1, zone.x2, zone.y2);
                        }
                   }
              } else {
                  // RGB
                   if (this->config_.input_type == kInputTypeFloat32) {
                        crop_success = this->process_rgb888_crop_and_scale_to_float32(
                            processing_source_ptr, zone, crop_w, crop_h,
                            out_buf->get(), this->config_.model_width, this->config_.model_height,
                            this->config_.model_channels, this->config_.normalize, stride);
                   } else {
                        crop_success = this->process_rgb888_crop_and_scale_to_uint8(
                            processing_source_ptr, zone, crop_w, crop_h,
                            out_buf->get(), this->config_.model_width, this->config_.model_height,
                            this->config_.model_channels, stride);
                   }
              }
          } else {
              ESP_LOGE(TAG, "Zone bounds error vs Master: [%d,%d->%d,%d] in %dx%d", 
                  zone.x1, zone.y1, zone.x2, zone.y2, master_width, master_height);
          }
      } else {
          // Legacy/Raw CameraImage path - adapted for span
          // Implementation of raw zone extraction using the span data:
           if (this->process_raw_zone_pointer(image_data.data(), image_data.size(), width, height, zone, out_buf->get(), required_size)) {
              crop_success = true;
           } else {
               // Fallback or error log
               ESP_LOGE(TAG, "Raw zone processing failed");
           }
      }
      
      if (crop_success) {
          result.data = std::move(out_buf);
          result.size = required_size;
          results.push_back(std::move(result));
          
          #ifdef DEBUG_ESP32_CAMERA_UTILS
          DEBUG_ZONE_INFO(zone, (zone.x2 - zone.x1), (zone.y2 - zone.y1), this->config_.model_width, this->config_.model_height);
          #endif
      } else {
          all_zones_successful = false;
      }
  }
  
  if (!all_zones_successful) {
      this->stats_.failed_frames++;
  }
  
  uint32_t processing_time = millis() - start_time;
  this->stats_.total_processing_time_ms += processing_time;
  
  return results;
}

bool ImageProcessor::process_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t* output_buffer,
    size_t output_buffer_size) {
    
    std::lock_guard<std::mutex> lock(this->processing_mutex_);
    DURATION_START();
    
    if (!this->validate_input_image(image)) {
        return false;
    }
    
    // Check zone bounds
    // Note: If generating a preview with rotation, the 'image' has original dimensions,
    // but the 'zone' targets the rotated result.
    // We must validate against the "logical" dimensions of the source corresponding to the operation.
    
    int max_w = this->config_.camera_width;
    int max_h = this->config_.camera_height;
    
    #ifdef USE_CAMERA_ROTATOR
    if (this->config_.pixel_format == "JPEG" && this->config_.rotation != ROTATION_0) {
        // Use preview config dimensions if available, as they represent the target rotation
        // But better: swap max_w/max_h if 90/270 rotation is active
        if (this->config_.rotation == ROTATION_90 || this->config_.rotation == ROTATION_270) {
            std::swap(max_w, max_h); 
        }
    }
    #endif

    if (zone.x1 < 0 || zone.y1 < 0 || zone.x2 > max_w || zone.y2 > max_h) {
        ESP_LOGE(TAG, "Zone out of bounds: [%d,%d->%d,%d] for image %dx%d (rot %d)",
                 zone.x1, zone.y1, zone.x2, zone.y2, max_w, max_h, static_cast<int>(this->config_.rotation));
        return false;
    }
    
    if (zone.x1 >= zone.x2 || zone.y1 >= zone.y2) {
        ESP_LOGE(TAG, "Invalid zone dimensions");
        return false;
    }

    bool success = false;
    
    if (this->config_.pixel_format == "JPEG") {
        CropZone adjusted_zone = adjust_zone_for_jpeg(zone, this->config_.camera_width, this->config_.camera_height);
        success = this->process_jpeg_zone_to_buffer(image, adjusted_zone, output_buffer, output_buffer_size);
    } else {
        success = this->process_raw_zone_to_buffer(image, zone, output_buffer, output_buffer_size);
    }
    
    DURATION_END("process_zone_to_buffer");
    return success;
}

// Helper to parse JPEG dimensions (accessible to class)
bool get_jpeg_dimensions(const uint8_t* data, size_t size, int& width, int& height) {
    if (size < 2 || data[0] != 0xFF || data[1] != 0xD8) return false;
    size_t pos = 2;
    while (pos < size) {
        // Skip padding 0xFF
        while (pos < size && data[pos] == 0xFF) pos++;
        if (pos >= size) return false;
        
        uint8_t marker = data[pos];
        pos++;
        
        if (marker == 0xDA) return false; // SOS - header ended
        if (marker == 0xD9) return false; // EOI
        
        if (pos + 2 > size) return false;
        uint16_t len = (data[pos] << 8) | data[pos + 1];
        
        if (marker == 0xC0 || marker == 0xC2) { // SOF0 (Baseline) or SOF2 (Progressive)
            if (pos + 7 > size) return false;
            height = (data[pos + 3] << 8) | data[pos + 4];
            width = (data[pos + 5] << 8) | data[pos + 6];
            return true;
        }
        
        pos += len;
    }
    return false;
}

bool ImageProcessor::process_jpeg_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t* output_buffer,
    size_t output_buffer_size) {
    
    const uint8_t *jpeg_data = image->get_data_buffer();
    size_t jpeg_size = image->get_data_length();
    
    if (!jpeg_data || jpeg_size == 0) return false;

    size_t required_size = this->get_required_buffer_size();
    
    if (!this->validate_buffer_size(required_size, output_buffer_size, "JPEG processing")) {
        return false;
    }

    // Determine actual JPEG dimensions to avoid resize errors
    int jpeg_width = this->config_.camera_width;
    int jpeg_height = this->config_.camera_height;
    
    // Always attempt to get actual dimensions from JPEG header
    if (get_jpeg_dimensions(jpeg_data, jpeg_size, jpeg_width, jpeg_height)) {
        if (jpeg_width != this->config_.camera_width || jpeg_height != this->config_.camera_height) {
             // Just debug log, this is expected in windowed mode or if using debug image
            ESP_LOGD(TAG, "JPEG dimensions (%dx%d) differ from config (%dx%d). Using actual dimensions.",
                     jpeg_width, jpeg_height, this->config_.camera_width, this->config_.camera_height);
        }
    } else {
        ESP_LOGW(TAG, "Failed to parse JPEG dimensions, using config");
    }

    // Determine decode format based on model needs
    // If model needs 1 channel, we decode directly to GRAY to save memory/time
    jpeg_pixel_format_t decode_format = JPEG_PIXEL_FORMAT_RGB888;
    int decode_channels = 3;
    
    // Explicit grayscale request
    if (this->config_.pixel_format == "GRAYSCALE") {
        decode_format = JPEG_PIXEL_FORMAT_GRAY;
        decode_channels = 1;
    }

    int dec_w = 0, dec_h = 0;
    
    // Determine Rotation Optimization
    // Check for standard angles to use decoder rotation (Hardware/Optimized)
    jpeg_rotate_t hw_rotation = JPEG_ROTATE_0D;
    bool software_rotation_needed = false;
    
    // Normalize rotation for check
    float rot = this->config_.rotation;
    while (rot < 0) rot += 360.0f;
    while (rot >= 360.0f) rot -= 360.0f;
    
    if (std::abs(rot) < 0.1f) {
         hw_rotation = JPEG_ROTATE_0D;
    } else if (std::abs(rot - 90.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_90D;
    } else if (std::abs(rot - 180.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_180D;
    } else if (std::abs(rot - 270.0f) < 0.1f) {
         hw_rotation = JPEG_ROTATE_270D;
    } else {
         software_rotation_needed = true;
    }

    JpegBufferPtr full_image_buf = decode_jpeg(jpeg_data, jpeg_size, &dec_w, &dec_h, decode_format, hw_rotation);
    
    if (!full_image_buf) {
        this->stats_.jpeg_decoding_errors++;
        return false;
    }

    // Update with authoritative dimensions
    jpeg_width = dec_w;
    jpeg_height = dec_h;

    int crop_width = zone.x2 - zone.x1;
    int crop_height = zone.y2 - zone.y1;

    // Validate zone against ACTUAL image dimensions
    // Note: If rotated, dimensions are already swapped by decode_jpeg if needed.
    if (zone.x1 < 0 || zone.y1 < 0 || 
        zone.x2 > jpeg_width || zone.y2 > jpeg_height) {
        ESP_LOGE(TAG, "Crop zone (%d,%d -> %d,%d) out of bounds for actual image (%dx%d) - Config: %dx%d",
                 zone.x1, zone.y1, zone.x2, zone.y2, jpeg_width, jpeg_height,
                 this->config_.camera_width, this->config_.camera_height);
        return false;
    }

    // 2. Handle Rotation (Software - Arbitrary)
    uint8_t* processing_buf = full_image_buf.get();
    int proc_width = jpeg_width;
    int proc_height = jpeg_height;
    UniqueBufferPtr rotated_buffer_storage;

    #ifdef USE_CAMERA_ROTATOR
    if (software_rotation_needed && std::abs(this->config_.rotation) > 0.01f) {
        
        // Allocate buffer for rotated image with new bounding box
        int out_w = 0, out_h = 0;
        
        // First calculate size
        float rads = this->config_.rotation * static_cast<float>(M_PI) / 180.0f;
        float abs_cos = std::abs(std::cos(rads));
        float abs_sin = std::abs(std::sin(rads));
        out_w = static_cast<int>(jpeg_width * abs_cos + jpeg_height * abs_sin);
        out_h = static_cast<int>(jpeg_width * abs_sin + jpeg_height * abs_cos);
        
        size_t rotated_size = out_w * out_h * decode_channels;
        rotated_buffer_storage = this->allocate_image_buffer(rotated_size);
        
        if (!rotated_buffer_storage) {
            ESP_LOGE(TAG, "Failed to allocate rotation buffer: %zu bytes", rotated_size);
            return false;
        }
        
        uint8_t* rotated_buf = rotated_buffer_storage->get();

        // Use helper to Rotate: Full -> Rotated
        bool rot_success = Rotator::perform_rotation(full_image_buf.get(), rotated_buf, 
                                                 jpeg_width, jpeg_height, 
                                                 decode_channels, this->config_.rotation, out_w, out_h); 
        
        if (rot_success) {
            // Update processing context to new dimensions
            proc_width = out_w;
            proc_height = out_h;
            
            // Clean up original decode buffer early to save memory
            full_image_buf.reset();
            
            // Wrap in RotatedPreviewImage to keep it alive and accessible for preview ONLY if requested
            if (this->config_.cache_preview_image) {
                // We transfer ownership of the UniqueBufferPtr to the shared_ptr<RotatedPreviewImage>
                std::shared_ptr<RotatedPreviewImage> cached_img = std::make_shared<RotatedPreviewImage>(
                    std::move(rotated_buffer_storage), 
                    rotated_size,
                    out_w, out_h,
                    (decode_channels == 1) ? PIXFORMAT_GRAYSCALE : PIXFORMAT_RGB888 
                );
                
                this->last_processed_image_ = cached_img;
                // ESP_LOGD(TAG, "ImageProcessor: Cached preview image. Ptr: %p", (void*)cached_img.get());
                
                // Access raw pointer from cached image
                processing_buf = cached_img->get_data_buffer();
            } else {
                ESP_LOGD(TAG, "ImageProcessor: Preview caching DISABLED. Releasing buffer.");
                // No caching, just use the rotated buffer directly
                // rotated_buffer_storage owns it, we just point processing_buf to it
                // and it will be auto-released at end of scope unless we moved it (which we didn't here)
                processing_buf = rotated_buffer_storage->get();
            }
            
        } else {
            ESP_LOGE(TAG, "Software rotation failed");
            return false;
        }
    }
    #endif

    // 3. Scale / Crop from Processing Buffer (Rotated if needed)
    int stride = proc_width;
    
    // Note: 'zone' is already validated against 'proc_width/proc_height'
    
    bool scale_success = false;
    
    if (decode_channels == 1) {
        // Grayscale Path
        if (this->config_.input_type == kInputTypeFloat32) {
            scale_success = this->process_grayscale_crop_and_scale_to_float32(
                processing_buf, zone, crop_width, crop_height,
                output_buffer, this->config_.model_width, this->config_.model_height,
                this->config_.model_channels, this->config_.normalize, stride);
        } else if (this->config_.input_type == kInputTypeUInt8) {
            scale_success = this->process_grayscale_crop_and_scale_to_uint8(
                processing_buf, zone, crop_width, crop_height,
                output_buffer, this->config_.model_width, this->config_.model_height,
                this->config_.model_channels, stride);
        }
    } else {
        // RGB Path (Default)
        if (this->config_.input_type == kInputTypeFloat32) {
            scale_success = this->process_rgb888_crop_and_scale_to_float32(
                processing_buf, zone, crop_width, crop_height,
                output_buffer, this->config_.model_width, this->config_.model_height,
                this->config_.model_channels, this->config_.normalize, stride);
        } else if (this->config_.input_type == kInputTypeUInt8) {
            scale_success = this->process_rgb888_crop_and_scale_to_uint8(
                processing_buf, zone, crop_width, crop_height,
                output_buffer, this->config_.model_width, this->config_.model_height,
                this->config_.model_channels, stride);
        }
    }
    
    if (!scale_success) {
        ESP_LOGE(TAG, "Scaling failed.");
    }
    
    // Cleanup handled by RAII (rotated_buffer_storage and full_image_buf)

    return scale_success;

}

ImageProcessor::ProcessResult ImageProcessor::process_zone(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone) {
    
    DURATION_START();
    ProcessResult result;
    
    if (!this->validate_input_image(image)) return result;
    if (!this->validate_zone(zone)) return result;

    size_t required_size = this->get_required_buffer_size();
    if (required_size == 0) return result;

    UniqueBufferPtr buffer = this->allocate_image_buffer(required_size);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate output buffer size: %zu", required_size);
        return result; 
    }
    
    bool success = false;
    
    if (this->config_.pixel_format == "JPEG") {
        CropZone adjusted_zone = adjust_zone_for_jpeg(zone, this->config_.camera_width, this->config_.camera_height);
        success = this->process_jpeg_zone_to_buffer(image, adjusted_zone, buffer->get(), required_size);
    } else {
        success = this->process_raw_zone_to_buffer(image, zone, buffer->get(), required_size);
    }
    
    if (success) {
        result.data = std::move(buffer);
        result.size = required_size;
    }
    
    DURATION_END("process_zone");
    return result;
}

bool ImageProcessor::validate_zone(const CropZone &zone) const {
    int max_w = this->config_.camera_width;
    int max_h = this->config_.camera_height;
    
    // Calculate effective max dimensions after rotation
    #ifdef USE_CAMERA_ROTATOR
    if (this->config_.pixel_format == "JPEG" && std::abs(this->config_.rotation) > 0.01f) {
        float rads = this->config_.rotation * static_cast<float>(M_PI) / 180.0f;
        float abs_cos = std::abs(std::cos(rads));
        float abs_sin = std::abs(std::sin(rads));
        
        // Calculate new bounding box size
        int rot_w = static_cast<int>(this->config_.camera_width * abs_cos + this->config_.camera_height * abs_sin);
        int rot_h = static_cast<int>(this->config_.camera_width * abs_sin + this->config_.camera_height * abs_cos);
        
        max_w = rot_w;
        max_h = rot_h;
    }
    #endif

    if (zone.x1 < 0 || zone.y1 < 0 || 
        zone.x2 > max_w || zone.y2 > max_h) {
        ESP_LOGE(TAG, "Zone validation failed: [%d,%d->%d,%d] vs Max [%d,%d] (Rot: %.1f)", 
                 zone.x1, zone.y1, zone.x2, zone.y2, max_w, max_h, this->config_.rotation);
        return false;
    }
    
    if (zone.x2 <= zone.x1 || zone.y2 <= zone.y1) {
        ESP_LOGE(TAG, "Zone validation failed: Invalid dimensions");
        return false;
    }
    
    return true;
}

bool ImageProcessor::process_raw_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t *output_buffer,
    size_t output_buffer_size) {

    if (!this->validate_input_image(image)) return false;
    
    return this->process_raw_zone_pointer(
        image->get_data_buffer(),
        image->get_data_length(),
        this->config_.camera_width,
        this->config_.camera_height,
        zone, 
        output_buffer, 
        output_buffer_size
    );
}

bool ImageProcessor::process_raw_zone_pointer(
    const uint8_t* input_data, 
    size_t input_len, 
    int width, 
    int height, 
    const CropZone &zone, 
    uint8_t *output_buffer, 
    size_t output_buffer_size) {

    // Validate buffer size against requirements
    size_t required_size = this->get_required_buffer_size();
    if (!this->validate_buffer_size(required_size, output_buffer_size, "Raw zone processing")) {
        return false;
    }

    int crop_width = zone.x2 - zone.x1;
    int crop_height = zone.y2 - zone.y1;

    // Check bounds
    if (zone.x1 < 0 || zone.y1 < 0 || zone.x2 > width || zone.y2 > height) {
        ESP_LOGE(TAG, "Zone out of bounds for raw processing: [%d,%d->%d,%d] in %dx%d", 
                 zone.x1, zone.y1, zone.x2, zone.y2, width, height);
        return false;
    }

    // Determine Stride and Channels
    int stride = width; 
    int channels = 1;

    if (this->config_.pixel_format == "RGB888") {
        stride = width * 3;
        channels = 3;
    } else if (this->config_.pixel_format == "RGB565") {
        stride = width * 2;
        channels = 2;
    } else if (this->config_.pixel_format == "YUV422") {
        stride = width * 2;
        channels = 2; // Packed
    } else if (this->config_.pixel_format == "GRAYSCALE") {
        stride = width;
        channels = 1;
    } else {
        ESP_LOGE(TAG, "Unsupported format for raw processing: %s", this->config_.pixel_format.c_str());
        return false;
    }

    // Determine target dimensions for the scaling step
    // If rotating 90 or 270, we need to swap dimensions for the intermediate (unrotated) buffer
    int scale_width = this->config_.model_width;
    int scale_height = this->config_.model_height;
    
    bool needs_rotation = (this->config_.rotation != ROTATION_0);
    uint8_t* target_buffer = output_buffer;
    UniqueBufferPtr temp_buffer_storage;

#ifdef USE_CAMERA_ROTATOR
    if (needs_rotation && (this->config_.rotation == ROTATION_90 || this->config_.rotation == ROTATION_270)) {
        std::swap(scale_width, scale_height);
    }
    
    // If rotating, we need a temporary buffer for the unrotated scaled image
    if (needs_rotation) {
        temp_buffer_storage = this->allocate_image_buffer(output_buffer_size);
        if (!temp_buffer_storage) {
            ESP_LOGE(TAG, "Failed to allocate temp buffer for rotation");
            return false;
        }
        target_buffer = temp_buffer_storage->get();
    }
#endif

    // Dispatch
    bool success = false;
    if (this->config_.pixel_format == "RGB888") {
        if (this->config_.input_type == kInputTypeFloat32) {
            success = this->process_rgb888_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels,
                this->config_.normalize, width); // Use original width for stride
        } else if (this->config_.input_type == kInputTypeUInt8) {
            success = this->process_rgb888_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels, width); // Use original width for stride
        }
    } else if (this->config_.pixel_format == "RGB565") {
        if (this->config_.input_type == kInputTypeFloat32) {
            success = this->process_rgb565_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels,
                this->config_.normalize, width); // Use original width for stride
        } else if (this->config_.input_type == kInputTypeUInt8) {
            success = this->process_rgb565_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels, width); // Use original width for stride
        }
    } else if (this->config_.pixel_format == "GRAYSCALE") {
        if (this->config_.input_type == kInputTypeFloat32) {
            success = this->process_grayscale_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels,
                this->config_.normalize, width); // Use original width for stride
        } else if (this->config_.input_type == kInputTypeUInt8) {
            success = this->process_grayscale_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                target_buffer, scale_width, scale_height, this->config_.model_channels, this->config_.camera_width);
        }
    } else {
        ESP_LOGE(TAG, "ProcessRawZone: Unsupported pixel format '%s'", this->config_.pixel_format.c_str());
        return false;
    }

    // Apply rotation if needed
    // Apply rotation if needed
    #ifdef USE_CAMERA_ROTATOR
    if (success && needs_rotation) {
        int element_size = (this->config_.input_type == kInputTypeFloat32) ? sizeof(float) : sizeof(uint8_t);
        int bytes_per_pixel = this->config_.model_channels * element_size;
        
        // rotate temp_buffer (scale_width x scale_height) -> output_buffer (model_width x model_height)
        int rotated_w, rotated_h;
        success = this->apply_software_rotation(target_buffer, output_buffer, 
                                        scale_width, scale_height, 
                                        bytes_per_pixel, this->config_.rotation, rotated_w, rotated_h);
    }
    #endif
    
    // cleanup automatic via UniqueBufferPtr

    return success;
}

// Implementation of helper functions (simplified for brevity, assuming standard bilinear/nearest)
// Note: In a real implementation, these would contain the actual pixel manipulation logic
// I will include the scaling logic from the original file.

bool ImageProcessor::scale_rgb888_to_float32(
    const uint8_t* src, int src_w, int src_h,
    uint8_t* dst, int dst_w, int dst_h, int channels, bool normalize) {
    
#ifdef USE_CAMERA_SCALER
    return Scaler::scale_rgb888_to_float32(src, src_w, src_h, dst, dst_w, dst_h, channels, normalize);
#else
    float* dst_float = reinterpret_cast<float*>(dst);
    float scale_x = static_cast<float>(src_w) / dst_w;
    float scale_y = static_cast<float>(src_h) / dst_h;
    
    for (int y = 0; y < dst_h; y++) {
        for (int x = 0; x < dst_w; x++) {
            int src_x = static_cast<int>(x * scale_x);
            int src_y = static_cast<int>(y * scale_y);
            
            int src_idx = (src_y * src_w + src_x) * 3;
            int dst_idx = (y * dst_w + x) * channels;
            
            if (channels == 3) {
                dst_float[dst_idx] = normalize ? src[src_idx] / 255.0f : src[src_idx];
                dst_float[dst_idx+1] = normalize ? src[src_idx+1] / 255.0f : src[src_idx+1];
                dst_float[dst_idx+2] = normalize ? src[src_idx+2] / 255.0f : src[src_idx+2];
            } else if (channels == 1) {
                // Convert to grayscale
                float gray = 0.299f * src[src_idx] + 0.587f * src[src_idx+1] + 0.114f * src[src_idx+2];
                dst_float[dst_idx] = normalize ? gray / 255.0f : gray;
            }
        }
    }
    return true;
#endif
}

bool ImageProcessor::scale_rgb888_to_uint8(
    const uint8_t* src, int src_w, int src_h,
    uint8_t* dst, int dst_w, int dst_h, int channels) {
    
#ifdef USE_CAMERA_SCALER
    return Scaler::scale_rgb888_to_uint8(src, src_w, src_h, dst, dst_w, dst_h, channels);
#else
    float scale_x = static_cast<float>(src_w) / dst_w;
    float scale_y = static_cast<float>(src_h) / dst_h;
    
    for (int y = 0; y < dst_h; y++) {
        for (int x = 0; x < dst_w; x++) {
            int src_x = static_cast<int>(x * scale_x);
            int src_y = static_cast<int>(y * scale_y);
            
            int src_idx = (src_y * src_w + src_x) * 3;
            int dst_idx = (y * dst_w + x) * channels;
            
            if (channels == 3) {
                dst[dst_idx] = src[src_idx];
                dst[dst_idx+1] = src[src_idx+1];
                dst[dst_idx+2] = src[src_idx+2];
            } else if (channels == 1) {
                dst[dst_idx] = static_cast<uint8_t>(0.299f * src[src_idx] + 0.587f * src[src_idx+1] + 0.114f * src[src_idx+2]);
            }
        }
    }
    return true;
#endif
}

// Stub implementations for other formats to save space, as they follow similar patterns
// In a full implementation, these would be populated with the logic from the original file

bool ImageProcessor::process_rgb888_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int channels, bool normalize, int src_stride_width) {
    
    // Create a temporary buffer pointing to the crop start is tricky because of stride
    // So we do crop and scale in one go
    float* float_output = reinterpret_cast<float*>(output_buffer);
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image using dynamic stride
            int src_pos = ((zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x)) * 3;
            int dst_pos = (y * model_width + x) * channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&float_output[dst_pos], r, g, b, channels, normalize);
        }
    }
    
    #ifdef DEBUG_ESP32_CAMERA_UTILS
    DEBUG_FIRST_PIXELS(float_output, model_width * model_height * channels, channels);
    DEBUG_CHANNEL_ORDER(float_output, model_width * model_height * channels, channels);
    #endif

    return true;
}

bool ImageProcessor::process_rgb888_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone& zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int channels, int src_stride_width) {
    
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image using dynamic stride
            int src_pos = ((zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x)) * 3;
            int dst_pos = (y * model_width + x) * channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&output_buffer[dst_pos], r, g, b, channels);
        }
    }

    #ifdef DEBUG_ESP32_CAMERA_UTILS
    // Manual logging can be added here if needed
    #endif

    return true;
}

bool ImageProcessor::process_rgb565_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels, bool normalize, int src_stride_width) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    const uint16_t* rgb565_data = reinterpret_cast<const uint16_t*>(input_data);
    
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            int src_pos = (zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x);
            uint16_t pixel = rgb565_data[src_pos];
            
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            int dst_pos = (y * model_width + x) * model_channels;
            arrange_channels(&float_output[dst_pos], r, g, b, model_channels, normalize);
        }
    }
    return true; 
}

bool ImageProcessor::process_rgb565_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels, int src_stride_width) {
    
    const uint16_t* rgb565_data = reinterpret_cast<const uint16_t*>(input_data);
    
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            int src_pos = (zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x);
            uint16_t pixel = rgb565_data[src_pos];
            
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            int dst_pos = (y * model_width + x) * model_channels;
            arrange_channels(&output_buffer[dst_pos], r, g, b, model_channels);
        }
    }
    return true;
}

bool ImageProcessor::process_grayscale_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels, bool normalize, int src_stride_width) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            int src_pos = (zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x);
            uint8_t gray = input_data[src_pos];
            
            int dst_pos = (y * model_width + x) * model_channels;
            
            if (model_channels >= 3) {
                float_output[dst_pos] = normalize ? gray / 255.0f : gray;
                float_output[dst_pos + 1] = normalize ? gray / 255.0f : gray;
                float_output[dst_pos + 2] = normalize ? gray / 255.0f : gray;
            } else if (model_channels == 1) {
                float_output[dst_pos] = normalize ? gray / 255.0f : gray;
            }
        }
    }
    return true;
}

bool ImageProcessor::process_grayscale_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone &zone, int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels, int src_stride_width) {
    
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            int src_pos = (zone.y1 + src_y) * src_stride_width + (zone.x1 + src_x);
            uint8_t gray = input_data[src_pos];
            
            int dst_pos = (y * model_width + x) * model_channels;
            
            if (model_channels >= 3) {
                output_buffer[dst_pos] = gray;
                output_buffer[dst_pos + 1] = gray;
                output_buffer[dst_pos + 2] = gray;
            } else if (model_channels == 1) {
                output_buffer[dst_pos] = gray;
            }
        }
    }
    return true;
}

void ImageProcessor::arrange_channels(float* output, uint8_t r, uint8_t g, uint8_t b, 
                                    int output_channels, bool normalize) const {
    if (output_channels >= 3) {
        if (this->config_.input_order == "BGR") {
            output[0] = normalize ? b / 255.0f : b;
            output[1] = normalize ? g / 255.0f : g;
            output[2] = normalize ? r / 255.0f : r;
        } else { // Default to RGB
            output[0] = normalize ? r / 255.0f : r;
            output[1] = normalize ? g / 255.0f : g;
            output[2] = normalize ? b / 255.0f : b;
        }
    } else if (output_channels == 1) {
        float gray = 0.299f * r + 0.587f * g + 0.114f * b;
        output[0] = normalize ? gray / 255.0f : gray;
    }
}

void ImageProcessor::arrange_channels(uint8_t* output, uint8_t r, uint8_t g, uint8_t b, 
                                    int output_channels) const {
    if (output_channels >= 3) {
        if (this->config_.input_order == "BGR") {
            output[0] = b;
            output[1] = g;
            output[2] = r;
        } else { // Default to RGB
            output[0] = r;
            output[1] = g;
            output[2] = b;
        }
    } else if (output_channels == 1) {
        output[0] = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
    }
}

#ifdef USE_CAMERA_ROTATOR
bool ImageProcessor::apply_software_rotation(
    const uint8_t* input, uint8_t* output,
    int width, int height, int bytes_per_pixel,
    float rotation_deg, int& out_w, int& out_h) {
    
#ifdef USE_CAMERA_ROTATOR
    // Delegate to modular Rotator
    return Rotator::perform_rotation(input, output, width, height, bytes_per_pixel, rotation_deg, out_w, out_h);
#else
    // Sanity check to prevent infinite loop
    if (!std::isfinite(rotation_deg)) {
        ESP_LOGE("ImageProcessor", "Invalid rotation value: %.2f", rotation_deg);
        return false;
    }

    // Normalize rotation to 0-360 positive
    float rot = rotation_deg;
    while (rot < 0) rot += 360.0f;
    while (rot >= 360.0f) rot -= 360.0f;

    // Handle standard rotations
    // Use a small epsilon for float comparison
    if (rot < 0.1f || rot > 359.9f) {
        // 0 degrees
        out_w = width;
        out_h = height;
        memcpy(output, input, width * height * bytes_per_pixel);
        return true;
    }
    
    if (std::abs(rot - 90.0f) < 0.1f) {
        // 90 degrees
        out_w = height;
        out_h = width;
        for (int y = 0; y < height; y++) {
            esphome::App.feed_wdt();
            for (int x = 0; x < width; x++) {
                // 90 deg: (x, y) -> (height - 1 - y, x)
                int src_idx = (y * width + x) * bytes_per_pixel;
                int dst_idx = (x * out_w + (height - 1 - y)) * bytes_per_pixel;
                memcpy(output + dst_idx, input + src_idx, bytes_per_pixel);
            }
        }
        return true;
    }

    if (std::abs(rot - 180.0f) < 0.1f) {
        // 180 degrees
        out_w = width;
        out_h = height;
        for (int y = 0; y < height; y++) {
            esphome::App.feed_wdt();
            for (int x = 0; x < width; x++) {
                // 180 deg: (x, y) -> (width - 1 - x, height - 1 - y)
                int src_idx = (y * width + x) * bytes_per_pixel;
                int dst_idx = ((height - 1 - y) * width + (width - 1 - x)) * bytes_per_pixel;
                memcpy(output + dst_idx, input + src_idx, bytes_per_pixel);
            }
        }
        return true;
    }

    if (std::abs(rot - 270.0f) < 0.1f) {
        // 270 degrees
        out_w = height;
        out_h = width;
        for (int y = 0; y < height; y++) {
            esphome::App.feed_wdt();
            for (int x = 0; x < width; x++) {
                // 270 deg: (x, y) -> (y, width - 1 - x)
                int src_idx = (y * width + x) * bytes_per_pixel;
                int dst_idx = ((width - 1 - x) * out_w + y) * bytes_per_pixel;
                memcpy(output + dst_idx, input + src_idx, bytes_per_pixel);
            }
        }
        return true;
    }
    
    // Fallback for arbitrary rotation (Nearest Neighbor) if needed or return false if strict
   
    // Calculate new dimensions (bounding box)
    float rads = rot * M_PI / 180.0f;
    float c = std::cos(rads);
    float s = std::sin(rads);
    float abs_c = std::abs(c);
    float abs_s = std::abs(s);
    
    out_w = static_cast<int>(width * abs_c + height * abs_s);
    out_h = static_cast<int>(width * abs_s + height * abs_c);

    int cx = width / 2;
    int cy = height / 2;
    int ncx = out_w / 2;
    int ncy = out_h / 2;

    for (int y = 0; y < out_h; y++) {
        esphome::App.feed_wdt();
        for (int x = 0; x < out_w; x++) {
            // Reverse map:
            // x_src = (x - ncx)*c + (y - ncy)*s + cx
            // y_src = -(x - ncx)*s + (y - ncy)*c + cy
            // Note: signs depend on coordinate system.
            
            int tx = x - ncx;
            int ty = y - ncy;
            
            int xs = static_cast<int>(tx * c + ty * s + cx);
            int ys = static_cast<int>(-tx * s + ty * c + cy);
            
            if (xs >= 0 && xs < width && ys >= 0 && ys < height) {
                 int src_idx = (ys * width + xs) * bytes_per_pixel;
                 int dst_idx = (y * out_w + x) * bytes_per_pixel;
                 memcpy(output + dst_idx, input + src_idx, bytes_per_pixel);
            } else {
                 // Fill black
                 int dst_idx = (y * out_w + x) * bytes_per_pixel;
                 memset(output + dst_idx, 0, bytes_per_pixel);
            }
        }
    }

    return true;
#endif
}
#endif
    


#ifdef DEBUG_ESP32_CAMERA_UTILS
// Debug functions implementation

void ImageProcessor::debug_log_image_stats(const uint8_t* data, size_t size,
                                         const char* stage) {
    if (!data || size == 0) return;
    
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    uint32_t sum = 0;
    int zero_count = 0;
    
    for (size_t i = 0; i < size; i++) {
        min_val = std::min(min_val, data[i]);
        max_val = std::max(max_val, data[i]);
        sum += data[i];
        if (data[i] == 0) zero_count++;
    }
    
    float mean = static_cast<float>(sum) / size;
    
    ESP_LOGD(TAG, "DEBUG %s: size=%zu, min=%u, max=%u, mean=%.1f, zeros=%d/%zu (%.1f%%)",
             stage, size, min_val, max_val, mean, 
             zero_count, size, (zero_count * 100.0f) / size);
             
    ESP_LOGD(TAG, "DEBUG %s first 10 values:", stage);
    char buf[64]; // Sufficient for 10 * 4 chars ("255 ")
    size_t offset = 0;
    for (int i = 0; i < std::min(10, static_cast<int>(size)); i++) {
        int w = snprintf(buf + offset, sizeof(buf) - offset, "%u ", data[i]);
        if (w < 0 || static_cast<size_t>(w) >= sizeof(buf) - offset) break;
        offset += w;
    }
    ESP_LOGD(TAG, "  %s", buf);
}

void ImageProcessor::debug_log_float_stats(const float* data, size_t count,
                                         const char* stage) {
    if (!data || count == 0) return;
    
    float min_val = 1e9;
    float max_val = -1e9;
    float sum = 0.0f;
    int zero_count = 0;
    
    for (size_t i = 0; i < count; i++) {
        min_val = std::min(min_val, data[i]);
        max_val = std::max(max_val, data[i]);
        sum += data[i];
    }
    
    float mean = sum / count;
    
    ESP_LOGD(TAG, "DEBUG %s: count=%zu, min=%.3f, max=%.3f, mean=%.3f",
             stage, count, min_val, max_val, mean);
             
    char buf[128]; // Sufficient for 10 * 8 chars ("1.234 ")
    size_t offset = 0;
    for (int i = 0; i < std::min(10, static_cast<int>(count)); i++) {
        int w = snprintf(buf + offset, sizeof(buf) - offset, "%.3f ", data[i]);
        if (w < 0 || static_cast<size_t>(w) >= sizeof(buf) - offset) break;
        offset += w;
    }
    ESP_LOGD(TAG, "  %s", buf);
}

void ImageProcessor::debug_log_image(const uint8_t* data, size_t size, 
                                   int width, int height, int channels,
                                   const char* stage) {
    if (!data || size == 0) return;
    ESP_LOGD(TAG, "DEBUG %s: %dx%dx%d (%zu bytes)", 
             stage, width, height, channels, size);
    debug_log_image_stats(data, size, stage);
}

void ImageProcessor::debug_log_float_image(const float* data, size_t count,
                                         int width, int height, int channels,
                                         const char* stage) {
    if (!data || count == 0) return;
    ESP_LOGD(TAG, "DEBUG %s: %dx%dx%d (%zu floats)", 
             stage, width, height, channels, count);
    debug_log_float_stats(data, count, stage);
}

void ImageProcessor::debug_log_rgb888_image(const uint8_t* data, 
                                          int width, int height,
                                          const char* stage) {
    debug_log_image(data, width * height * 3, width, height, 3, stage);
}

void ImageProcessor::debug_analyze_processed_zone(const uint8_t* data, 
                                                 int width, int height, 
                                                 int channels,
                                                 const char* zone_name) {
    if (!data || width <= 0 || height <= 0) return;
    
    ESP_LOGI(TAG, "ZONE_ANALYSIS:%s:%dx%dx%d", zone_name, width, height, channels);
    /* Too verbose for full analysis every time, enabled only for small check */
    if (width <= 10 && height <= 10) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pos = (y * width + x) * channels;
                ESP_LOGI(TAG, "  Pixel[%d,%d] ch0=%u", x, y, data[pos]);
            }
        }
    }
}

void ImageProcessor::debug_analyze_float_zone(const float* data, 
                                             int width, int height, 
                                             int channels,
                                             const char* zone_name,
                                             bool normalized) {
    if (!data || width <= 0 || height <= 0) return;
    debug_log_float_stats(data, width * height * channels, zone_name);
}

void ImageProcessor::debug_output_zone_preview(const uint8_t* data,
                                              int width, int height,
                                              int channels,
                                              const char* zone_name) {
    if (width > 64 || height > 64) return; // Too big for log
    
    ESP_LOGI(TAG, "PREVIEW:%s", zone_name);
    char line[70]; // 64 chars + null
    for (int y = 0; y < height; y++) {
        int x;
        for (x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            uint8_t val = data[pos]; // Take first channel (or grayscale)
            char c = val > 128 ? '#' : '.';
            line[x] = c;
        }
        line[x] = '\0';
        ESP_LOGI(TAG, "%s", line);
    }
}

void ImageProcessor::debug_output_float_preview(const float* data,
                                               int width, int height,
                                               int channels,
                                               const char* zone_name,
                                               bool normalized) {
     if (width > 64 || height > 64) return;
     
     ESP_LOGI(TAG, "FLOAT_PREVIEW:%s", zone_name);
     char line[70];
     for (int y = 0; y < height; y++) {
        int x;
        for (x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            float val = data[pos];
            if (normalized) val *= 255.0f;
            char c = val > 128.0f ? '#' : '.';
            line[x] = c;
        }
        line[x] = '\0';
        ESP_LOGI(TAG, "%s", line);
    }
}
#endif

}  // namespace esp32_camera_utils
}  // namespace esphome
