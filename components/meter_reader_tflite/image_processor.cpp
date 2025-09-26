/**
 * @file image_processor.cpp
 * @brief Implements the ImageProcessor class for handling image manipulation
 *        such as cropping, scaling, and format conversion for TensorFlow Lite.
 */

// Prevent old JPEG headers from being included
#define ESP_JPEG_DEC_H
#define ESP_JPEG_COMMON_H

// Include esp_new_jpeg FIRST, before any ESPHome camera headers
#include <cstdint> //// for uint8_t, uint16_t (needed by esp new jpeg)
#include "esp_jpeg_dec.h"
#include "esp_jpeg_common.h"

// Then include other headers
#include "image_processor.h"
// #include "esp_log.h"
#include "debug_utils.h"
#include <algorithm>
// #include <cstring>


#ifdef DEBUG_METER_READER_TFLITE
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
            /* Simple heuristic for BGR vs RGB */ \
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


namespace esphome {
namespace meter_reader_tflite {

static const char *const TAG = "ImageProcessor";

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

/**
 * @brief Validates buffer size for the given context.
 */
bool ImageProcessor::validate_buffer_size(size_t required, size_t available, const char* context) const {
    if (available < required) {
        ESP_LOGE(TAG, "Buffer too small for %s: need %zu bytes, have %zu bytes", 
                context, required, available);
        return false;
    }
    return true;
}

/**
 * @brief Validates input image for processing.
 */
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
    
    // Additional format-specific validation
    if (config_.pixel_format == "JPEG") {
        // Basic JPEG signature check
        if (image->get_data_length() < 2) {
            ESP_LOGE(TAG, "JPEG data too short");
            return false;
        }
        
        const uint8_t* data = image->get_data_buffer();
        if (data[0] != 0xFF || data[1] != 0xD8) {
            ESP_LOGW(TAG, "Invalid JPEG signature: 0x%02X 0x%02X", data[0], data[1]);
            // Still try to process, but warn
        }
    }
    
    return true;
}

/**
 * @brief Adjusts crop zone to nearest multiple of 8 for JPEG decoder requirements.
 */
CropZone adjust_zone_for_jpeg(const CropZone &zone, int max_width, int max_height) {
    CropZone adjusted = zone;
    
    // Adjust width to nearest multiple of 8
    int width = zone.x2 - zone.x1;
    int adjusted_width = (width + 4) / 8 * 8; // Round to nearest multiple of 8
    if (adjusted_width > width) {
        adjusted_width -= 8; // Ensure we don't exceed original bounds
    }
    adjusted_width = std::max(8, adjusted_width); // Minimum 8 pixels
    
    // Adjust height to nearest multiple of 8
    int height = zone.y2 - zone.y1;
    int adjusted_height = (height + 4) / 8 * 8; // Round to nearest multiple of 8
    if (adjusted_height > height) {
        adjusted_height -= 8;
    }
    adjusted_height = std::max(8, adjusted_height); // Minimum 8 pixels
    
    // Center the adjusted zone within original bounds
    adjusted.x1 = zone.x1 + (width - adjusted_width) / 2;
    adjusted.y1 = zone.y1 + (height - adjusted_height) / 2;
    adjusted.x2 = adjusted.x1 + adjusted_width;
    adjusted.y2 = adjusted.y1 + adjusted_height;
    
    // Ensure within image bounds
    adjusted.x1 = std::max(0, adjusted.x1);
    adjusted.y1 = std::max(0, adjusted.y1);
    adjusted.x2 = std::min(max_width, adjusted.x2);
    adjusted.y2 = std::min(max_height, adjusted.y2);
    
    // Final adjustment to ensure multiples of 8
    int final_width = adjusted.x2 - adjusted.x1;
    int final_height = adjusted.y2 - adjusted.y1;
    final_width = (final_width / 8) * 8;
    final_height = (final_height / 8) * 8;
    adjusted.x2 = adjusted.x1 + final_width;
    adjusted.y2 = adjusted.y1 + final_height;
    
    ESP_LOGD(TAG, "Adjusted zone from [%d,%d,%d,%d] to [%d,%d,%d,%d] for JPEG",
             zone.x1, zone.y1, zone.x2, zone.y2,
             adjusted.x1, adjusted.y1, adjusted.x2, adjusted.y2);
    
    return adjusted;
}

/**
 * @brief Constructs an ImageProcessor object.
 */
ImageProcessor::ImageProcessor(const ImageProcessorConfig &config, 
                             ModelHandler* model_handler)
  : config_(config), model_handler_(model_handler) {
  
  if (!config_.validate()) {
    ESP_LOGE(TAG, "Invalid image processor configuration");
    ESP_LOGE(TAG, "  Camera: %dx%d, Format: %s", 
             config_.camera_width, config_.camera_height, config_.pixel_format.c_str());
  }

  // Determine bytes per pixel based on the camera pixel format.
  if (config_.pixel_format == "RGB888") {
    bytes_per_pixel_ = 3;
    ESP_LOGD(TAG, "Using RGB888 format (3 bytes per pixel)");
  } else if (config_.pixel_format == "RGB565") {
    bytes_per_pixel_ = 2;
    ESP_LOGD(TAG, "Using RGB565 format (2 bytes per pixel)");
  } else if (config_.pixel_format == "YUV422") {
    bytes_per_pixel_ = 2;
    ESP_LOGD(TAG, "Using YUV422 format (2 bytes per pixel)");
  } else if (config_.pixel_format == "GRAYSCALE") {
    bytes_per_pixel_ = 1;
    ESP_LOGD(TAG, "Using grayscale format (1 byte per pixel)");
  } else if (config_.pixel_format == "JPEG") {
    bytes_per_pixel_ = 3; // JPEG will be decoded to RGB888.
    ESP_LOGD(TAG, "Using JPEG format (will decode to RGB888)");
  } else {
    ESP_LOGE(TAG, "Unsupported pixel format: %s", config_.pixel_format.c_str());
    bytes_per_pixel_ = 3; // Default to RGB888 for unsupported formats.
    ESP_LOGW(TAG, "Defaulting to RGB888 format");
  }

  ESP_LOGD(TAG, "ImageProcessor initialized with:");
  ESP_LOGD(TAG, "  Camera resolution: %dx%d", config_.camera_width, config_.camera_height);
  ESP_LOGD(TAG, "  Model input: %dx%dx%d", 
           model_handler_->get_input_width(),
           model_handler_->get_input_height(),
           model_handler_->get_input_channels());
  ESP_LOGD(TAG, "  Normalize: %s", model_handler_->get_config().normalize ? "true" : "false");
  ESP_LOGD(TAG, "  Input order: %s", model_handler_->get_config().input_order.c_str()); 
}

/**
 * @brief Splits the input image into specified crop zones and processes each.
 */
std::vector<ImageProcessor::ProcessResult> ImageProcessor::split_image_in_zone(
    std::shared_ptr<camera::CameraImage> image,
    const std::vector<CropZone> &zones) {
   
  std::lock_guard<std::mutex> lock(processing_mutex_);
  std::vector<ProcessResult> results;
  
  stats_.total_frames++;
  uint32_t start_time = millis();
  
  ESP_LOGD(TAG, "Starting image processing");
  ESP_LOGD(TAG, "Input image size: %zu bytes", image->get_data_length());

  // Validate input image
  if (!validate_input_image(image)) {
    ESP_LOGE(TAG, "Invalid input image");
    stats_.failed_frames++;
    return results;
  }

  // If no zones are specified, process the full image.
  if (zones.empty()) {
    ESP_LOGD(TAG, "No zones specified - processing full image");
    CropZone full_zone{0, 0, config_.camera_width, config_.camera_height};
    ProcessResult result = process_zone(image, full_zone);
    if (result.data) {
      ESP_LOGD(TAG, "Full image processed successfully (%zu bytes)", result.size);
      results.push_back(std::move(result));
    } else {
      ESP_LOGE(TAG, "Failed to process full image");
      stats_.failed_frames++;
    }
  } else {
    // Process each specified crop zone.
    ESP_LOGD(TAG, "Processing %d crop zones", zones.size());
    bool all_zones_successful = true;
    
    for (size_t i = 0; i < zones.size(); i++) {
      ESP_LOGD(TAG, "Processing zone %d: [%d,%d,%d,%d]", 
              i+1, zones[i].x1, zones[i].y1, zones[i].x2, zones[i].y2);
      
      ProcessResult result = process_zone(image, zones[i]);
      if (result.data) {
        ESP_LOGD(TAG, "Zone %d processed successfully (%zu bytes)", i+1, result.size);
        results.push_back(std::move(result));
      } else {
        ESP_LOGE(TAG, "Failed to process zone %d", i+1);
        all_zones_successful = false;
      }
    }
    
    if (!all_zones_successful) {
      stats_.failed_frames++;
    }
  }
  
  // Update statistics
  uint32_t processing_time = millis() - start_time;
  stats_.total_processing_time_ms += processing_time;
  
  ESP_LOGD(TAG, "Image processing complete. Generated %d outputs in %lums", 
           results.size(), processing_time);
  ESP_LOGD(TAG, "Success rate: %.1f%%, Avg time: %.1fms", 
           stats_.get_success_rate(), stats_.get_avg_processing_time());
  
  return results;
}

/**
 * @brief Processes a zone directly into a pre-allocated buffer for minimal memory usage.
 */
bool ImageProcessor::process_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t* output_buffer,
    size_t output_buffer_size) {
    
    std::lock_guard<std::mutex> lock(processing_mutex_);
    DURATION_START();
    
    if (!validate_input_image(image)) {
        ESP_LOGE(TAG, "Invalid input image");
        return false;
    }
    
    if (!validate_zone(zone)) {
        ESP_LOGE(TAG, "Invalid zone [%d,%d,%d,%d]", zone.x1, zone.y1, zone.x2, zone.y2);
        return false;
    }

    bool success = false;
    
    if (config_.pixel_format == "JPEG") {
        // For JPEG, we need to adjust the zone to multiples of 8
        CropZone adjusted_zone = adjust_zone_for_jpeg(zone, config_.camera_width, config_.camera_height);
        success = process_jpeg_zone_to_buffer(image, adjusted_zone, output_buffer, output_buffer_size);
    } else {
        success = process_raw_zone_to_buffer(image, zone, output_buffer, output_buffer_size);
    }
    
    DURATION_END("process_zone_to_buffer");
    return success;
}


/**
 * @brief Processes JPEG zone by decoding the full image first, then cropping and scaling to model's input type
 */
bool ImageProcessor::process_jpeg_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t* output_buffer,
    size_t output_buffer_size) {
    
    const uint8_t *jpeg_data = image->get_data_buffer();
    size_t jpeg_size = image->get_data_length();
    
    if (!jpeg_data || jpeg_size == 0) {
        ESP_LOGE(TAG, "Invalid JPEG data");
        return false;
    }

    // Get model input dimensions and type
    const int model_width = model_handler_->get_input_width();
    const int model_height = model_handler_->get_input_height();
    const int model_channels = model_handler_->get_input_channels();
    
    // Get model input tensor to determine expected data type and size
    TfLiteTensor* input_tensor = model_handler_->input_tensor();
    if (!input_tensor) {
        ESP_LOGE(TAG, "Cannot determine model input type");
        return false;
    }
    
    TfLiteType input_type = input_tensor->type;
    size_t required_size = 0;
    
    if (input_type == kTfLiteFloat32) {
        required_size = model_width * model_height * model_channels * sizeof(float);
    } else if (input_type == kTfLiteUInt8) {
        required_size = model_width * model_height * model_channels;
    } else {
        ESP_LOGE(TAG, "Unsupported model input type: %d", input_type);
        return false;
    }
    
    // Verify output buffer size is sufficient
    if (!validate_buffer_size(required_size, output_buffer_size, "JPEG processing")) {
        ESP_LOGE(TAG, "Need %zu bytes for %s output, got %zu bytes", 
                 required_size, 
                 input_type == kTfLiteFloat32 ? "float32" : "uint8",
                 output_buffer_size);
        return false;
    }

    // Step 1: Decode the full JPEG image to RGB888
    ESP_LOGD(TAG, "Decoding full JPEG image first for %s input", 
             input_type == kTfLiteFloat32 ? "float32" : "uint8");
    
    jpeg_dec_config_t decode_config = DEFAULT_JPEG_DEC_CONFIG();
    decode_config.output_type = JPEG_PIXEL_FORMAT_RGB888;
    decode_config.scale.width = static_cast<uint16_t>(config_.camera_width);
    decode_config.scale.height = static_cast<uint16_t>(config_.camera_height);
    decode_config.clipper.width = static_cast<uint16_t>(config_.camera_width);
    decode_config.clipper.height = static_cast<uint16_t>(config_.camera_height);
    decode_config.rotate = JPEG_ROTATE_0D;
    decode_config.block_enable = false;
    
    // Add debug logging for config
    ESP_LOGD(TAG, "JPEG decoder config:");
    ESP_LOGD(TAG, "  output_type: %d", decode_config.output_type);
    ESP_LOGD(TAG, "  scale: %dx%d", decode_config.scale.width, decode_config.scale.height);
    ESP_LOGD(TAG, "  clipper: %dx%d", decode_config.clipper.width, decode_config.clipper.height);

    jpeg_dec_handle_t decoder = nullptr;
    jpeg_error_t ret = jpeg_dec_open(&decode_config, &decoder);
    
    if (ret != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open JPEG decoder: %s", jpeg_error_to_string(ret));
        stats_.jpeg_decoding_errors++;
        return false;
    }

    // Allocate 16-byte aligned buffer for full decoded image using jpeg_calloc_align
    size_t full_image_size = config_.camera_width * config_.camera_height * 3;
    uint8_t* full_image_buf = (uint8_t*)jpeg_calloc_align(full_image_size, 16);
    if (!full_image_buf) {
        ESP_LOGE(TAG, "Failed to allocate full image buffer with jpeg_calloc_align");
        jpeg_dec_close(decoder);
        return false;
    }

    // Prepare IO control for full decoding
    jpeg_dec_io_t io;
    memset(&io, 0, sizeof(io));
    io.inbuf = const_cast<uint8_t*>(jpeg_data);
    io.inbuf_len = jpeg_size;
    io.inbuf_remain = jpeg_size;
    io.outbuf = full_image_buf;
    io.out_size = full_image_size;

    // Parse header and decode full image
    jpeg_dec_header_info_t header_info;
    ret = jpeg_dec_parse_header(decoder, &io, &header_info);
    if (ret != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Failed to parse JPEG header: %s", jpeg_error_to_string(ret));
        jpeg_free_align(full_image_buf);
        jpeg_dec_close(decoder);
        stats_.jpeg_decoding_errors++;
        return false;
    }
    
    
    ESP_LOGD(TAG, "JPEG header info:");
    ESP_LOGD(TAG, "  width: %d, height: %d", header_info.width, header_info.height);
    // Note: components and MCU fields are not available in this version of the JPEG decoder

    ret = jpeg_dec_process(decoder, &io);
    if (ret != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Failed to decode JPEG: %s", jpeg_error_to_string(ret));
        jpeg_free_align(full_image_buf);
        jpeg_dec_close(decoder);
        stats_.jpeg_decoding_errors++;
        return false;
    }

    ESP_LOGD(TAG, "Full JPEG decoded successfully to %dx%d RGB888", 
             config_.camera_width, config_.camera_height);
             
#ifdef DEBUG_METER_READER_TFLITE
    debug_log_rgb888_image(full_image_buf, config_.camera_width, config_.camera_height, 
                      "jpeg_decoded_full");
#endif

    // Step 2: Crop from the full decoded RGB888 image
    int crop_width = zone.x2 - zone.x1;
    int crop_height = zone.y2 - zone.y1;
    
    // Validate crop zone
    if (zone.x1 < 0 || zone.y1 < 0 || 
        zone.x2 > config_.camera_width || zone.y2 > config_.camera_height) {
        ESP_LOGE(TAG, "Crop zone [%d,%d,%d,%d] out of bounds [0,0,%d,%d]",
                 zone.x1, zone.y1, zone.x2, zone.y2,
                 config_.camera_width, config_.camera_height);
        jpeg_free_align(full_image_buf);
        jpeg_dec_close(decoder);
        return false;
    }

    // Allocate temporary buffer for cropped RGB888
    size_t cropped_size = crop_width * crop_height * 3;
    uint8_t* cropped_buf = (uint8_t*)jpeg_calloc_align(cropped_size, 16);
    if (!cropped_buf) {
        ESP_LOGE(TAG, "Failed to allocate cropped buffer with jpeg_calloc_align");
        jpeg_free_align(full_image_buf);
        jpeg_dec_close(decoder);
        return false;
    }

    // Perform cropping
    for (int y = 0; y < crop_height; y++) {
        const uint8_t* src = full_image_buf + ((zone.y1 + y) * config_.camera_width + zone.x1) * 3;
        uint8_t* dst = cropped_buf + y * crop_width * 3;
        memcpy(dst, src, crop_width * 3);
    }
    
    
#ifdef DEBUG_METER_READER_TFLITE
    debug_log_rgb888_image(cropped_buf, crop_width, crop_height, 
                      "jpeg_cropped");
#endif

    // Free full image buffer
    jpeg_free_align(full_image_buf);
    jpeg_dec_close(decoder);

    // Step 3: Scale cropped RGB888 to model dimensions
    bool scale_success = false;
    
    if (input_type == kTfLiteFloat32) {
        scale_success = scale_rgb888_to_float32(cropped_buf, crop_width, crop_height,
                                              output_buffer, model_width, model_height,
                                              model_channels, model_handler_->get_config().normalize);
    } else if (input_type == kTfLiteUInt8) {
        scale_success = scale_rgb888_to_uint8(cropped_buf, crop_width, crop_height,
                                            output_buffer, model_width, model_height,
                                            model_channels);
    }

/* #ifdef DEBUG_METER_READER_TFLITE
    if (input_type == kTfLiteFloat32) {
        debug_log_float_image(reinterpret_cast<float*>(output_buffer), 
                             model_width * model_height * model_channels,
                             model_width, model_height, model_channels,
                             "jpeg_scaled_float");
    } else if (input_type == kTfLiteUInt8) {
        debug_log_image(output_buffer, required_size,
                       model_width, model_height, model_channels,
                       "jpeg_scaled_uint8");
}
#endif */

#ifdef DEBUG_METER_READER_TFLITE
    #ifdef DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL
        // Add ZONE_ANALYSIS debug output here
        if (input_type == kTfLiteFloat32) {
            debug_analyze_float_zone(reinterpret_cast<float*>(output_buffer), 
                                    model_width, model_height, model_channels,
                                    "jpeg_final_output", 
                                    model_handler_->get_config().normalize);
            // debug_output_float_preview(reinterpret_cast<float*>(output_buffer),
                                     // model_width, model_height, model_channels,
                                     // "jpeg_final_preview",
                                     // model_handler_->get_config().normalize);
        } else if (input_type == kTfLiteUInt8) {
            debug_analyze_processed_zone(output_buffer, 
                                       model_width, model_height, model_channels,
                                       "jpeg_final_output");
            // debug_output_zone_preview(output_buffer,
                                    // model_width, model_height, model_channels,
                                    // "jpeg_final_preview");
        }
    #endif
#endif

    // Free cropped buffer
    jpeg_free_align(cropped_buf);

    if (!scale_success) {
        ESP_LOGE(TAG, "Failed to scale cropped image");
        return false;
    }

    ESP_LOGD(TAG, "JPEG zone processed successfully: %dx%d -> %dx%d -> %dx%d",
             config_.camera_width, config_.camera_height,
             crop_width, crop_height,
             model_width, model_height);
             
    return true;
}

/**
 * @brief Processes a single crop zone from the input image.
 */
ImageProcessor::ProcessResult ImageProcessor::process_zone(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone) {
    
    DURATION_START();
    ProcessResult result;
    
    ESP_LOGD(TAG, "Processing zone [%d,%d,%d,%d]", zone.x1, zone.y1, zone.x2, zone.y2);
    ESP_LOGD(TAG, "Zone dimensions: %dx%d", zone.x2 - zone.x1, zone.y2 - zone.y1);

    // Validate input and zone
    if (!validate_input_image(image)) {
        ESP_LOGE(TAG, "Invalid input image");
        return result;
    }
    
    if (!validate_zone(zone)) {
        ESP_LOGE(TAG, "Invalid zone [%d,%d,%d,%d]", zone.x1, zone.y1, zone.x2, zone.y2);
        return result;
    }

    // Get model input dimensions
    const int model_width = model_handler_->get_input_width();
    const int model_height = model_handler_->get_input_height();
    const int model_channels = model_handler_->get_input_channels();
    
    // Get model input tensor to determine expected data type
    TfLiteTensor* input_tensor = model_handler_->input_tensor();
    if (!input_tensor) {
        ESP_LOGE(TAG, "Cannot determine model input type");
        return result;
    }
    
    TfLiteType input_type = input_tensor->type;
    size_t required_size = 0;
    
    if (input_type == kTfLiteFloat32) {
        required_size = model_width * model_height * model_channels * sizeof(float);
    } else if (input_type == kTfLiteUInt8) {
        required_size = model_width * model_height * model_channels;
    } else {
        ESP_LOGE(TAG, "Unsupported model input type: %d", input_type);
        return result;
    }

    // Allocate output buffer
    UniqueBufferPtr buffer = allocate_image_buffer(required_size);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for output", required_size);
        return result;
    }
    
    // Process based on input format
    bool success = false;
    
    if (config_.pixel_format == "JPEG") {
        // For JPEG, we need to adjust the zone to multiples of 8
        CropZone adjusted_zone = adjust_zone_for_jpeg(zone, config_.camera_width, config_.camera_height);
        success = process_jpeg_zone_to_buffer(image, adjusted_zone, buffer->get(), required_size);
    } else {
        success = process_raw_zone_to_buffer(image, zone, buffer->get(), required_size);
    }
    
    if (success) {
        result.data = std::move(buffer);
        result.size = required_size;
        ESP_LOGD(TAG, "Zone processed successfully (%zu bytes)", result.size);
    } else {
        ESP_LOGE(TAG, "Failed to process zone");
    }
    
    DURATION_END("process_zone");
    return result;
}

/**
 * @brief Validates that a crop zone is within image bounds and has positive dimensions.
 */
bool ImageProcessor::validate_zone(const CropZone &zone) const {
    if (zone.x1 < 0 || zone.y1 < 0 || 
        zone.x2 > config_.camera_width || zone.y2 > config_.camera_height) {
        ESP_LOGE(TAG, "Zone [%d,%d,%d,%d] out of bounds [0,0,%d,%d]",
                 zone.x1, zone.y1, zone.x2, zone.y2,
                 config_.camera_width, config_.camera_height);
        return false;
    }
    
    if (zone.x2 <= zone.x1 || zone.y2 <= zone.y1) {
        ESP_LOGE(TAG, "Invalid zone dimensions: width=%d, height=%d", 
                 zone.x2 - zone.x1, zone.y2 - zone.y1);
        return false;
    }
    
    return true;
}

/**
 * @brief Processes raw image data (non-JPEG formats) directly to output buffer.
 */
bool ImageProcessor::process_raw_zone_to_buffer(
    std::shared_ptr<camera::CameraImage> image,
    const CropZone &zone,
    uint8_t* output_buffer,
    size_t output_buffer_size) {
    
    const uint8_t *input_data = image->get_data_buffer();
    size_t input_size = image->get_data_length();
    
    if (!input_data || input_size == 0) {
        ESP_LOGE(TAG, "Invalid input data");
        return false;
    }

    // Get model input dimensions and type
    const int model_width = model_handler_->get_input_width();
    const int model_height = model_handler_->get_input_height();
    const int model_channels = model_handler_->get_input_channels();
    
    // Get model input tensor to determine expected data type and size
    TfLiteTensor* input_tensor = model_handler_->input_tensor();
    if (!input_tensor) {
        ESP_LOGE(TAG, "Cannot determine model input type");
        return false;
    }
    
    TfLiteType input_type = input_tensor->type;
    size_t required_size = 0;
    
    if (input_type == kTfLiteFloat32) {
        required_size = model_width * model_height * model_channels * sizeof(float);
    } else if (input_type == kTfLiteUInt8) {
        required_size = model_width * model_height * model_channels;
    } else {
        ESP_LOGE(TAG, "Unsupported model input type: %d", input_type);
        return false;
    }
    
    // Verify output buffer size is sufficient
    if (!validate_buffer_size(required_size, output_buffer_size, "raw processing")) {
        ESP_LOGE(TAG, "Need %zu bytes for %s output, got %zu bytes", 
                 required_size, 
                 input_type == kTfLiteFloat32 ? "float32" : "uint8",
                 output_buffer_size);
        return false;
    }

    // Calculate crop dimensions
    int crop_width = zone.x2 - zone.x1;
    int crop_height = zone.y2 - zone.y1;
    
    // Validate crop zone
    if (zone.x1 < 0 || zone.y1 < 0 || 
        zone.x2 > config_.camera_width || zone.y2 > config_.camera_height) {
        ESP_LOGE(TAG, "Crop zone [%d,%d,%d,%d] out of bounds [0,0,%d,%d]",
                 zone.x1, zone.y1, zone.x2, zone.y2,
                 config_.camera_width, config_.camera_height);
        return false;
    }

    // Log the processing info at the start
    ESP_LOGD(TAG, "Processing %s and input type %s", 
             config_.pixel_format.c_str(), 
             tflite_type_to_string(input_type));

    // Process based on input format
    bool success = false;
    
    if (config_.pixel_format == "RGB888") {
        if (input_type == kTfLiteFloat32) {
            success = process_rgb888_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels,
                model_handler_->get_config().normalize);
        } else if (input_type == kTfLiteUInt8) {
            success = process_rgb888_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels);
        }
    } else if (config_.pixel_format == "RGB565") {
        if (input_type == kTfLiteFloat32) {
            success = process_rgb565_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels,
                model_handler_->get_config().normalize);
        } else if (input_type == kTfLiteUInt8) {
            success = process_rgb565_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels);
        }
    } else if (config_.pixel_format == "GRAYSCALE") {
        if (input_type == kTfLiteFloat32) {
            success = process_grayscale_crop_and_scale_to_float32(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels,
                model_handler_->get_config().normalize);
        } else if (input_type == kTfLiteUInt8) {
            success = process_grayscale_crop_and_scale_to_uint8(
                input_data, zone, crop_width, crop_height,
                output_buffer, model_width, model_height, model_channels);
        }
    } else {
        ESP_LOGE(TAG, "Unsupported raw format: %s", config_.pixel_format.c_str());
        return false;
    }

    if (!success) {
        ESP_LOGE(TAG, "Failed to process raw zone");
        return false;
    }

    ESP_LOGD(TAG, "Raw zone processed successfully: %dx%d -> %dx%d -> %dx%d",
             crop_width, crop_height, model_width, model_height);
             
#ifdef DEBUG_METER_READER_TFLITE
    if (input_type == kTfLiteFloat32) {
        debug_log_float_image(reinterpret_cast<float*>(output_buffer), 
                             model_width * model_height * model_channels,
                             model_width, model_height, model_channels,
                             "raw_processed_float");
    } else if (input_type == kTfLiteUInt8) {
        debug_log_image(output_buffer, required_size,
                       model_width, model_height, model_channels,
                       "raw_processed_uint8");
    }
#endif
             
    return true;
}


bool ImageProcessor::process_rgb888_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels,
    bool normalize) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    const std::string& input_order = model_handler_->get_config().input_order;
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = ((zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x)) * 3;
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&float_output[dst_pos], r, g, b, model_channels, normalize);
        }
    }
    
/* #ifdef DEBUG_METER_READER_TFLITE

    DEBUG_ZONE_INFO(zone, crop_width, crop_height, model_width, model_height);
    DEBUG_FIRST_PIXELS(float_output, model_width * model_height * model_channels, model_channels);
    DEBUG_CHANNEL_ORDER(float_output, model_width * model_height * model_channels, model_channels);
    
    
    // For the other debug functions, we need to convert float to uint8 for analysis
    // Create a temporary uint8 buffer for debugging
    std::vector<uint8_t> debug_buffer(model_width * model_height * model_channels);
    for (size_t i = 0; i < debug_buffer.size(); i++) {
        if (normalize) {
            debug_buffer[i] = static_cast<uint8_t>(float_output[i] * 255.0f);
        } else {
            debug_buffer[i] = static_cast<uint8_t>(float_output[i]);
        }
    }
    
    // Enhanced debugging with the converted buffer
    debug_analyze_processed_zone(debug_buffer.data(), model_width, model_height, 
                               model_channels, "final_output_uint8");
    
    debug_output_zone_preview(debug_buffer.data(), model_width, model_height,
                            model_channels, "ascii_preview");
    
    // Also debug the original crop before scaling
    ESP_LOGI(TAG, "Crop zone: [%d,%d,%d,%d] -> %dx%d -> %dx%d", 
             zone.x1, zone.y1, zone.x2, zone.y2, 
             crop_width, crop_height,
             model_width, model_height);
    
    // Debug the first few float values directly
    ESP_LOGI(TAG, "First 5 float values:");
    for (int i = 0; i < std::min(5, model_width * model_height * model_channels); i++) {
        ESP_LOGI(TAG, "  [%d]: %.6f", i, float_output[i]);
    }
#endif */
    
    return true;
}

bool ImageProcessor::process_rgb888_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels) {
    
    const std::string& input_order = model_handler_->get_config().input_order;
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = ((zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x)) * 3;
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&output_buffer[dst_pos], r, g, b, model_channels);
        }
    }
    
/* #ifdef DEBUG_METER_READER_TFLITE
    // Use output_buffer instead of float_output for uint8 processing
    DEBUG_ZONE_INFO(zone, crop_width, crop_height, model_width, model_height);
    DEBUG_FIRST_PIXELS(output_buffer, model_width * model_height * model_channels, model_channels);
    DEBUG_CHANNEL_ORDER(output_buffer, model_width * model_height * model_channels, model_channels);
             
    debug_log_image(output_buffer, model_width * model_height * model_channels,
                   model_width, model_height, model_channels,
                   "rgb888_to_uint8");
                   
    // Enhanced debugging
    debug_analyze_processed_zone(output_buffer, model_width, model_height, 
                               model_channels, "final_output");
    
    debug_output_zone_preview(output_buffer, model_width, model_height,
                            model_channels, "ascii_preview");
    
    // Also debug the original crop before scaling
    ESP_LOGI(TAG, "Crop zone: [%d,%d,%d,%d] -> %dx%d", 
             zone.x1, zone.y1, zone.x2, zone.y2, 
             crop_width, crop_height);
#endif */
    
    return true;
}

// Update process_rgb565_crop_and_scale_to_float32:
bool ImageProcessor::process_rgb565_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels,
    bool normalize) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    const uint16_t* rgb565_data = reinterpret_cast<const uint16_t*>(input_data);
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = (zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x);
            uint16_t pixel = rgb565_data[src_pos];
            
            // Convert RGB565 to RGB888
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            arrange_channels(&float_output[dst_pos], r, g, b, model_channels, normalize);
        }
    }
    
    return true;
}

// Update process_rgb565_crop_and_scale_to_uint8:
bool ImageProcessor::process_rgb565_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels) {
    
    const uint16_t* rgb565_data = reinterpret_cast<const uint16_t*>(input_data);
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = (zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x);
            uint16_t pixel = rgb565_data[src_pos];
            
            // Convert RGB565 to RGB888
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            arrange_channels(&output_buffer[dst_pos], r, g, b, model_channels);
        }
    }
    
    return true;
}

// Update scale_rgb888_to_float32:
bool ImageProcessor::scale_rgb888_to_float32(
    const uint8_t* input_data, int input_width, int input_height,
    uint8_t* output_buffer, int output_width, int output_height, int output_channels,
    bool normalize) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(input_width) / output_width;
    float y_scale = static_cast<float>(input_height) / output_height;
    
    for (int y = 0; y < output_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= input_height) src_y = input_height - 1;
        
        for (int x = 0; x < output_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= input_width) src_x = input_width - 1;
            
            // Calculate source position
            int src_pos = (src_y * input_width + src_x) * 3;
            
            // Calculate destination position
            int dst_pos = (y * output_width + x) * output_channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&float_output[dst_pos], r, g, b, output_channels, normalize);
        }
    }
    
    return true;
}

// Update scale_rgb888_to_uint8:
bool ImageProcessor::scale_rgb888_to_uint8(
    const uint8_t* input_data, int input_width, int input_height,
    uint8_t* output_buffer, int output_width, int output_height, int output_channels) {
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(input_width) / output_width;
    float y_scale = static_cast<float>(input_height) / output_height;
    
    for (int y = 0; y < output_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= input_height) src_y = input_height - 1;
        
        for (int x = 0; x < output_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= input_width) src_x = input_width - 1;
            
            // Calculate source position
            int src_pos = (src_y * input_width + src_x) * 3;
            
            // Calculate destination position
            int dst_pos = (y * output_width + x) * output_channels;
            
            uint8_t r = input_data[src_pos];
            uint8_t g = input_data[src_pos + 1];
            uint8_t b = input_data[src_pos + 2];
            
            arrange_channels(&output_buffer[dst_pos], r, g, b, output_channels);
        }
    }
    
    return true;
}


/**
 * @brief Processes grayscale crop and scales to float32 output.
 */
bool ImageProcessor::process_grayscale_crop_and_scale_to_float32(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels,
    bool normalize) {
    
    float* float_output = reinterpret_cast<float*>(output_buffer);
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = (zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x);
            uint8_t gray = input_data[src_pos];
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            if (model_channels >= 3) {
                // Replicate grayscale to all RGB channels
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

/**
 * @brief Processes grayscale crop and scales to uint8 output.
 */
bool ImageProcessor::process_grayscale_crop_and_scale_to_uint8(
    const uint8_t* input_data, const CropZone &zone,
    int crop_width, int crop_height,
    uint8_t* output_buffer, int model_width, int model_height, int model_channels) {
    
    // Simple nearest-neighbor scaling
    float x_scale = static_cast<float>(crop_width) / model_width;
    float y_scale = static_cast<float>(crop_height) / model_height;
    
    for (int y = 0; y < model_height; y++) {
        int src_y = static_cast<int>(y * y_scale);
        if (src_y >= crop_height) src_y = crop_height - 1;
        
        for (int x = 0; x < model_width; x++) {
            int src_x = static_cast<int>(x * x_scale);
            if (src_x >= crop_width) src_x = crop_width - 1;
            
            // Calculate source position in original image
            int src_pos = (zone.y1 + src_y) * config_.camera_width + (zone.x1 + src_x);
            uint8_t gray = input_data[src_pos];
            
            // Calculate destination position
            int dst_pos = (y * model_width + x) * model_channels;
            
            if (model_channels >= 3) {
                // Replicate grayscale to all RGB channels
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

/**
 * @brief Allocates an image buffer with appropriate memory type.
 */
ImageProcessor::UniqueBufferPtr ImageProcessor::allocate_image_buffer(size_t size) {
    uint8_t* buf = nullptr;
    bool is_spiram = false;
    bool is_jpeg_aligned = false;
    
    // For JPEG decoding, we need 16-byte aligned memory
    if (config_.pixel_format == "JPEG") {
        buf = (uint8_t*)jpeg_calloc_align(size, 16);
        is_jpeg_aligned = (buf != nullptr);
        if (buf) {
            ESP_LOGD(TAG, "Allocated %zu bytes with jpeg_calloc_align (16-byte aligned)", size);
        }
    } else {
        // Try SPIRAM first for non-JPEG formats
        #ifdef CONFIG_SPIRAM
        if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >= size) {
            buf = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            is_spiram = (buf != nullptr);
        }
        #endif
        
        // Fallback to internal RAM
        if (!buf) {
            buf = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT);
        }
    }
    
    if (buf) {
        // Update peak memory usage
        stats_.peak_memory_usage = std::max(stats_.peak_memory_usage, static_cast<uint32_t>(size));
        return std::unique_ptr<TrackedBuffer>(new TrackedBuffer(buf, is_spiram, is_jpeg_aligned));
    }
    
    stats_.memory_allocation_errors++;
    ESP_LOGE(TAG, "Failed to allocate %zu bytes", size);
    return nullptr;
}


void ImageProcessor::arrange_channels(float* output, uint8_t r, uint8_t g, uint8_t b, 
                                    int output_channels, bool normalize) const {
    const std::string& input_order = model_handler_->get_config().input_order;
    
    if (output_channels >= 3) {
        if (input_order == "BGR") {
            output[0] = normalize ? b / 255.0f : b;
            output[1] = normalize ? g / 255.0f : g;
            output[2] = normalize ? r / 255.0f : r;
        } else { // Default to RGB
            output[0] = normalize ? r / 255.0f : r;
            output[1] = normalize ? g / 255.0f : g;
            output[2] = normalize ? b / 255.0f : b;
        }
    } else if (output_channels == 1) {
        // Convert to grayscale: 0.299*R + 0.587*G + 0.114*B
        float gray = 0.299f * r + 0.587f * g + 0.114f * b;
        output[0] = normalize ? gray / 255.0f : gray;
    }
}

void ImageProcessor::arrange_channels(uint8_t* output, uint8_t r, uint8_t g, uint8_t b, 
                                    int output_channels) const {
    const std::string& input_order = model_handler_->get_config().input_order;
    
    if (output_channels >= 3) {
        if (input_order == "BGR") {
            output[0] = b;
            output[1] = g;
            output[2] = r;
        } else { // Default to RGB
            output[0] = r;
            output[1] = g;
            output[2] = b;
        }
    } else if (output_channels == 1) {
        // Convert to grayscale: 0.299*R + 0.587*G + 0.114*B
        output[0] = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
    }
}


// In image_processor.cpp, update the function implementations:
#ifdef DEBUG_METER_READER_TFLITE
void ImageProcessor::debug_log_image_stats(const uint8_t* data, size_t size,
                                         const std::string& stage) {
    if (!data || size == 0) return;
    
    // Calculate statistics
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
             stage.c_str(), size, min_val, max_val, mean, 
             zero_count, size, (zero_count * 100.0f) / size);
    
    // Log first few values for pattern recognition
    ESP_LOGD(TAG, "DEBUG %s first 10 values:", stage.c_str());
    std::string values_str;
    for (int i = 0; i < std::min(10, (int)size); i++) {
        values_str += std::to_string(data[i]) + " ";
    }
    ESP_LOGD(TAG, "  %s", values_str.c_str());
}

void ImageProcessor::debug_log_float_stats(const float* data, size_t count,
                                         const std::string& stage) {
    if (!data || count == 0) return;
    
    // Calculate statistics
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    float sum = 0.0f;
    int zero_count = 0;
    
    for (size_t i = 0; i < count; i++) {
        min_val = std::min(min_val, data[i]);
        max_val = std::max(max_val, data[i]);
        sum += data[i];
        if (data[i] == 0.0f) zero_count++;
    }
    
    float mean = sum / count;
    
    ESP_LOGD(TAG, "DEBUG %s: count=%zu, min=%.3f, max=%.3f, mean=%.3f, zeros=%d/%zu (%.1f%%)",
             stage.c_str(), count, min_val, max_val, mean, 
             zero_count, count, (zero_count * 100.0f) / count);
    
    // Log first few values
    ESP_LOGD(TAG, "DEBUG %s first 10 values:", stage.c_str());
    std::string values_str;
    for (int i = 0; i < std::min(10, (int)count); i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f ", data[i]);
        values_str += buf;
    }
    ESP_LOGD(TAG, "  %s", values_str.c_str());
}

void ImageProcessor::debug_log_image(const uint8_t* data, size_t size, 
                                   int width, int height, int channels,
                                   const std::string& stage) {
    if (!data || size == 0) return;
    
    ESP_LOGD(TAG, "DEBUG %s: %dx%dx%d (%zu bytes)", 
             stage.c_str(), width, height, channels, size);
    
    debug_log_image_stats(data, size, stage);
}

void ImageProcessor::debug_log_float_image(const float* data, size_t count,
                                         int width, int height, int channels,
                                         const std::string& stage) {
    if (!data || count == 0) return;
    
    ESP_LOGD(TAG, "DEBUG %s: %dx%dx%d (%zu floats)", 
             stage.c_str(), width, height, channels, count);
    
    debug_log_float_stats(data, count, stage);
}

void ImageProcessor::debug_log_rgb888_image(const uint8_t* data, 
                                          int width, int height,
                                          const std::string& stage) {
    if (!data || width <= 0 || height <= 0) return;
    
    size_t size = width * height * 3;
    ESP_LOGD(TAG, "DEBUG %s: %dx%d RGB (%zu bytes)", 
             stage.c_str(), width, height, size);
    
    debug_log_image_stats(data, size, stage);
    
    // Log RGB values for first pixel
    if (size >= 3) {
        ESP_LOGD(TAG, "DEBUG %s first pixel: R=%u, G=%u, B=%u",
                 stage.c_str(), data[0], data[1], data[2]);
    }
}


void ImageProcessor::debug_analyze_processed_zone(const uint8_t* data, 
                                                 int width, int height, 
                                                 int channels,
                                                 const std::string& zone_name) {
    if (!data || width <= 0 || height <= 0) return;
    
    const size_t total_pixels = width * height;
    ESP_LOGI(TAG, "ZONE_ANALYSIS:%s:%dx%dx%d:normalized=false", 
             zone_name.c_str(), width, height, channels);
    
    // Output ALL pixels for complete image reconstruction
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            ESP_LOGI(TAG, "  Pixel[%d,%d]:", x, y);
            for (int c = 0; c < channels; c++) {
                ESP_LOGI(TAG, "    Channel %d: %u", c, data[pos + c]);
            }
        }
    }
    
    // Calculate statistics
    uint8_t min_val = 255;
    uint8_t max_val = 0;
    uint32_t sum = 0;
    
    for (int i = 0; i < width * height * channels; i++) {
        min_val = std::min(min_val, data[i]);
        max_val = std::max(max_val, data[i]);
        sum += data[i];
    }
    
    float mean = static_cast<float>(sum) / (width * height * channels);
    ESP_LOGI(TAG, "  Stats: min=%u, max=%u, mean=%.6f", min_val, max_val, mean);
}

// And for the float version:
void ImageProcessor::debug_analyze_float_zone(const float* data, 
                                             int width, int height, 
                                             int channels,
                                             const std::string& zone_name,
                                             bool normalized) {
    if (!data || width <= 0 || height <= 0) return;
    
    const size_t total_pixels = width * height;
    ESP_LOGI(TAG, "ZONE_ANALYSIS:%s:%dx%dx%d:normalized=%s", 
             zone_name.c_str(), width, height, channels,
             normalized ? "true" : "false");
    
    // Output ALL pixels for complete image reconstruction
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            ESP_LOGI(TAG, "  Pixel[%d,%d]:", x, y);
            for (int c = 0; c < channels; c++) {
                ESP_LOGI(TAG, "    Channel %d: %.6f", c, data[pos + c]);
            }
        }
    }
    
    // Calculate statistics
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    float sum = 0.0f;
    
    for (int i = 0; i < width * height * channels; i++) {
        min_val = std::min(min_val, data[i]);
        max_val = std::max(max_val, data[i]);
        sum += data[i];
    }
    
    float mean = sum / (width * height * channels);
    ESP_LOGI(TAG, "  Stats: min=%.6f, max=%.6f, mean=%.6f", min_val, max_val, mean);
    
    // Check expected range
    if (normalized) {
        if (min_val >= 0.0f && max_val <= 1.0f) {
            ESP_LOGI(TAG, "  -> Values in expected normalized range [0,1]");
        } else {
            ESP_LOGI(TAG, "  -> WARNING: Values outside normalized range [0,1]");
        }
    } else {
        if (min_val >= 0.0f && max_val <= 255.0f) {
            ESP_LOGI(TAG, "  -> Values in expected raw range [0,255]");
        } else {
            ESP_LOGI(TAG, "  -> WARNING: Values outside raw range [0,255]");
        }
    }
}

void ImageProcessor::debug_output_zone_preview(const uint8_t* data,
                                              int width, int height,
                                              int channels,
                                              const std::string& zone_name) {
    // Output a simple ASCII art preview for small zones
    if (width > 20 || height > 10) {
        ESP_LOGI(TAG, "PREVIEW_TOO_LARGE:%s", zone_name.c_str());
        return;
    }
    
    ESP_LOGI(TAG, "PREVIEW_START:%s:%dx%d", zone_name.c_str(), width, height);
    
    for (int y = 0; y < height; y++) {
        std::string line;
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            uint8_t brightness;
            
            if (channels == 1) {
                brightness = data[pos];
            } else {
                // Convert to grayscale for preview
                brightness = static_cast<uint8_t>(
                    0.299 * data[pos] + 0.587 * data[pos + 1] + 0.114 * data[pos + 2]
                );
            }
            
            // Map brightness to ASCII characters
            const char* chars = " .:-=+*#%@";
            int char_idx = (brightness * 9) / 255;
            line += chars[char_idx];
        }
        ESP_LOGI(TAG, "%s", line.c_str());
    }
    
    ESP_LOGI(TAG, "PREVIEW_END:%s", zone_name.c_str());
}



void ImageProcessor::debug_output_float_preview(const float* data,
                                               int width, int height,
                                               int channels,
                                               const std::string& zone_name,
                                               bool normalized) {
    // Output a simple ASCII art preview for small zones
    if (width > 20 || height > 10) {
        ESP_LOGI(TAG, "FLOAT_PREVIEW_TOO_LARGE:%s", zone_name.c_str());
        return;
    }
    
    ESP_LOGI(TAG, "FLOAT_PREVIEW_START:%s:%dx%d:normalized=%s", 
             zone_name.c_str(), width, height,
             normalized ? "true" : "false");
    
    for (int y = 0; y < height; y++) {
        std::string line;
        for (int x = 0; x < width; x++) {
            int pos = (y * width + x) * channels;
            float brightness;
            
            if (channels == 1) {
                brightness = data[pos];
            } else {
                // Convert to grayscale for preview
                brightness = 0.299f * data[pos] + 0.587f * data[pos + 1] + 0.114f * data[pos + 2];
            }
            
            // Convert to 0-255 range for ASCII mapping
            uint8_t ascii_val;
            if (normalized) {
                ascii_val = static_cast<uint8_t>(brightness * 255.0f);
            } else {
                ascii_val = static_cast<uint8_t>(brightness);
            }
            
            // Map brightness to ASCII characters
            const char* chars = " .:-=+*#%@";
            int char_idx = (ascii_val * 9) / 255;
            line += chars[char_idx];
        }
        ESP_LOGI(TAG, "%s", line.c_str());
    }
    
    ESP_LOGI(TAG, "FLOAT_PREVIEW_END:%s", zone_name.c_str());
}

#endif

/**
 * @brief Resets processing statistics.
 */
// void ImageProcessor::reset_stats() {
    // std::lock_guard<std::mutex> lock(processing_mutex_);
    // stats_ = ProcessingStats();
// }

/**
 * @brief Gets current processing statistics.
 */
// ImageProcessor::ProcessingStats ImageProcessor::get_stats() const {
    // std::lock_guard<std::mutex> lock(processing_mutex_);
    // return stats_;
// }

} // namespace meter_reader_tflite
} // namespace esphome
