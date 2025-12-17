#pragma once

// #define USE_CAMERA_CROPPER

#ifdef USE_CAMERA_CROPPER

#include <cstdint>
#include <cstddef>
#include "crop_zone_handler.h" // Ensure we can see CropZone definition

namespace esphome {
namespace esp32_camera_utils {

/**
 * @brief Represents a view into an image buffer without owning the data.
 * Used for zero-copy cropping where the stride might be larger than the width.
 */
struct ImageView {
    const uint8_t* data;
    int width;
    int height;
    int stride;    // Bytes per row in the original source
    int channels;
    
    // Check if the view is valid
    bool is_valid() const { return data != nullptr && width > 0 && height > 0; }
};

class Cropper {
 public:
  /**
   * @brief Get a zero-copy view of a crop zone within a larger image.
   * 
   * @param src Source image buffer.
   * @param src_w Source image width.
   * @param src_h Source image height.
   * @param channels Number of channels.
   * @param zone The crop zone definition.
   * @return ImageView pointing to the start of the crop, with appropriate stride.
   */
  static ImageView get_crop_view(const uint8_t* src, int src_w, int src_h, int channels, 
                                 const CropZone& zone);

  /**
   * @brief Copy a crop zone to a packed buffer (removing stride).
   * 
   * @param view The partial view/crop.
   * @param dst Destination buffer (must be large enough: w*h*c).
   */
  static bool extract_to_buffer(const ImageView& view, uint8_t* dst);

  /**
   * @brief Validate that a zone fits within the image dimensions.
   */
  static bool validate_zone(const CropZone& zone, int img_w, int img_h);
};

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_CROPPER
