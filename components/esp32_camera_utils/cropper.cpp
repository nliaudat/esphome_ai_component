#include "cropper.h"

#ifdef USE_CAMERA_CROPPER

#include <cstring>
#include "../tflite_micro_helper/debug_utils.h"

namespace esphome {
namespace esp32_camera_utils {

ImageView Cropper::get_crop_view(const uint8_t* src, int src_w, int src_h, int channels, 
                                 const CropZone& zone) {
    ImageView view = {nullptr, 0, 0, 0, 0};

    if (!src) return view;

    // Validate zone bounds
    if (zone.x1 < 0 || zone.y1 < 0 || zone.x2 > src_w || zone.y2 > src_h) {
        return view; // Invalid
    }

    int width = zone.x2 - zone.x1;
    int height = zone.y2 - zone.y1;

    if (width <= 0 || height <= 0) return view;

    // Calculate pointer to top-left pixel of the crop
    int start_offset = (zone.y1 * src_w + zone.x1) * channels;

    view.data = src + start_offset;
    view.width = width;
    view.height = height;
    view.stride = src_w * channels; // Stride remains the full width of original image
    view.channels = channels;

    return view;
}

bool Cropper::extract_to_buffer(const ImageView& view, uint8_t* dst) {
    DURATION_START();
    if (!view.is_valid() || !dst) return false;

    int row_size = view.width * view.channels;
    
    // If the view is contiguous (stride equals width), we can do a single memcpy
    if (row_size == view.stride) {
        memcpy(dst, view.data, row_size * view.height);
    } else {
        // Otherwise, copy row by row
        const uint8_t* src_row = view.data;
        uint8_t* dst_row = dst;
        
        for (int y = 0; y < view.height; y++) {
            memcpy(dst_row, src_row, row_size);
            src_row += view.stride;
            dst_row += row_size;
        }
    }
    DURATION_END("extract_to_buffer");
    return true;
}

bool Cropper::validate_zone(const CropZone& zone, int img_w, int img_h) {
     return (zone.x1 >= 0 && zone.y1 >= 0 && 
             zone.x2 <= img_w && zone.y2 <= img_h &&
             zone.x2 > zone.x1 && zone.y2 > zone.y1);
}

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_CROPPER
