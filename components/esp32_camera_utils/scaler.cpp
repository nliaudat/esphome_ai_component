#include "scaler.h"

#ifdef USE_CAMERA_SCALER

#include <cmath>
#include <algorithm>
#include "../tflite_micro_helper/debug_utils.h"

namespace esphome {
namespace esp32_camera_utils {

// Optimized implementation using fixed-point math (Q16.16)
bool Scaler::scale_rgb888_to_uint8(const uint8_t* src, int src_w, int src_h, 
                                   uint8_t* dst, int dst_w, int dst_h, int channels) {
    DURATION_START();
    if (!src || !dst) return false;

    // Pre-calculate scale ratio in fixed point
    uint32_t ratio_x = (src_w << FIXED_POINT_SHIFT) / dst_w;
    uint32_t ratio_y = (src_h << FIXED_POINT_SHIFT) / dst_h;

    for (int y = 0; y < dst_h; y++) {
        // Calculate source Y index (nearest neighbor)
        int src_y = (y * ratio_y) >> FIXED_POINT_SHIFT;
        const uint8_t* src_row = src + (src_y * src_w * 3);
        uint8_t* dst_row = dst + (y * dst_w * channels);

        for (int x = 0; x < dst_w; x++) {
            // Calculate source X index
            int src_x = (x * ratio_x) >> FIXED_POINT_SHIFT;
            const uint8_t* src_pixel = src_row + (src_x * 3);
            
            if (channels == 3) {
                dst_row[0] = src_pixel[0];
                dst_row[1] = src_pixel[1];
                dst_row[2] = src_pixel[2];
                dst_row += 3;
            } else if (channels == 1) {
                // Grayscale conversion: Y = 0.299R + 0.587G + 0.114B
                // Integer approximation: Y = (19595*R + 38469*G + 7472*B) >> 16
                // Or simpler: (77*R + 150*G + 29*B) >> 8
                uint16_t gray = (77 * src_pixel[0] + 150 * src_pixel[1] + 29 * src_pixel[2]) >> 8;
                *dst_row++ = static_cast<uint8_t>(gray);
            }
        }
    }
    DURATION_END("scale_rgb888_to_uint8");
    return true;
}

bool Scaler::scale_rgb888_to_float32(const uint8_t* src, int src_w, int src_h, 
                                     uint8_t* dst, int dst_w, int dst_h, int channels, bool normalize) {
    DURATION_START();
    if (!src || !dst) return false;

    float* dst_float = reinterpret_cast<float*>(dst);
    uint32_t ratio_x = (src_w << FIXED_POINT_SHIFT) / dst_w;
    uint32_t ratio_y = (src_h << FIXED_POINT_SHIFT) / dst_h;

    for (int y = 0; y < dst_h; y++) {
        int src_y = (y * ratio_y) >> FIXED_POINT_SHIFT;
        const uint8_t* src_row = src + (src_y * src_w * 3);
        float* dst_row = dst_float + (y * dst_w * channels);

        for (int x = 0; x < dst_w; x++) {
            int src_x = (x * ratio_x) >> FIXED_POINT_SHIFT;
            const uint8_t* src_pixel = src_row + (src_x * 3);

            if (channels == 3) {
                if (normalize) {
                    *dst_row++ = src_pixel[0] * (1.0f / 255.0f);
                    *dst_row++ = src_pixel[1] * (1.0f / 255.0f);
                    *dst_row++ = src_pixel[2] * (1.0f / 255.0f);
                } else {
                    *dst_row++ = static_cast<float>(src_pixel[0]);
                    *dst_row++ = static_cast<float>(src_pixel[1]);
                    *dst_row++ = static_cast<float>(src_pixel[2]);
                }
            } else if (channels == 1) {
                uint16_t gray = (77 * src_pixel[0] + 150 * src_pixel[1] + 29 * src_pixel[2]) >> 8;
                if (normalize) {
                    *dst_row++ = gray * (1.0f / 255.0f);
                } else {
                    *dst_row++ = static_cast<float>(gray);
                }
            }
        }
    }
    DURATION_END("scale_rgb888_to_float32");
    return true;
}

bool Scaler::scale_nearest(const uint8_t* src, int src_w, int src_h, int src_channels,
                           uint8_t* dst, int dst_w, int dst_h) {
    DURATION_START();
    if (!src || !dst) return false;

    uint32_t ratio_x = (src_w << FIXED_POINT_SHIFT) / dst_w;
    uint32_t ratio_y = (src_h << FIXED_POINT_SHIFT) / dst_h;

    for (int y = 0; y < dst_h; y++) {
        int src_y = (y * ratio_y) >> FIXED_POINT_SHIFT;
        const uint8_t* src_row = src + (src_y * src_w * src_channels);
        uint8_t* dst_row = dst + (y * dst_w * src_channels);

        for (int x = 0; x < dst_w; x++) {
            int src_x = (x * ratio_x) >> FIXED_POINT_SHIFT;
            const uint8_t* src_pixel = src_row + (src_x * src_channels);

            // Copy all channels
            for (int c = 0; c < src_channels; c++) {
                dst_row[c] = src_pixel[c];
            }
            dst_row += src_channels;
        }
    }
    DURATION_END("scale_nearest");
    return true;
}

bool Scaler::scale_bilinear(const uint8_t* src, int src_w, int src_h, int src_channels,
                            uint8_t* dst, int dst_w, int dst_h) {
     // TODO: Implement Bilinear if higher quality needed. 
     // For now, defaulting to nearest as it's much faster on S3 without SIMD.
     DURATION_START();
     bool res = scale_nearest(src, src_w, src_h, src_channels, dst, dst_w, dst_h);
     DURATION_END("scale_bilinear");
     return res;
}

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_SCALER
