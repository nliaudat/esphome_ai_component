#pragma once

// #define USE_CAMERA_ROTATOR

#ifdef USE_CAMERA_ROTATOR

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "esphome/core/log.h"
#include "../tflite_micro_helper/debug_utils.h"

namespace esphome {
namespace esp32_camera_utils {

class Rotator {
 public:
  /**
   * @brief Apply arbitrary software rotation.
   * 
   * @param input Input buffer.
   * @param output Output buffer (must be pre-allocated).
   * @param src_w Source width.
   * @param src_h Source height.
   * @param channels Number of channels.
   * @param angle_deg Rotation angle in degrees (clockwise).
   * @param out_w Output width.
   * @param out_h Output height.
   * @return true if successful.
   */
  static bool perform_rotation(const uint8_t* input, uint8_t* output, 
                     int src_w, int src_h, int channels, 
                     float angle_deg, int out_w, int out_h);

  /**
   * @brief Calculate the bounding box dimensions for a rotated image.
   * 
   * @param src_w Source width.
   * @param src_h Source height.
   * @param angle_deg Angle in degrees.
   * @param out_w [out] New width.
   * @param out_h [out] New height.
   */
  static void get_rotated_dimensions(int src_w, int src_h, float angle_deg, 
                                     int& out_w, int& out_h);

 private:
  static constexpr float PI = 3.14159265359f;
  static constexpr float DEG_TO_RAD = PI / 180.0f;
};

// Implementation moved to header to resolve linker issues
inline void Rotator::get_rotated_dimensions(int src_w, int src_h, float angle_deg, 
                                     int& out_w, int& out_h) {
    float angle_rad = angle_deg * DEG_TO_RAD;
    float cos_a = std::abs(std::cos(angle_rad));
    float sin_a = std::abs(std::sin(angle_rad));

    out_w = (int)(src_w * cos_a + src_h * sin_a);
    out_h = (int)(src_w * sin_a + src_h * cos_a);
    
    // Ensure even dimensions for compatibility usually
    if (out_w % 2 != 0) out_w++;
    if (out_h % 2 != 0) out_h++;
}

inline bool Rotator::perform_rotation(const uint8_t* input, uint8_t* output, 
                     int src_w, int src_h, int channels, 
                     float angle_deg, int out_w, int out_h) {
    if (!input || !output) return false;
    
    // We only log non-trivial rotations to avoid spam on pass-through
    bool is_trivial = (std::abs(angle_deg) < 0.1f || std::abs(angle_deg - 360.0f) < 0.1f);
    if (!is_trivial) {
        DURATION_START();
    } else {
        // Still define start to avoid compilation errors if macro expands freely
        #ifdef DURATION_START
        DURATION_START(); 
        #endif
    }

    // Sanity check to prevent infinite loop
    if (!std::isfinite(angle_deg)) {
        return false;
    }

    // Normalize rotation
    float rot = angle_deg;
    while (rot < 0) rot += 360.0f;
    while (rot >= 360.0f) rot -= 360.0f;

    // 0 degrees (Copy)
    if (rot < 0.1f || rot > 359.9f) {
        size_t size = src_w * src_h * channels;
        memcpy(output, input, size);
        if (!is_trivial) DURATION_END("perform_rotation_0");
        else DURATION_END("perform_rotation_0_trivial");
        return true;
    }

    static const char* TAG = "rotator"; // Needed for ESP_LOGD inside macro if not globally defined here

    // 90 degrees
    if (std::abs(rot - 90.0f) < 0.1f) {
        for (int y = 0; y < src_h; y++) {
            for (int x = 0; x < src_w; x++) {
                int src_idx = (y * src_w + x) * channels;
                int dst_idx = (x * out_w + (src_h - 1 - y)) * channels;
                
                if (channels == 3) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                    output[dst_idx+2] = input[src_idx+2];
                } else if (channels == 2) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                } else {
                    memcpy(output + dst_idx, input + src_idx, channels);
                }
            }
        }
        DURATION_END("perform_rotation_90");
        return true;
    }

    // 180 degrees
    if (std::abs(rot - 180.0f) < 0.1f) {
        for (int y = 0; y < src_h; y++) {
            for (int x = 0; x < src_w; x++) {
                int src_idx = (y * src_w + x) * channels;
                int dst_idx = ((src_h - 1 - y) * out_w + (src_w - 1 - x)) * channels;
                
                if (channels == 3) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                    output[dst_idx+2] = input[src_idx+2];
                } else if (channels == 2) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                } else {
                    memcpy(output + dst_idx, input + src_idx, channels);
                }
            }
        }
        DURATION_END("perform_rotation_180");
        return true;
    }

    // 270 degrees
    if (std::abs(rot - 270.0f) < 0.1f) {
        for (int y = 0; y < src_h; y++) {
            for (int x = 0; x < src_w; x++) {
                int src_idx = (y * src_w + x) * channels;
                int dst_idx = ((src_w - 1 - x) * out_w + y) * channels;
                
                if (channels == 3) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                    output[dst_idx+2] = input[src_idx+2];
                } else if (channels == 2) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                } else {
                     memcpy(output + dst_idx, input + src_idx, channels);
                }
            }
        }
        DURATION_END("perform_rotation_270");
        return true;
    }

    // Arbitrary rotation (Nearest Neighbor)
    float angle_rad = rot * DEG_TO_RAD;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);

    float cx = src_w / 2.0f;
    float cy = src_h / 2.0f;
    float ox = out_w / 2.0f;
    float oy = out_h / 2.0f;

    for (int y = 0; y < out_h; y++) {
        for (int x = 0; x < out_w; x++) {
            float dx = x - ox;
            float dy = y - oy;

            int sx = (int)(dx * cos_a + dy * sin_a + cx);
            int sy = (int)(-dx * sin_a + dy * cos_a + cy);

            if (sx >= 0 && sx < src_w && sy >= 0 && sy < src_h) {
                int src_idx = (sy * src_w + sx) * channels;
                int dst_idx = (y * out_w + x) * channels;
                
                if (channels == 3) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                    output[dst_idx+2] = input[src_idx+2];
                } else if (channels == 2) {
                    output[dst_idx] = input[src_idx];
                    output[dst_idx+1] = input[src_idx+1];
                } else {
                     memcpy(output + dst_idx, input + src_idx, channels);
                }
            } else {
                int dst_idx = (y * out_w + x) * channels;
                memset(output + dst_idx, 0, channels); // Fill black
            }
        }
    }
    DURATION_END("perform_rotation_arbitrary");
    return true;
}

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_ROTATOR
