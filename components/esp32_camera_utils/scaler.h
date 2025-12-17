#pragma once

// #define USE_CAMERA_SCALER

#ifdef USE_CAMERA_SCALER

#include <cstdint>
#include <cstddef>
#include "esp_heap_caps.h"

namespace esphome {
namespace esp32_camera_utils {

class Scaler {
 public:
  /**
   * @brief Scale RGB888 buffer to uint8 (grayscale) with resizing.
   * Uses fixed-point arithmetic for optimization.
   */
  static bool scale_rgb888_to_uint8(const uint8_t* src, int src_w, int src_h, 
                                    uint8_t* dst, int dst_w, int dst_h, int channels);

  /**
   * @brief Scale RGB888 buffer to float32 (for TensorFlow Lite) with resizing.
   * Uses fixed-point arithmetic for optimization.
   */
  static bool scale_rgb888_to_float32(const uint8_t* src, int src_w, int src_h, 
                                      uint8_t* dst, int dst_w, int dst_h, int channels, bool normalize);

  /**
   * @brief Generic nearest-neighbor scaling for any byte buffer.
   */
  static bool scale_nearest(const uint8_t* src, int src_w, int src_h, int src_channels,
                            uint8_t* dst, int dst_w, int dst_h);
                            
  /**
   * @brief Simple bilinear scaling (software only, slower but better quality).
   */
  static bool scale_bilinear(const uint8_t* src, int src_w, int src_h, int src_channels,
                             uint8_t* dst, int dst_w, int dst_h);

 private:
  static constexpr int FIXED_POINT_SHIFT = 16;
};

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_SCALER
