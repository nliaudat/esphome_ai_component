#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// RAII timer replaces DURATION_START/END macros (ง7.4).
// Zero-cost when DEBUG_DURATION is not defined.
#ifdef DEBUG_DURATION
class ScopedTimer {
 public:
  explicit ScopedTimer(const char *name) : name_(name), start_(esphome::millis()) {}
  ~ScopedTimer() { ESP_LOGD("rotator", "%s took %ums", name_, millis() - start_); }
  uint32_t start_time() const { return start_; }
 private:
  const char *name_;
  uint32_t start_;
};
#else
class ScopedTimer {
 public:
  explicit ScopedTimer(const char *) : start_(esphome::millis()) {}
  uint32_t start_time() const { return start_; }
 private:
  uint32_t start_;
};
#endif

#ifdef USE_CAMERA_ROTATOR
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
  static bool perform_rotation(const uint8_t *input, uint8_t *output, int src_w, int src_h, int channels,
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
  static void get_rotated_dimensions(int src_w, int src_h, float angle_deg, int &out_w, int &out_h);

 private:
  // Use lowercase constants to avoid conflicts with Arduino.h PI and DEG_TO_RAD macros
  static constexpr float pi = 3.14159265359f;
  static constexpr float deg_to_rad = pi / 180.0f;
};

// Implementation moved to header to resolve linker issues
inline void Rotator::get_rotated_dimensions(int src_w, int src_h, float angle_deg, int &out_w, int &out_h) {
  float angle_rad = angle_deg * deg_to_rad;
  float cos_a = std::abs(std::cos(angle_rad));
  float sin_a = std::abs(std::sin(angle_rad));

  out_w = static_cast<int>(src_w * cos_a + src_h * sin_a);
  out_h = static_cast<int>(src_w * sin_a + src_h * cos_a);

  // Ensure even dimensions for compatibility usually
  if (out_w % 2 != 0)
    out_w++;
  if (out_h % 2 != 0)
    out_h++;
}

inline bool Rotator::perform_rotation(const uint8_t *input, uint8_t *output, int src_w, int src_h, int channels,
                                      float angle_deg, int out_w, int out_h) {
  if (!input || !output)
    return false;

  // RAII timer records start time and logs duration on scope exit
  ScopedTimer timer("perform_rotation");

  // Sanity check to prevent infinite loop
  if (!std::isfinite(angle_deg)) {
    return false;
  }

  // Normalize rotation
  float rot = angle_deg;
  while (rot < 0)
    rot += 360.0f;
  while (rot >= 360.0f)
    rot -= 360.0f;

  // 0 degrees (Copy)
  if (rot < 0.1f || rot > 359.9f) {
    // Guarded multiplication per ยง8.1 (CVE-2026-23833 pattern)
    if (src_w <= 0 || src_h <= 0 || channels <= 0)
      return false;
    if (static_cast<uint64_t>(src_h) > SIZE_MAX / static_cast<size_t>(src_w))
      return false;
    const size_t pixels = static_cast<size_t>(src_w) * static_cast<size_t>(src_h);
    if (pixels > SIZE_MAX / static_cast<size_t>(channels))
      return false;
    size_t size = pixels * static_cast<size_t>(channels);
    memcpy(output, input, size);
    return true;
  }

  // 90 degrees
  if (std::abs(rot - 90.0f) < 0.1f) {
    for (int y = 0; y < src_h; y++) {
      for (int x = 0; x < src_w; x++) {
        int src_idx = (y * src_w + x) * channels;
        int dst_idx = (x * out_w + (src_h - 1 - y)) * channels;

        if (channels == 3) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
          output[dst_idx + 2] = input[src_idx + 2];
        } else if (channels == 2) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
        } else {
          memcpy(output + dst_idx, input + src_idx, channels);
        }
      }
    }
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
          output[dst_idx + 1] = input[src_idx + 1];
          output[dst_idx + 2] = input[src_idx + 2];
        } else if (channels == 2) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
        } else {
          memcpy(output + dst_idx, input + src_idx, channels);
        }
      }
    }
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
          output[dst_idx + 1] = input[src_idx + 1];
          output[dst_idx + 2] = input[src_idx + 2];
        } else if (channels == 2) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
        } else {
          memcpy(output + dst_idx, input + src_idx, channels);
        }
      }
    }
    return true;
  }

  // Arbitrary rotation (Nearest Neighbor)
  float angle_rad = rot * deg_to_rad;
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

      int sx = static_cast<int>(dx * cos_a + dy * sin_a + cx);
      int sy = static_cast<int>(-dx * sin_a + dy * cos_a + cy);

      if (sx >= 0 && sx < src_w && sy >= 0 && sy < src_h) {
        int src_idx = (sy * src_w + sx) * channels;
        int dst_idx = (y * out_w + x) * channels;

        if (channels == 3) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
          output[dst_idx + 2] = input[src_idx + 2];
        } else if (channels == 2) {
          output[dst_idx] = input[src_idx];
          output[dst_idx + 1] = input[src_idx + 1];
        } else {
          memcpy(output + dst_idx, input + src_idx, channels);
        }
      } else {
        int dst_idx = (y * out_w + x) * channels;
        memset(output + dst_idx, 0, channels);  // Fill black
      }
    }
  }
  return true;
}

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif  // USE_CAMERA_ROTATOR
