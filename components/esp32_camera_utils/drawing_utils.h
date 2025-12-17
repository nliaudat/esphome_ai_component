#pragma once

// Macro to enable/disable drawing functionality
// #define USE_CAMERA_DRAWING

#ifdef USE_CAMERA_DRAWING

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace esp32_camera_utils {

/**
 * @brief Utilities for drawing shapes on image buffers.
 * Supports RGB565 and RGB888 formats.
 */
class DrawingUtils {
 public:
  /**
   * @brief Draw a single pixel.
   * 
   * @param buffer Pointer to the image buffer.
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param w Image width.
   * @param h Image height.
   * @param channels Number of channels (2 for RGB565, 3 for RGB888).
   * @param color Color in RGB565 format (uint16_t).
   */
  static void draw_pixel(uint8_t* buffer, int x, int y, int w, int h, int channels, uint16_t color);

  /**
   * @brief Draw a rectangle outline.
   */
  static void draw_rectangle(uint8_t* buffer, int x, int y, int rect_w, int rect_h, 
                             int img_w, int img_h, int channels, uint16_t color);

  /**
   * @brief Draw a filled rectangle.
   */
  static void draw_filled_rectangle(uint8_t* buffer, int x, int y, int rect_w, int rect_h, 
                                    int img_w, int img_h, int channels, uint16_t color);

  /**
   * @brief Draw a circle outline.
   */
  static void draw_circle(uint8_t* buffer, int center_x, int center_y, int radius, 
                          int img_w, int img_h, int channels, uint16_t color);

  /**
   * @brief Draw a filled circle.
   */
  static void draw_filled_circle(uint8_t* buffer, int center_x, int center_y, int radius, 
                                 int img_w, int img_h, int channels, uint16_t color);

 private:
  // Helper to convert RGB565 color to RGB888
  static void rgb565_to_rgb888(uint16_t color, uint8_t& r, uint8_t& g, uint8_t& b);
};

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_DRAWING
