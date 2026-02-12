#include "esphome/core/defines.h"
#include "drawing_utils.h"

#ifdef USE_CAMERA_DRAWING

namespace esphome {
namespace esp32_camera_utils {

void DrawingUtils::rgb565_to_rgb888(uint16_t color, uint8_t& r, uint8_t& g, uint8_t& b) {
  r = (color >> 11) << 3;
  g = ((color >> 5) & 0x3F) << 2;
  b = (color & 0x1F) << 3;
}

void DrawingUtils::draw_pixel(uint8_t* buffer, int x, int y, int w, int h, int channels, uint16_t color) {
  if (x < 0 || x >= w || y < 0 || y >= h) return;

  int index = (y * w + x) * channels;

  if (channels == 2) {
    // RGB565
    buffer[index] = static_cast<uint8_t>(color & 0xFF);     // Low byte
    buffer[index + 1] = static_cast<uint8_t>(color >> 8);   // High byte (ESP32 is Little Endian usually, but check camera format)
    // Standard RGB565 handling: Low byte first, then High byte.
    // This matches common ESP32 camera configurations.
    // If colors look swapped, we swap these.
    // Standard RGB565 is often High byte first in memory for some displays, but let's stick to simple copy.
    // Re-checking standard assignment: usually cast to uint16_t* is safer if aligned.
    // But buffer is uint8_t*.
    // Let's rely on standard byte order.
    *(uint16_t*)(&buffer[index]) = color; 
  } else if (channels == 3) {
    // RGB888
    uint8_t r, g, b;
    rgb565_to_rgb888(color, r, g, b);
    buffer[index] = r;
    buffer[index + 1] = g;
    buffer[index + 2] = b;
  }
}

void DrawingUtils::draw_rectangle(uint8_t* buffer, int x, int y, int rect_w, int rect_h, 
                                  int img_w, int img_h, int channels, uint16_t color) {
    // Top and bottom
    for (int i = x; i < x + rect_w; ++i) {
        draw_pixel(buffer, i, y, img_w, img_h, channels, color);
        draw_pixel(buffer, i, y + rect_h - 1, img_w, img_h, channels, color);
    }
    // Left and right
    for (int j = y; j < y + rect_h; ++j) {
        draw_pixel(buffer, x, j, img_w, img_h, channels, color);
        draw_pixel(buffer, x + rect_w - 1, j, img_w, img_h, channels, color);
    }
}

void DrawingUtils::draw_filled_rectangle(uint8_t* buffer, int x, int y, int rect_w, int rect_h, 
                                         int img_w, int img_h, int channels, uint16_t color) {
    for (int j = y; j < y + rect_h; ++j) {
        for (int i = x; i < x + rect_w; ++i) {
            draw_pixel(buffer, i, j, img_w, img_h, channels, color);
        }
    }
}

void DrawingUtils::draw_circle(uint8_t* buffer, int center_x, int center_y, int radius, 
                               int img_w, int img_h, int channels, uint16_t color) {
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;

    auto plot_circle_points = [&](int cx, int cy, int x, int y) {
        draw_pixel(buffer, cx + x, cy + y, img_w, img_h, channels, color);
        draw_pixel(buffer, cx - x, cy + y, img_w, img_h, channels, color);
        draw_pixel(buffer, cx + x, cy - y, img_w, img_h, channels, color);
        draw_pixel(buffer, cx - x, cy - y, img_w, img_h, channels, color);
        draw_pixel(buffer, cx + y, cy + x, img_w, img_h, channels, color);
        draw_pixel(buffer, cx - y, cy + x, img_w, img_h, channels, color);
        draw_pixel(buffer, cx + y, cy - x, img_w, img_h, channels, color);
        draw_pixel(buffer, cx - y, cy - x, img_w, img_h, channels, color);
    };

    while (y >= x) {
        plot_circle_points(center_x, center_y, x, y);
        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
    }
}

void DrawingUtils::draw_filled_circle(uint8_t* buffer, int center_x, int center_y, int radius, 
                                      int img_w, int img_h, int channels, uint16_t color) {
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;

    auto draw_scanline = [&](int y_pos, int x1, int x2) {
        for (int x = x1; x <= x2; ++x) {
            draw_pixel(buffer, x, y_pos, img_w, img_h, channels, color);
        }
    };

    while (y >= x) {
        // Draw horizontal lines logic
        draw_scanline(center_y + x, center_x - y, center_x + y);
        draw_scanline(center_y - x, center_x - y, center_x + y);
        draw_scanline(center_y + y, center_x - x, center_x + x);
        draw_scanline(center_y - y, center_x - x, center_x + x);
        
        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
    }
}

}  // namespace esp32_camera_utils
}  // namespace esphome

#endif // USE_CAMERA_DRAWING
