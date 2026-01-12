// Multi-algorithm needle detection implementations
// This file contains preprocessing and detection algorithms for analog_reader

#include "analog_reader.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <vector>
#include <cstring>

namespace esphome {
namespace analog_reader {

// PREPROCESSING FUNCTIONS

std::vector<uint8_t> AnalogReader::preprocess_image(const uint8_t* img, int w, int h, int cx, int cy, int radius) {
    // Create a copy for preprocessing
    std::vector<uint8_t> processed(w * h);
    memcpy(processed.data(), img, w * h);
    
    // Step 1: CLAHE (Contrast Limited Adaptive Histogram Equalization)
    apply_clahe(processed.data(), w, h);
    
    // Step 2: Remove circular background
    remove_background(processed.data(), w, h, cx, cy, radius);
    
    // Step 3: Median filter to reduce noise
    median_filter_3x3(processed.data(), w, h);
    
    return processed;
}

void AnalogReader::apply_clahe(uint8_t* img, int w, int h, int tile_size) {
    // Simple tiled histogram equalization
    for (int tile_y = 0; tile_y < h; tile_y += tile_size) {
        for (int tile_x = 0; tile_x < w; tile_x += tile_size) {
            // Calculate local histogram
            std::vector<int> hist(256, 0);
            int tile_pixels = 0;
            
            for (int y = tile_y; y < std::min(tile_y + tile_size, h); y++) {
                for (int x = tile_x; x < std::min(tile_x + tile_size, w); x++) {
                    hist[img[y * w + x]]++;
                    tile_pixels++;
                }
            }
            
            // Build CDF
            std::vector<int> cdf(256, 0);
            cdf[0] = hist[0];
            for (int i = 1; i < 256; i++) {
                cdf[i] = cdf[i-1] + hist[i];
            }
            
            // Apply histogram equalization
            for (int y = tile_y; y < std::min(tile_y + tile_size, h); y++) {
                for (int x = tile_x; x < std::min(tile_x + tile_size, w); x++) {
                    uint8_t val = img[y * w + x];
                    if (tile_pixels > 0) {
                        img[y * w + x] = (uint8_t)((cdf[val] * 255) / tile_pixels);
                    }
                }
            }
        }
    }
}

void AnalogReader::remove_background(uint8_t* img, int w, int h, int cx, int cy, int radius) {
    // Calculate average background intensity in outer annular region (80-100% radius)
    float bg_sum = 0;
    int bg_count = 0;
    
    for (int deg = 0; deg < 360; deg += 5) {
        float rad = deg * M_PI / 180.0f;
        for (int r = (int)(radius * 0.8f); r < radius; r++) {
            int px = cx + (int)(r * cos(rad));
            int py = cy + (int)(r * sin(rad));
            if (px >= 0 && px < w && py >= 0 && py < h) {
                bg_sum += img[py * w + px];
                bg_count++;
            }
        }
    }
    
    if (bg_count > 0) {
        float bg_avg = bg_sum / bg_count;
        // Subtract background and enhance contrast
        for (int i = 0; i < w * h; i++) {
            float diff = (float)img[i] - bg_avg;
            img[i] = (uint8_t)std::max(0.0f, std::min(255.0f, diff * 2.0f + 128.0f));
        }
    }
}

void AnalogReader::median_filter_3x3(uint8_t* img, int w, int h) {
    std::vector<uint8_t> temp(w * h);
    memcpy(temp.data(), img, w * h);
    
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            std::vector<uint8_t> window;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    window.push_back(temp[(y + dy) * w + (x + dx)]);
                }
            }
            std::nth_element(window.begin(), window.begin() + 4, window.end());
            img[y * w + x] = window[4];  // Median value
        }
    }
}

// DETECTION ALGORITHMS

AnalogReader::DetectionResult AnalogReader::detect_radial_profile(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    std::vector<float> radial_profile(360, 0.0f);
    std::vector<float> edge_strength(360, 0.0f);
    
    // Scan range: use configurable constants
    int start_r = (int)(radius * kScanStartRadius);
    int end_r = (int)(radius * kScanEndRadius);
    
    for (int deg = 0; deg < 360; deg++) {
        float rad = deg * M_PI / 180.0f;
        float dx = cos(rad);
        float dy = sin(rad);
        
        std::vector<uint8_t> ray_values;
        
        // Sample along ray
        for (int r = start_r; r < end_r; r++) {
            int px = cx + (int)(r * dx);
            int py = cy + (int)(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                ray_values.push_back(img[py * w + px]);
            }
        }
        
        if (ray_values.size() > 0) {
            // Calculate average intensity along ray
            float sum = 0;
            for (auto val : ray_values) sum += val;
            radial_profile[deg] = sum / ray_values.size();
            
            // Calculate edge strength (gradient between inner and outer parts)
            if (ray_values.size() > 2) {
                int mid = ray_values.size() / 2;
                float inner_avg = 0, outer_avg = 0;
                for (int i = 0; i < mid; i++) inner_avg += ray_values[i];
                for (size_t i = mid; i < ray_values.size(); i++) outer_avg += ray_values[i];
                inner_avg /= mid;
                outer_avg /= (ray_values.size() - mid);
                edge_strength[deg] = fabs(outer_avg - inner_avg);
            }
        }
    }
    
    // Find needle using combined criteria
    float best_score = -FLT_MAX;
    float best_angle = 0;
    
    for (int deg = 0; deg < 360; deg++) {
        // Intensity score depends on needle type
        float intensity_score;
        if (dial.needle_type == NEEDLE_TYPE_LIGHT) {
            intensity_score = radial_profile[deg];  // Light needle = high value
        } else {
            intensity_score = 255.0f - radial_profile[deg];  // Dark needle = low value
        }
        
        // Combined score: use configurable weights
        float combined_score = intensity_score * kIntensityWeight + edge_strength[deg] * kEdgeWeight;
        
        if (combined_score > best_score) {
            best_score = combined_score;
            best_angle = (float)deg;
        }
    }
    
    // Calculate confidence (0-1)
    float min_score = *std::min_element(radial_profile.begin(), radial_profile.end());
    float max_score = *std::max_element(radial_profile.begin(), radial_profile.end());
    float confidence = (max_score > min_score) ? (best_score / (max_score * kIntensityWeight + 255.0f * kEdgeWeight)) : 0.5f;
    
    return {best_angle, confidence, "radial_profile"};
}

AnalogReader::DetectionResult AnalogReader::detect_hough_transform(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    // Step 1: Simple edge detection (Sobel-like)
    std::vector<uint8_t> edges(w * h, 0);
    
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int gx = -img[(y-1)*w + (x-1)] - 2*img[y*w + (x-1)] - img[(y+1)*w + (x-1)]
                    + img[(y-1)*w + (x+1)] + 2*img[y*w + (x+1)] + img[(y+1)*w + (x+1)];
            
            int gy = -img[(y-1)*w + (x-1)] - 2*img[(y-1)*w + x] - img[(y-1)*w + (x+1)]
                    + img[(y+1)*w + (x-1)] + 2*img[(y+1)*w + x] + img[(y+1)*w + (x+1)];
            
            int magnitude = (int)sqrt(gx * gx + gy * gy);
            edges[y*w + x] = (magnitude > 30) ? 255 : 0;  // Threshold
        }
    }
    
    // Step 2: Polar Hough accumulator for lines through center
    const int NUM_ANGLES = 360;
    std::vector<int> accumulator(NUM_ANGLES, 0);
    
    // Only consider edges in the dial area (30-90% radius)
    int min_r = (int)(radius * 0.3f);
    int max_r = (int)(radius * 0.9f);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (edges[y*w + x] > 0) {
                float dx = x - cx;
                float dy = y - cy;
                float r = sqrt(dx * dx + dy * dy);
                
                if (r >= min_r && r <= max_r) {
                    // Calculate angle from center to this edge pixel
                    float theta = atan2(dy, dx) * 180.0f / M_PI;
                    if (theta < 0) theta += 360.0f;
                    
                    int angle_idx = (int)theta % NUM_ANGLES;
                    accumulator[angle_idx]++;
                }
            }
        }
    }
    
    // Find angle with maximum votes
    int max_votes = 0;
    int best_angle_idx = 0;
    
    for (int i = 0; i < NUM_ANGLES; i++) {
        if (accumulator[i] > max_votes) {
            max_votes = accumulator[i];
            best_angle_idx = i;
        }
    }
    
    float confidence = std::min(1.0f, (float)max_votes / (radius * 0.1f));  // Normalized
    
    return {(float)best_angle_idx, confidence, "hough_transform"};
}

AnalogReader::DetectionResult AnalogReader::detect_template_match(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    // Coarse search every 10 degrees
    const int COARSE_STEP = 10;
    float best_score = -FLT_MAX;
    float best_angle = 0;
    
    for (int deg = 0; deg < 360; deg += COARSE_STEP) {
        float rad = deg * M_PI / 180.0f;
        float dx = cos(rad);
        float dy = sin(rad);
        
        float score = 0.0f;
        int count = 0;
        
        // Sample along needle direction (30-90% radius)
        int start_r = (int)(radius * 0.3f);
        int end_r = (int)(radius * 0.9f);
        
        for (int r = start_r; r < end_r; r++) {
            int px = cx + (int)(r * dx);
            int py = cy + (int)(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                // Score based on needle type
                float pixel_val = img[py * w + px];
                if (dial.needle_type == NEEDLE_TYPE_DARK) {
                    score += (255.0f - pixel_val);  // Dark needle = low values
                } else {
                    score += pixel_val;  // Light needle = high values
                }
                count++;
            }
        }
        
        if (count > 0) {
            score /= count;
            if (score > best_score) {
                best_score = score;
                best_angle = (float)deg;
            }
        }
    }
    
    // Fine search around best angle (±5 degrees, 1 degree steps)
    float refined_angle = best_angle;
    float refined_score = best_score;
    
    for (int offset = -5; offset <= 5; offset++) {
        float test_angle = best_angle + offset;
        if (test_angle < 0) test_angle += 360.0f;
        if (test_angle >= 360.0f) test_angle -= 360.0f;
        
        float rad = test_angle * M_PI / 180.0f;
        float dx = cos(rad);
        float dy = sin(rad);
        
        float score = 0.0f;
        int count = 0;
        
        int start_r = (int)(radius * 0.3f);
        int end_r = (int)(radius * 0.9f);
        
        for (int r = start_r; r < end_r; r++) {
            int px = cx + (int)(r * dx);
            int py = cy + (int)(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                float pixel_val = img[py * w + px];
                if (dial.needle_type == NEEDLE_TYPE_DARK) {
                    score += (255.0f - pixel_val);
                } else {
                    score += pixel_val;
                }
                count++;
            }
        }
        
        if (count > 0) {
            score /= count;
            if (score > refined_score) {
                refined_score = score;
                refined_angle = test_angle;
            }
        }
    }
    
    float confidence = refined_score / 255.0f;
    
    return {refined_angle, confidence, "template_match"};
}

// FILE CONTINUES - This is multi_algorithm.cpp

}  // namespace analog_reader
}  // namespace esphome
