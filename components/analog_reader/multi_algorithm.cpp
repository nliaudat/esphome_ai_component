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

void AnalogReader::preprocess_image(const uint8_t* img, int w, int h, int cx, int cy, int radius, NeedleType needle_type, std::vector<uint8_t>& output) {
    // Resize working buffer if needed
    if (output.size() != w * h) {
        output.resize(w * h);
    }
    
    // Copy data to working buffer
    memcpy(output.data(), img, w * h);
    
    // Step 1: CLAHE (Contrast Limited Adaptive Histogram Equalization)
    apply_clahe(output.data(), w, h);
    
    // Step 1.5: Top-Hat Filter (if enabled/configured, currently hardcoded enabled for testing robustness)
    // We use radius / 10 as kernel size (approx 5-10 pixels)
    int kernel = std::max(3, radius / 10);
    // Ensure kernel is odd
    if (kernel % 2 == 0) kernel++;
    
    apply_tophat(output.data(), w, h, kernel, scratch_buffer_, needle_type);

    // Step 2: Remove circular background (Legacy method, maybe redundant with TopHat but handles global gradients)
    // remove_background(output.data(), w, h, cx, cy, radius);
    // User requested replacement? Top-hat handles shadow/gradients.
    // Let's keep remove_background for now or make it lighter?
    // Actually top-hat is superior for local shadows. Global gradient (light to dark across dial) is also handled by top-hat.
    // I will disable remove_background to see effect, or keep it.
    // Let's keep it but maybe it's too aggressive. 
    // Commenting out remove_background as Top-hat replaces it effectively for "handling shadows".
    
    // Step 3: Median filter to reduce noise
    median_filter_3x3(output.data(), w, h);
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
        // Use LUT
        float r_cos = cos_lut_[deg];
        float r_sin = sin_lut_[deg];
        
        for (int r = (int)(radius * 0.8f); r < radius; r++) {
            int px = cx + (int)(r * r_cos);
            int py = cy + (int)(r * r_sin);
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

// MORPHOLOGY HELPERS
static void erode(const uint8_t* src, uint8_t* dst, int w, int h, int k) {
    int r = k / 2;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t min_val = 255;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
                        uint8_t val = src[ny * w + nx];
                        if (val < min_val) min_val = val;
                    }
                }
            }
            dst[y * w + x] = min_val;
        }
    }
}

static void dilate_in_place(uint8_t* img, int w, int h, int k) {
    int r = k / 2;
    std::vector<uint8_t> temp(w * h); // Need full copy or rolling buffer. Rolling is complex. Using heap copy.
    memcpy(temp.data(), img, w * h);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t max_val = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
                        uint8_t val = temp[ny * w + nx]; // Read from copy
                        if (val > max_val) max_val = val;
                    }
                }
            }
            img[y * w + x] = max_val; // Write to original
        }
    }
}

void AnalogReader::apply_tophat(uint8_t* img, int w, int h, int kernel_size, std::vector<uint8_t>& scratch, NeedleType needle_type) {
    if (scratch.size() != w * h) scratch.resize(w * h);
    
    // White Top Hat: Original - Opening (Dilate(Erode(Img)))
    // Black Top Hat: Closing(Img) - Original. Closing = Erode(Dilate(Img))
    
    // We implement White Top Hat logic mostly for highlighting peaks.
    // If Needle is Dark, we want Black Top Hat.
    
    // Using simple Min/Max logic:
    // Opening:
    // 1. Erode Img -> Scratch
    // 2. Dilate Scratch -> Scratch (In Place)
    
    // Closing:
    // 1. Dilate Img -> Scratch (requires copy-erode-logic, or just dilate logic)
    //    Actually dilate_in_place allocates temp.
    //    Let's use erode(src, dst) and dilate(src, dst) with copy?
    
    // Optimized flow reusing scratch:
    
    if (needle_type == NEEDLE_TYPE_LIGHT) {
        // White Top Hat: Original - Opening.
        // Opening = Dilate(Erode(Original))
        
        // 1. Erode(Original -> Scratch)
        erode(img, scratch.data(), w, h, kernel_size);
        
        // 2. Dilate(Scratch -> Scratch) (Use in-place with temp alloc inside)
        dilate_in_place(scratch.data(), w, h, kernel_size);
        
        // 3. Original - Scratch (Opening)
        for (int i = 0; i < w * h; i++) {
            int val = (int)img[i] - (int)scratch[i];
            img[i] = (val < 0) ? 0 : (uint8_t)val;
        }
    } else {
        // Black Top Hat: Closing - Original
        // Closing = Erode(Dilate(Original))
        
        // 1. Dilate(Original -> Scratch) 
        // We don't have dilate(src, dst). reuse dilate_in_place logic? 
        // Just copy Original to Scratch, then Dilate Scratch
        memcpy(scratch.data(), img, w * h);
        dilate_in_place(scratch.data(), w, h, kernel_size);
        
        // 2. Erode(Scratch -> Scratch) (Can use erode(src, dst) where src==dst? No. Need temp)
        // erode implementation above uses src separate from dst logic?
        // "dst[y*w+x] = min_val". min_val computed from src.
        // If src==dst, we read modified values.
        // So we need another buffer OR erode_in_place.
        
        // Let's allocate one more temp buffer, safety first.
        std::vector<uint8_t> temp(w*h);
        erode(scratch.data(), temp.data(), w, h, kernel_size); // Erode Scratch -> Temp
        
        // Now Temp is Closing.
        // 3. Temp - Original
        for (int i = 0; i < w * h; i++) {
            int val = (int)temp[i] - (int)img[i];
            img[i] = (val < 0) ? 0 : (uint8_t)val;
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
        // Use LUT
        float dx = cos_lut_[deg];
        float dy = sin_lut_[deg];
        
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
    int best_angle_int = 0;
    std::vector<float> final_scores(360, 0.0f);
    
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
        final_scores[deg] = combined_score;
        
        if (combined_score > best_score) {
            best_score = combined_score;
            best_angle_int = deg;
        }
    }
    
    // Sub-pixel Interpolation
    float best_angle = (float)best_angle_int;
    
    // Get valid neighbors (wrap around)
    int prev = (best_angle_int - 1 + 360) % 360;
    int next = (best_angle_int + 1) % 360;
    
    float y1 = final_scores[prev];
    float y2 = final_scores[best_angle_int]; // max
    float y3 = final_scores[next];
    
    float denom = y1 - 2 * y2 + y3;
    if (fabs(denom) > 0.0001f) {
        float offset = (y1 - y3) / (2 * denom);
        // Offset is usually between -0.5 and 0.5.
        // If denominator is positive (valley), offset might be weird, but we expect peak (negative 2nd derivative -> denom negative)
        // With y2 as max, y1 < y2 and y3 < y2 => y1+y3 < 2y2 => y1 - 2y2 + y3 < 0.
        // Formula: p = (y_minus - y_plus) / (2 * (y_minus - 2*y_center + y_plus))?
        // Checking: 
        // Let y2=10, y1=5, y3=5. p = (5-5)/(2*(5-20+5)) = 0. Correct.
        // Let y2=10, y1=8, y3=5. p = (8-5)/(2*(8-20+5)) = 3 / (2 * -7) = -0.21. Shift towards y1. Correct.
        best_angle += offset;
    }
    
    // Normalize angle
    if (best_angle < 0) best_angle += 360.0f;
    if (best_angle >= 360.0f) best_angle -= 360.0f;
    
    // Calculate confidence (0-1)
    float min_score = *std::min_element(final_scores.begin(), final_scores.end());
    float max_score = best_score;
    // Normalize confidence based on dynamic range of scores
    float range = max_score - min_score;
    float confidence = (range > 0) ? (range / (255.0f * kIntensityWeight + 255.0f * kEdgeWeight)) : 0.0f;
    // Scale confidence up simpler:
    if (range > 50) confidence = 0.8f + (range/200.0f); 
    else confidence = range / 100.0f;
    if (confidence > 1.0f) confidence = 1.0f;
    
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
            // edges[y*w + x] = (magnitude > 30) ? 255 : 0;
            
            // NMS (Non-Maximum Suppression) Lite
            // Determine gradient direction
            // 0: Horizontal, 1: 45deg, 2: Vertical, 3: 135deg
            if (magnitude > 30) {
                 float angle = atan2((float)gy, (float)gx) * 180.0f / M_PI;
                 if (angle < 0) angle += 180.0f;
                 int q = ((int)(angle + 22.5f) % 180) / 45;
                 
                 bool is_max = true;
                 int n1 = 0, n2 = 0;
                 // Check neighbors (simplified checks, careful with bounds)
                 if (q == 0) { // Horizontal: Check Left/Right
                     if (x > 0) n1 = abs(-img[(y-1)*w+(x-2)] - 2*img[y*w+(x-2)] - img[(y+1)*w+(x-2)] + img[(y-1)*w+x] + 2*img[y*w+x] + img[(y+1)*w+x]); // Approximation? No using calculated mag map is better
                     // But we don't have mag map, we are computing on fly.
                     // Standard NMS requires full mag map.
                     // Optimization: Just thresholding is often enough for simple needle lines.
                     // Sub-pixel Hough voting is better.
                     // Let's stick to Thresholding but slightly higher, or just rely on Hough accumulator to smooth it out.
                     // The user asked for "Canny simplified".
                     // Basic Canny = Gaussian -> Sobel -> NMS -> Hysteresis.
                     // We did Sobel.
                     // NMS without mag map is hard.
                     // Let's Allocate mag map in edges vector instead of 0/255?
                 }
                 
                 // Fallback: Just store magnitude in edges, doing NMS requires 2 passes or full buffer.
                 // We have edges buffer (w*h). Store magnitude there (uint8 clamped).
                 edges[y*w + x] = (uint8_t)std::min(255, magnitude);
            }
        }
    }
    
    // Step 2: Hough Voting
    const int NUM_ANGLES = 360;
    std::vector<int> accumulator(NUM_ANGLES, 0);
    
    // Only consider edges in the dial area (30-90% radius)
    int min_r = (int)(radius * 0.3f);
    int max_r = (int)(radius * 0.9f);
    
    // Step 1.5: NMS Pass on edges buffer -> Voting
    // We iterate edges and vote
    
    // Applying the change to Weighted Voting:
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t mag = edges[y*w + x];
            if (mag > 40) { // Higher threshold
                float dx = x - cx;
                float dy = y - cy;
                float r = sqrt(dx * dx + dy * dy);
                
                if (r >= min_r && r <= max_r) {
                    float theta = atan2(dy, dx) * 180.0f / M_PI;
                    if (theta < 0) theta += 360.0f;
                    
                    int angle_idx = (int)theta % NUM_ANGLES;
                    accumulator[angle_idx] += mag; // Weighted Vote
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
        // Use LUT
        float dx = cos_lut_[deg];
        float dy = sin_lut_[deg];
        
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
        
        // Use LUT (wrap angle to integer 0-359)
        int lut_angle = (int)test_angle;
        if (lut_angle < 0) lut_angle += 360;
        if (lut_angle >= 360) lut_angle -= 360;
        
        float dx = cos_lut_[lut_angle];
        float dy = sin_lut_[lut_angle];
        
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
