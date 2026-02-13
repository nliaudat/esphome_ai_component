// Multi-algorithm needle detection implementations
// This file contains preprocessing and detection algorithms for analog_reader

#include "analog_reader.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <vector>
#include <cstring>
#include <esp_heap_caps.h>

namespace esphome {
namespace analog_reader {

static const char *const TAG = "multi_algorithm";

// PREPROCESSING FUNCTIONS

void AnalogReader::preprocess_image(const uint8_t* img, int w, int h, int cx, int cy, int radius, NeedleType needle_type, std::vector<uint8_t>& output) {
    // Safe Resize Logic
    size_t required_size = w * h;
    
    // Helper lambda to safely resize standard vectors.
    auto safe_resize = [&](std::vector<uint8_t>& vec, const char* name) -> bool {
        if (vec.size() != required_size) {
            // Check if we need to allocate new memory (capacity increase)
            if (vec.capacity() < required_size) {
                // Check if enough contiguous memory exists to avoid hard crash (abort)
                // Note: std::vector resize often allocates new block then copies, so we need free block >= new size.
                // However, overhead might be higher. This is a best-effort check.
                size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                if (largest_block < required_size) {
                    ESP_LOGE(TAG, "CRITICAL: Not enough memory to resize %s. Need %u, Largest Free %u", name, required_size, largest_block);
                    return false;
                }
            }
            vec.resize(required_size);
        }
        return true;
    };

    if (!safe_resize(output, "output")) return;
    
    // Copy data to working buffer
    memcpy(output.data(), img, w * h);

    if (!safe_resize(this->scratch_buffer_, "scratch1")) return;
    if (!safe_resize(this->scratch_buffer_2_, "scratch2")) return;

    // Step 1: CLAHE (Contrast Limited Adaptive Histogram Equalization)
    this->apply_clahe(output.data(), w, h);
    
    // Step 1.5: Top-Hat Filter (if enabled/configured, currently hardcoded enabled for testing robustness)
    // We use radius / 10 as kernel size (approx 5-10 pixels)
    int kernel = std::max(3, radius / 10);
    // Ensure kernel is odd
    if (kernel % 2 == 0) kernel++;
    
    this->apply_tophat(output.data(), w, h, kernel, this->scratch_buffer_, this->scratch_buffer_2_, needle_type);

    // Step 2: Remove circular background
    // (Disabled as Top-hat covers it, but keeping placeholder comment)
    
    // Step 3: Median filter to reduce noise
    this->median_filter_3x3(output.data(), w, h);

    if (this->debug_) {
        ESP_LOGD(TAG, "Preprocessing complete. NeedleType: %s, Kernel: %d", 
                 needle_type == NEEDLE_TYPE_LIGHT ? "Light" : "Dark", kernel);
    }
}

void AnalogReader::apply_clahe(uint8_t* img, int w, int h, int tile_size) {
    // Simple tiled histogram equalization
    // Pre-allocate scratch arrays outside the loop to avoid per-tile heap allocations
    std::vector<int> hist(256);
    std::vector<int> cdf(256);
    for (int tile_y = 0; tile_y < h; tile_y += tile_size) {
        for (int tile_x = 0; tile_x < w; tile_x += tile_size) {
            // Calculate local histogram
            std::fill(hist.begin(), hist.end(), 0);
            int tile_pixels = 0;
            
            for (int y = tile_y; y < std::min(tile_y + tile_size, h); y++) {
                for (int x = tile_x; x < std::min(tile_x + tile_size, w); x++) {
                    hist[img[y * w + x]]++;
                    tile_pixels++;
                }
            }
            
            // Build CDF
            cdf[0] = hist[0];
            for (int i = 1; i < 256; i++) {
                cdf[i] = cdf[i-1] + hist[i];
            }
            
            // Apply histogram equalization
            for (int y = tile_y; y < std::min(tile_y + tile_size, h); y++) {
                for (int x = tile_x; x < std::min(tile_x + tile_size, w); x++) {
                    uint8_t val = img[y * w + x];
                    if (tile_pixels > 0) {
                        img[y * w + x] = static_cast<uint8_t>((cdf[val] * 255) / tile_pixels);
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
        float r_cos = AnalogReader::cos_lut_[deg];
        float r_sin = AnalogReader::sin_lut_[deg];
        
        for (int r = static_cast<int>(radius * 0.8f); r < radius; r++) {
            int px = cx + static_cast<int>(r * r_cos);
            int py = cy + static_cast<int>(r * r_sin);
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
            float diff = static_cast<float>(img[i]) - bg_avg;
            img[i] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, diff * 2.0f + 128.0f)));
        }
    }
}

void AnalogReader::median_filter_3x3(uint8_t* img, int w, int h) {
    std::vector<uint8_t> temp(w * h);
    memcpy(temp.data(), img, w * h);
    
    // Fixed-size window on stack — avoids heap allocation per pixel
    uint8_t window[9];
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int count = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    window[count++] = temp[(y + dy) * w + (x + dx)];
                }
            }
            std::nth_element(window, window + 4, window + count);
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

static void dilate_in_place(uint8_t* img, int w, int h, int k, uint8_t* temp_buffer) {
    int r = k / 2;
    // Use provided temp buffer instead of allocating vector
    // std::vector<uint8_t> temp(w * h); 
    memcpy(temp_buffer, img, w * h);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t max_val = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
                        uint8_t val = temp_buffer[ny * w + nx]; // Read from copy
                        if (val > max_val) max_val = val;
                    }
                }
            }
            img[y * w + x] = max_val; // Write to original
        }
    }
}

void AnalogReader::apply_tophat(uint8_t* img, int w, int h, int kernel_size, std::vector<uint8_t>& scratch, std::vector<uint8_t>& scratch2, NeedleType needle_type) {
    if (scratch.size() != static_cast<size_t>(w * h)) scratch.resize(w * h);
    if (scratch2.size() != static_cast<size_t>(w * h)) scratch2.resize(w * h); // Should be resized by caller but safety
    
    // White Top Hat: Original - Opening (Dilate(Erode(Img)))
    // Black Top Hat: Closing(Img) - Original. Closing = Erode(Dilate(Img))
    
    if (needle_type == NEEDLE_TYPE_LIGHT) {
        // White Top Hat: Original - Opening.
        // Opening = Dilate(Erode(Original))
        
        // 1. Erode(Original -> Scratch)
        erode(img, scratch.data(), w, h, kernel_size);
        
        // 2. Dilate(Scratch -> Scratch) (Use in-place with temp alloc provided)
        dilate_in_place(scratch.data(), w, h, kernel_size, scratch2.data());
        
        // 3. Original - Scratch (Opening)
        for (int i = 0; i < w * h; i++) {
            int val = static_cast<int>(img[i]) - static_cast<int>(scratch[i]);
            img[i] = (val < 0) ? 0 : static_cast<uint8_t>(val);
        }
    } else {
        // Black Top Hat: Closing - Original
        // Closing = Erode(Dilate(Original))
        
        // 1. Dilate(Original -> Scratch) 
        // Just copy Original to Scratch, then Dilate Scratch
        memcpy(scratch.data(), img, w * h);
        dilate_in_place(scratch.data(), w, h, kernel_size, scratch2.data());
        
        // 2. Erode(Scratch -> Scratch2) 
        // Use scratch2 as destination for erode
        erode(scratch.data(), scratch2.data(), w, h, kernel_size); // Erode Scratch -> Scratch2
        
        // Now Scratch2 is Closing.
        // 3. Scratch2 - Original
        for (int i = 0; i < w * h; i++) {
            int val = static_cast<int>(scratch2[i]) - static_cast<int>(img[i]);
            img[i] = (val < 0) ? 0 : static_cast<uint8_t>(val);
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
    
    // Connectivity Scan
    int start_r = 5; // Start close to center (skip nut)
    int max_radius_scan = static_cast<int>(radius * dial.max_scan_radius);
    
    // Configurable gap threshold
    const int MAX_GAP = 5;

    for (int deg = 0; deg < 360; deg++) {
        float dx = AnalogReader::cos_lut_[deg];
        float dy = AnalogReader::sin_lut_[deg];
        
        float score = 0.0f;
        int gap_count = 0;
        int pixel_count = 0;

        // Trace ray from center outwards
        for (int r = start_r; r < max_radius_scan; r++) {
            int px = cx + static_cast<int>(r * dx);
            int py = cy + static_cast<int>(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                uint8_t val = img[py * w + px];
                
                // TopHat image: Needle is always BRIGHT (High Value)
                // Threshold to consider "active detection"
                if (val > 20) { 
                    score += val;
                    gap_count = 0; // Reset gap
                } else {
                    gap_count++;
                    // If we hit a large gap, assume needle ended (e.g. this was the tail, or just noise)
                     // Penalize the gap? No, just don't add to score.
                    if (gap_count > MAX_GAP) {
                         break; // Stop tracing this ray
                    }
                }
                pixel_count++;
            } else {
                break;
            }
        }
        
        // Edge strength calculation (simplified radial gradient)
        // We can reuse the loop or keep it simple.
        // For now, rely on Intensity Score which is now robustly "Length * Brightness".
        
        radial_profile[deg] = score; 
    }
    
    // Find needle using combined criteria
    float best_score = -FLT_MAX;
    int best_angle_int = 0;
    std::vector<float> final_scores(360, 0.0f);
    
    for (int deg = 0; deg < 360; deg++) {
        // Since we process TopHat, higher value is ALWAYS better for needle presence.
        // No need to invert for NEEDLE_TYPE_DARK.
        float final_score = radial_profile[deg];
        
        // Enhance with Neighbor averaging to smooth out noise
        // (Optional, but good for stability)
        
        final_scores[deg] = final_score;
        
        if (final_score > best_score) {
            best_score = final_score;
            best_angle_int = deg;
        }
    }
    
    // Sub-pixel Interpolation
    float best_angle = static_cast<float>(best_angle_int);
    
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
    
    if (this->debug_) {
        ESP_LOGD(TAG, "Radial Profile: Best Angle=%.1f, Conf=%.2f", best_angle, confidence);
    }
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
            
            int magnitude = static_cast<int>(sqrt(gx * gx + gy * gy));
            // edges[y*w + x] = (magnitude > 30) ? 255 : 0;
            
            // NMS (Non-Maximum Suppression) Lite
            // Determine gradient direction
            // 0: Horizontal, 1: 45deg, 2: Vertical, 3: 135deg
            if (magnitude > 30) {
                 float angle = atan2(static_cast<float>(gy), static_cast<float>(gx)) * 180.0f / static_cast<float>(M_PI);
                 if (angle < 0) angle += 180.0f;
                 int q = (static_cast<int>(angle + 22.5f) % 180) / 45;
                 
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
                 edges[y*w + x] = static_cast<uint8_t>(std::min(255, magnitude));
            }
        }
    }
    
    // Step 2: Hough Voting
    const int NUM_ANGLES = 360;
    std::vector<int> accumulator(NUM_ANGLES, 0);
    
    // Only consider edges in the dial area (30-90% radius)
    int min_r = static_cast<int>(radius * dial.min_scan_radius);
    int max_r = static_cast<int>(radius * dial.max_scan_radius);
    
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
                    float theta = atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
                    if (theta < 0) theta += 360.0f;
                    
                    int angle_idx = static_cast<int>(theta) % NUM_ANGLES;
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
    
    float confidence = std::min(1.0f, static_cast<float>(max_votes) / (radius * 0.1f));  // Normalized
    
    if (this->debug_) {
        ESP_LOGD(TAG, "Hough Transform: Best Angle Index=%d, Conf=%.2f", best_angle_idx, confidence);
    }
    return {static_cast<float>(best_angle_idx), confidence, "hough_transform"};
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
        float dx = AnalogReader::cos_lut_[deg];
        float dy = AnalogReader::sin_lut_[deg];
        
        float score = 0.0f;
        int count = 0;
        
        // Sample along needle direction (30-90% radius)
        int start_r = static_cast<int>(radius * dial.min_scan_radius);
        int end_r = static_cast<int>(radius * dial.max_scan_radius);
        
        for (int r = start_r; r < end_r; r++) {
            int px = cx + static_cast<int>(r * dx);
            int py = cy + static_cast<int>(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                // Score based on needle type - Preprocessed is always Bright
                float pixel_val = img[py * w + px];
                score += pixel_val;
                count++;
            }
        }
        
        if (count > 0) {
            score /= count;
            if (score > best_score) {
                best_score = score;
                best_angle = static_cast<float>(deg);
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
        int lut_angle = static_cast<int>(test_angle);
        if (lut_angle < 0) lut_angle += 360;
        if (lut_angle >= 360) lut_angle -= 360;
        
        float dx = AnalogReader::cos_lut_[lut_angle];
        float dy = AnalogReader::sin_lut_[lut_angle];
        
        float score = 0.0f;
        int count = 0;
        
        int start_r = static_cast<int>(radius * dial.min_scan_radius);
        int end_r = static_cast<int>(radius * dial.max_scan_radius);
        
        for (int r = start_r; r < end_r; r++) {
            int px = cx + static_cast<int>(r * dx);
            int py = cy + static_cast<int>(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                // Preprocessed image (TopHat) always has Bright Needle
                float pixel_val = img[py * w + px];
                score += pixel_val;
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
    
    if (this->debug_) {
        ESP_LOGD(TAG, "Template Match: Refined Angle=%.1f, Conf=%.2f", refined_angle, confidence);
    }
    return {refined_angle, confidence, "template_match"};
}


AnalogReader::DetectionResult AnalogReader::detect_legacy(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    // Original Enhanced Radial Profile Analysis (from checkpoint, before multi-algorithm)
    // Combines average intensity along each ray with edge strength detection
    // Score = 70% intensity + 30% edge gradient (inner vs outer parts)
    
    std::vector<float> radial_profile(360, 0.0f);
    std::vector<float> edge_strength(360, 0.0f);
    
    // Ray Continuity Scan
    int start_r = static_cast<int>(radius * 0.15f); // Start close to center
    int end_r = static_cast<int>(radius * kScanEndRadius); // Use existing end_r
    
    const int MAX_GAP = 5;

    for (int deg = 0; deg < 360; deg++) {
        float rad = deg * static_cast<float>(M_PI) / 180.0f;
        float dx = cos(rad);
        float dy = sin(rad);
        
        float score = 0.0f;
        int gap_count = 0;
        
        // Scan ray from center outwards
        for (int r = start_r; r < end_r; r++) {
            int px = cx + lround(r * dx);
            int py = cy + lround(r * dy);
            
            if (px >= 0 && px < w && py >= 0 && py < h) {
                uint8_t val = img[py * w + px];
                bool is_needle_pixel = false;
                
                // Logic depends on needle type (RAW image)
                if (dial.needle_type == NEEDLE_TYPE_LIGHT) {
                    if (val > 128) is_needle_pixel = true; // Simple threshold for now, or relative to avg
                    score += val; // Accumulate intensity
                } else {
                    // Dark needle
                    if (val < 128) is_needle_pixel = true;
                    score += (255 - val); // Accumulate darkness
                }
                
                // Connectivity Check
                // Basic check: If pixel is "Background-ish", gap++.
                // Assume background is > 100 for Dark Needle?
                if (dial.needle_type == NEEDLE_TYPE_DARK && val > 150) gap_count++;
                else if (dial.needle_type == NEEDLE_TYPE_LIGHT && val < 100) gap_count++;
                else gap_count = 0;
                
                if (gap_count > MAX_GAP) break; // Terminate ray
            }
        }
        radial_profile[deg] = score;
        
        // Edge strength is hard to do with varying lengths. Set to 0 or derive from score.
        // We will just use the score.
        edge_strength[deg] = 0; 
    }
    
    // Find needle using combined criteria
    float best_score = -FLT_MAX;
    float best_angle = 0;
    
    for (int deg = 0; deg < 360; deg++) {
        // Intensity score depends on needle type
        // Note: radial_profile[deg] already contains inverted values for dark needles (255 - val)
        // So we use it directly without a second inversion
        float intensity_score;
        if (dial.needle_type == NEEDLE_TYPE_LIGHT) {
            intensity_score = radial_profile[deg];  // Light needle = high value
        } else {
            intensity_score = radial_profile[deg];  // Dark needle = already inverted, use directly
        }
        
        // Combined score with weights (edge_strength is 0, so only intensity matters)
        float combined_score = intensity_score * kIntensityWeight + edge_strength[deg] * kEdgeWeight;
        
        if (combined_score > best_score) {
            best_score = combined_score;
            best_angle = static_cast<float>(deg);
        }
    }
    
    // Calculate confidence (0-1) based on dynamic range (signal-to-noise ratio)
    // Since edge_strength is 0, we use only the radial_profile range
    float min_score = *std::min_element(radial_profile.begin(), radial_profile.end());
    float max_score = *std::max_element(radial_profile.begin(), radial_profile.end());
    float confidence = (max_score > min_score) ? ((max_score - min_score) / max_score) : 0.5f;
    
    return {best_angle, confidence, "legacy"};
}

}  // namespace analog_reader
}  // namespace esphome
