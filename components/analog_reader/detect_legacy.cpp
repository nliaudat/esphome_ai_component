// Legacy detection algorithm - original radial edge detection (no preprocessing)
// This file implements the original needle detection before multi-algorithm system

#include "analog_reader.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace esphome {
namespace analog_reader {

AnalogReader::DetectionResult AnalogReader::detect_legacy(const uint8_t* img, int w, int h, const DialConfig& dial) {
    int cx = w / 2;
    int cy = h / 2;
    int radius = std::min(cx, cy) - 2;
    
    // Original Enhanced Radial Profile Analysis (from checkpoint, before multi-algorithm)
    // Combines average intensity along each ray with edge strength detection
    // Score = 70% intensity + 30% edge gradient (inner vs outer parts)
    
    std::vector<float> radial_profile(360, 0.0f);
    std::vector<float> edge_strength(360, 0.0f);
    
    // Scan range: 30-90% radius
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
        
        // Combined score with weights
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
    
    return {best_angle, confidence, "legacy"};
}

}  // namespace analog_reader
}  // namespace esphome
