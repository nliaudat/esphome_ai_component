# Radial Profile Algorithm

## Overview
This is the default and most versatile algorithm for analyzing analog dials. It works by scanning the image radially from the center outward, creating a 360-degree intensity profile.

## How It Works

1.  **Preprocessing**: 
    *   The image crop is enhanced using CLAHE (Contrast Limited Adaptive Histogram Equalization) to handle uneven lighting.
    *   If enabled, a **Top-Hat Filter** is applied to isolate the needle structure from shadows and reflections.
    *   If **Color Detection** is configured, the image is converted to a "Color Distance Map". Pixels matching the target color become **dark** (low value), while non-matching pixels become **light**. This allows the standard detection logic (which searches for dark needles) to work instantly.

2.  **Radial Scanning**:
    *   The algorithm casts 360 imaginary rays (one for each degree) from the center of the crop.
    *   For each ray, it samples pixel values from 30% radius (skipping the central hub) to 90% radius (avoiding outer markings).
    *   **Intensity Score**: Calculates the average brightness along the ray.
    *   **Edge Strength**: Calculates the contrast between the inner part and outer part of the ray (detecting the tip of the needle).

3.  **Scoring**:
    *   A combined score is calculated for each angle: `Score = (Intensity * Weight) + (Edge * Weight)`.
    *   For dark needles, the intensity is inverted (searching for darkest line).
    *   Trigonometric Look-Up Tables (LUTs) ensure this process is extremely fast.

4.  **Sub-pixel Refinement**:
    *   Once the best integer angle (e.g., 45°) is found, the algorithm looks at the scores of its neighbors (44° and 46°).
    *   It uses **Parabolic Interpolation** to calculate the peak's true position (e.g., 45.3°), providing higher precision than the scan step size.

## Best For
*   Standard needle gauges (pressure, temperature, flow).
*   High-contrast dials (black on white, white on black).
*   Colored needles (Red, Orange) when using `target_color`.
