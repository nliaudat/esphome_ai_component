# Hough Transform Algorithm (Weighted Voting)

## Overview
The Hough Transform is a robust computer vision technique used for detecting lines and shapes. In this component, it is optimized to find lines passing through the center of the dial.

## How It Works

1.  **Edge Detection**:
    *   The image first passes through a Sobel filter to detect edges (sudden changes in brightness).
    *   This creates an "Edge Map" where silhouettes of the needle, numbers, and graduations appear white.

2.  **Weighted Voting**:
    *   The algorithm iterates through every edge pixel in the specific annular region (30%-90% radius).
    *   We assume the needle passes through the center. Each edge pixel "votes" for the angle it forms with the center.
    *   **Weighted Vote**: Instead of just counting `+1`, the vote is weighted by the **strength** (magnitude) of the edge. Stronger edges (like the sharp side of a needle) contribute more to the decision than faint edges (like shadows).

3.  **Accumulator**:
    *   Votes are collected in a 360-bin histogram (Accumulator).
    *   The angle with the highest accumulated "mass" of votes is selected as the needle.

## Strengths & Weaknesses
*   **Strengths**: 
    *   Very robust against broken needles or needles partially obscured by text/reflections.
    *   Ignores broad lighting gradients that might confuse intensity-based methods.
*   **Weaknesses**:
    *   Can be confused by strong linear features that *also* point to the center (e.g., long graduation lines).
    *   Slightly more computationally expensive than the Radial Profile.

## Best For
*   Dials with "thin" needles.
*   Situations where lighting is uneven but edges are still visible.
