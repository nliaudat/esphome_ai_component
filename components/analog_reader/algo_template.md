# Template Matching Algorithm

## Overview
Template Matching is a brute-force approach that is rigorous but computationally intensive. Instead of analyzing features, it virtually "rotates" the image to find the best alignment.

## How It Works

1.  **Concept**:
    *   Imagine checking every single angle (0 to 360) and asking: "If the needle were here, would the pixels match?"

2.  **Execution steps**:
    *   The algorithm performs a **Coarse Search** first (checking every 10 degrees) to find the general vicinity of the needle.
    *   It then does a **Fine Search** (every 1 degree) around the best candidate found in the coarse step.
    *   For each check, it sums the pixel intensities along the predicted line.

3.  **Difference from Radial Profile**:
    *   While Radial Profile builds a map of *all* angles at once, Template Matching behaves more like a "validation" pass.
    *   It focuses purely on intensity summation and does not consider edge gradients or other features.

## Best For
*   **Verification**: Good for double-checking results from other algorithms.
*   **Simple, Low-Noise Dials**: Works best when the needle is the dominant feature.
*   **Debugging**: Useful to understand the raw "signal" strength at a specific angle.

## Drawbacks
*   Slower than other methods due to the nested search loops.
*   Prone to getting stuck in local maxima (e.g., finding a shadow instead of the needle) if the coarse step is too large relative to the needle width.
