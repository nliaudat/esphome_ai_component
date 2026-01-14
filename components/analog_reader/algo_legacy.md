# Legacy Algorithm (Raw Intensity + Connectivity)

## Overview
The **Legacy** algorithm is the original detection method, preserved and enhanced for reliability. Unlike the modern algorithms that rely on heavy preprocessing (TopHat filters), "Legacy" works directly on the **Raw Image** data.

## How It Works

1.  **Raw Analysis**:
    *   It analyzes the original grayscale pixels without running expensive filters like CLAHE or Top-Hat.
    *   It adapts logic based on `needle_type`: for a **Dark Needle**, it looks for low pixel values (darkness); for a **Light Needle**, it looks for high values (brightness).

2.  **Center Connectivity (The "Anti-Flip" Logic)**:
    *   The algorithm casts rays from the center outward (0-360°).
    *   It strictly enforces **Connectivity**: It traces pixels starting from the center and stops immediately if it encounters a "Gap" (a sequence of non-needle pixels).
    *   **Why this matters**: The **Head** of the needle is long, while the **Tail** is short. By summing the intensity/darkness only along the *connected* segment:
        *   The **Head** accumulates a large score (e.g., 50 pixels long).
        *   The **Tail** accumulates a small score (e.g., 10 pixels long) before the gap breaks the summation.
    *   This logic inherently resolves the 180° ambiguity that simpler algorithms suffer from.

3.  **Scoring**:
    *   The final score for each angle is simply the total accumulated intensity/darkness of the connected ray.
    *   The angle with the highest "mass" of connected needle pixels wins.

## Best For
*   **Simple, clean dials** where the needle is clearly visible.
*   **Debugging**: Comparing "Raw" vision vs. "Preprocessed" vision.
*   **Fallback**: In rare cases where preprocessing artifacts (glare/reflections) confuse the modern `radial_profile`, this raw approach often ignores them.

## Resource Usage (IoT)
*   **RAM**: **Minimal**. Works directly on the existing FrameBuffer. No extra allocation required.
*   **CPU**: **Low**. Very fast execution (< 5ms). No costly filters (Sobel, Top-Hat) are run.
*   **Energy**: **Best**. Most power-efficient option for battery devices if you have clean lighting.
