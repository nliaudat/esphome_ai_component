# draw_regions.py
import cv2
import json
import sys
import os
import argparse

# Global variables
regions = []
drawing = False
ix, iy = -1, -1
normalize_mode = False

# Zoom variables
zoom_level = 1.0
original_image = None

def show_help():
    """Display help information."""
    help_text = """
Draw Regions Tool

Usage:
  python draw_regions.py [IMAGE_PATH] [OPTIONS]

Options:
  -h, --help          Show this help message and exit
  -o, --output FILE   Specify output JSON file (default: regions.json)
  -n, --normalize     Normalize regions to common size and top alignment

Description:
  This tool allows you to draw rectangular regions on an image. All regions
  will have their width and height automatically adjusted to be multiples of 8
  for compatibility with various image processing algorithms.

  Interactive controls:
    - Mouse wheel: Zoom in/out (centered on mouse position)
    - Left mouse drag: Draw regions
    - 'r': Reset regions (clear all)
    - 'f': Reset zoom to full size
    - 's': Save regions
    - 'q': Quit

Instructions:
  1. Click and drag to draw a rectangle
  2. Regions are auto-adjusted to multiples of 8 in both dimensions
  3. Press 's' to save regions to JSON file
  4. Press 'q' to quit the application

Examples:
  python draw_regions.py image.jpg
  python draw_regions.py photo.png -o my_regions.json
  python draw_regions.py image.jpg --normalize
  python draw_regions.py --help
"""
    print(help_text)

def get_display_image():
    """Get the zoomed display image."""
    global original_image, zoom_level
    
    if original_image is None:
        return None
    
    if zoom_level == 1.0:
        return original_image.copy()
    
    img_height, img_width = original_image.shape[:2]
    
    # Calculate new dimensions
    new_width = int(img_width * zoom_level)
    new_height = int(img_height * zoom_level)
    
    # Resize the image
    display_image = cv2.resize(original_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    
    return display_image

def screen_to_image_coords(sx, sy):
    """Convert screen coordinates to original image coordinates."""
    global zoom_level
    
    if zoom_level == 1.0:
        return sx, sy
    
    # Apply inverse of zoom transformation
    img_x = int(sx / zoom_level)
    img_y = int(sy / zoom_level)
    
    return img_x, img_y

def normalize_region(x1, y1, x2, y2, reference_size=None):
    """
    Normalize a region to ensure (x1, y1) is the top-left corner and (x2, y2) is the bottom-right corner.
    Also adjust coordinates to be multiples of 8.
    
    Args:
        x1, y1, x2, y2: Coordinates of the region (in image coordinates).
        reference_size: If provided, use this size for normalization (width, height)
        
    Returns:
        tuple: Normalized region (x1, y1, x2, y2) with dimensions as multiples of 8.
    """
    if x1 > x2:
        x1, x2 = x2, x1  # Swap x1 and x2 if x1 > x2
    if y1 > y2:
        y1, y2 = y2, y1  # Swap y1 and y2 if y1 > y2
    
    if reference_size and normalize_mode:
        # Use reference size for normalization
        ref_width, ref_height = reference_size
        # Keep original x1 position, use reference width and height
        x2 = x1 + ref_width
        y2 = y1 + ref_height
    else:
        # Calculate width and height from current selection
        width = x2 - x1
        height = y2 - y1
        
        # Adjust width to be multiple of 8
        adjusted_width = (width // 8) * 8
        if adjusted_width == 0 and width > 0:
            adjusted_width = 8  # Minimum width of 8 pixels
        
        # Adjust height to be multiple of 8
        adjusted_height = (height // 8) * 8
        if adjusted_height == 0 and height > 0:
            adjusted_height = 8  # Minimum height of 8 pixels
        
        # Calculate new coordinates
        x2 = x1 + adjusted_width
        y2 = y1 + adjusted_height
    
    return (x1, y1, x2, y2)

def draw_rectangle(event, x, y, flags, param):
    """Callback function to draw rectangles on the image."""
    global regions, drawing, ix, iy, normalize_mode, zoom_level

    # Convert screen coordinates to image coordinates
    img_x, img_y = screen_to_image_coords(x, y)

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start drawing
        drawing = True
        ix, iy = img_x, img_y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            # Update the rectangle while dragging
            display_image = get_display_image()
            if display_image is None:
                return
            
            # Create a copy of the display image to draw on
            temp_image = display_image.copy()
            
            # Determine reference size for normalization
            reference_size = None
            if normalize_mode and regions:
                # Use the size of the first region as reference
                first_region = regions[0]
                ref_width = first_region[2] - first_region[0]
                ref_height = first_region[3] - first_region[1]
                reference_size = (ref_width, ref_height)
            
            # Normalize the coordinates (in image coordinates)
            x1_img, y1_img, x2_img, y2_img = normalize_region(ix, iy, img_x, img_y, reference_size)
            
            # Convert back to screen coordinates for display
            if zoom_level != 1.0:
                x1_scr = int(x1_img * zoom_level)
                y1_scr = int(y1_img * zoom_level)
                x2_scr = int(x2_img * zoom_level)
                y2_scr = int(y2_img * zoom_level)
            else:
                x1_scr, y1_scr, x2_scr, y2_scr = x1_img, y1_img, x2_img, y2_img
            
            # Draw the rectangle
            cv2.rectangle(temp_image, (x1_scr, y1_scr), (x2_scr, y2_scr), (0, 255, 0), 2)
            
            # Display dimensions information
            width = x2_img - x1_img
            height = y2_img - y1_img
            cv2.putText(temp_image, f"Size: {width}x{height}", (x1_scr, max(y1_scr-10, 10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Show the updated image
            show_instructions(temp_image)
            cv2.imshow("Draw Regions", temp_image)

    elif event == cv2.EVENT_LBUTTONUP:
        # Finish drawing
        drawing = False
        
        # Determine reference size for normalization
        reference_size = None
        if normalize_mode and regions:
            # Use the size of the first region as reference
            first_region = regions[0]
            ref_width = first_region[2] - first_region[0]
            ref_height = first_region[3] - first_region[1]
            reference_size = (ref_width, ref_height)
        
        # Normalize the coordinates (in image coordinates)
        x1_img, y1_img, x2_img, y2_img = normalize_region(ix, iy, img_x, img_y, reference_size)
        
        # Get final dimensions
        width = x2_img - x1_img
        height = y2_img - y1_img
        
        # Save the region in original image coordinates
        regions.append([x1_img, y1_img, x2_img, y2_img])
        
        # Redraw the main image to show all regions
        redraw_image()
        
        mode_info = " (Normalized)" if normalize_mode and len(regions) > 1 else ""
        print(f"Region added: {[x1_img, y1_img, x2_img, y2_img]} (Size: {width}x{height}){mode_info}")
        
    elif event == cv2.EVENT_MOUSEWHEEL:
        # Zoom with mouse wheel
        zoom_speed = 1.2
        if flags > 0:  # Zoom in
            zoom_level *= zoom_speed
            zoom_level = min(zoom_level, 10.0)  # Max 10x zoom
        else:  # Zoom out
            zoom_level /= zoom_speed
            zoom_level = max(zoom_level, 0.1)   # Min 0.1x zoom
        
        redraw_image()

def redraw_image():
    """Redraw the main image with all regions."""
    global original_image, regions, zoom_level
    
    if original_image is None:
        return
    
    # Get display image
    display_image = get_display_image()
    if display_image is None:
        return
    
    temp_image = display_image.copy()
    
    # Draw all regions
    for region in regions:
        x1_img, y1_img, x2_img, y2_img = region
        
        # Convert to screen coordinates
        if zoom_level != 1.0:
            x1_scr = int(x1_img * zoom_level)
            y1_scr = int(y1_img * zoom_level)
            x2_scr = int(x2_img * zoom_level)
            y2_scr = int(y2_img * zoom_level)
        else:
            x1_scr, y1_scr, x2_scr, y2_scr = x1_img, y1_img, x2_img, y2_img
        
        # Only draw if region is visible in viewport
        if (x1_scr < display_image.shape[1] and x2_scr > 0 and 
            y1_scr < display_image.shape[0] and y2_scr > 0):
            
            # Draw rectangle
            cv2.rectangle(temp_image, (x1_scr, y1_scr), (x2_scr, y2_scr), (0, 255, 0), 2)
            
            # Add dimension text
            width = x2_img - x1_img
            height = y2_img - y1_img
            cv2.putText(temp_image, f"{width}x{height}", (x1_scr, max(y1_scr-10, 10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Show instructions and info
    show_instructions(temp_image)
    
    # Show zoom info
    if zoom_level != 1.0:
        info_text = f"Zoom: {zoom_level:.1f}x | Regions: {len(regions)} | Press 'f' for full size"
        cv2.putText(temp_image, info_text, (10, temp_image.shape[0] - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        info_text = f"Full size | Regions: {len(regions)}"
        cv2.putText(temp_image, info_text, (10, temp_image.shape[0] - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    if normalize_mode:
        cv2.putText(temp_image, "NORMALIZE MODE", (10, temp_image.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    cv2.imshow("Draw Regions", temp_image)

def show_instructions(image):
    """
    Display instructions on the image.
    
    Args:
        image (numpy.ndarray): The image to display instructions on.
    """
    instructions = [
        "Instructions:",
        "Left drag: Draw region",
        "Wheel: Zoom  r: Reset regions",
        "f: Full size  s: Save  q: Quit"
    ]
    
    if normalize_mode:
        instructions.append("NORMALIZE: All regions same size")
    
    y_offset = 30
    for line in instructions:
        cv2.putText(image, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_offset += 20

def validate_regions():
    """
    Validate that all regions have dimensions that are multiples of 8.
    """
    for i, region in enumerate(regions):
        x1, y1, x2, y2 = region
        width = x2 - x1
        height = y2 - y1
        
        if width % 8 != 0 or height % 8 != 0:
            print(f"Warning: Region {i} has invalid dimensions {width}x{height}")
            return False
    
    # Additional validation for normalize mode
    if normalize_mode and len(regions) > 1:
        first_size = (regions[0][2] - regions[0][0], regions[0][3] - regions[0][1])
        for i, region in enumerate(regions[1:], 1):
            current_size = (region[2] - region[0], region[3] - region[1])
            if current_size != first_size:
                print(f"Warning: Region {i} size {current_size} doesn't match reference size {first_size}")
                return False
    
    print("All regions validated: dimensions are multiples of 8" + 
          (" and consistent" if normalize_mode else ""))
    return True

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Draw rectangular regions on an image with dimensions as multiples of 8",
        add_help=False
    )
    parser.add_argument(
        'image_path',
        nargs='?',
        default='sample.jpg',
        help='Path to the input image (default: sample.jpg)'
    )
    parser.add_argument(
        '-o', '--output',
        default='regions.json',
        help='Output JSON file for regions (default: regions.json)'
    )
    parser.add_argument(
        '-n', '--normalize',
        action='store_true',
        help='Normalize all regions to same size (based on first region)'
    )
    parser.add_argument(
        '-h', '--help',
        action='store_true',
        help='Show this help message and exit'
    )
    
    return parser.parse_args()

def main():
    global original_image, normalize_mode, zoom_level, regions

    # Parse command line arguments
    args = parse_arguments()
    
    # Show help if requested
    if args.help:
        show_help()
        return
    
    # Set normalize mode
    normalize_mode = args.normalize
    
    # Check if file exists
    if not os.path.exists(args.image_path):
        print(f"Error: Image file '{args.image_path}' not found.")
        print("\nUsage: python draw_regions.py [IMAGE_PATH] [OPTIONS]")
        print("Use 'python draw_regions.py --help' for more information.")
        return

    # Load the image
    original_image = cv2.imread(args.image_path)
    if original_image is None:
        print(f"Error: Unable to load image from {args.image_path}.")
        return

    # Create a window and set the mouse callback
    cv2.namedWindow("Draw Regions")
    cv2.setMouseCallback("Draw Regions", draw_rectangle)

    # Display helper prompt in the console
    print("=== Draw Regions Tool ===")
    print(f"Input image: {args.image_path}")
    print(f"Output file: {args.output}")
    print(f"Normalize mode: {'ON' if normalize_mode else 'OFF'}")
    print(f"Image dimensions: {original_image.shape[1]}x{original_image.shape[0]} pixels")
    print("\nInteractive Controls:")
    print("  Mouse wheel: Zoom in/out")
    print("  Left mouse drag: Draw regions")
    print("  r: Reset regions (clear all)")
    print("  f: Reset zoom to full size")
    print("  s: Save regions")
    print("  q: Quit")
    print("-" * 40)

    output_file = args.output

    # Initial display
    redraw_image()

    while True:
        # Wait for key press
        key = cv2.waitKey(0) & 0xFF
        
        if key == ord("q"):  # Quit
            break
            
        elif key == ord("s"):  # Save regions
            if regions:
                # Validate regions before saving
                if validate_regions():
                    # Save regions as JSON
                    with open(output_file, "w") as f:
                        json.dump(regions, f)
                    print(f"Regions saved to {output_file}")
                    
                    # Show summary
                    if normalize_mode and len(regions) > 0:
                        first_region = regions[0]
                        ref_width = first_region[2] - first_region[0]
                        ref_height = first_region[3] - first_region[1]
                        print(f"All regions normalized to: {ref_width}x{ref_height} pixels")
                else:
                    print("Warning: Some regions have invalid dimensions. Save cancelled.")
            else:
                print("No regions to save.")
                
        elif key == ord("r"):  # Reset regions (clear all)
            regions.clear()
            redraw_image()
            print("All regions cleared")
            
        elif key == ord("f"):  # Reset zoom to full size
            zoom_level = 1.0
            redraw_image()
            print("Zoom reset to full size")

    # Clean up
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()