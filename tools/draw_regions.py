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

  When --normalize is used:
    - All regions will have the same size (based on the first region drawn)
    - Regions are aligned to the top boundary
    - Size is adjusted to multiples of 8

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

def normalize_region(x1, y1, x2, y2, reference_size=None):
    """
    Normalize a region to ensure (x1, y1) is the top-left corner and (x2, y2) is the bottom-right corner.
    Also adjust coordinates to be multiples of 8.
    
    Args:
        x1, y1, x2, y2: Coordinates of the region.
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
    global regions, drawing, ix, iy, img, normalize_mode

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start drawing
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        # Update the rectangle while dragging
        if drawing:
            # Create a copy of the image to draw on
            display_image = img.copy()
            
            # Determine reference size for normalization
            reference_size = None
            if normalize_mode and regions:
                # Use the size of the first region as reference
                first_region = regions[0]
                ref_width = first_region[2] - first_region[0]
                ref_height = first_region[3] - first_region[1]
                reference_size = (ref_width, ref_height)
            
            # Normalize the coordinates
            x1, y1, x2, y2 = normalize_region(ix, iy, x, y, reference_size)
            
            # Draw the rectangle
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Display dimensions information
            width = x2 - x1
            height = y2 - y1
            cv2.putText(display_image, f"Size: {width}x{height}", (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Show normalization mode indicator
            if normalize_mode:
                if regions:
                    cv2.putText(display_image, f"NORMALIZED (Ref: {width}x{height})", 
                               (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(display_image, "NORMALIZED (Draw first region)", 
                               (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, (0, 255, 255), 2)
            
            # Show the updated image
            cv2.imshow("Draw Regions", display_image)

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
        
        # Normalize the coordinates
        x1, y1, x2, y2 = normalize_region(ix, iy, x, y, reference_size)
        
        # Get final dimensions
        width = x2 - x1
        height = y2 - y1
        
        # Save the region (x1, y1, x2, y2)
        regions.append([x1, y1, x2, y2])
        
        # Draw the final rectangle on the original image
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Add dimension text to the final image
        cv2.putText(img, f"{width}x{height}", (x1, y1-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow("Draw Regions", img)
        
        mode_info = " (Normalized)" if normalize_mode and regions else ""
        print(f"Region added: {[x1, y1, x2, y2]} (Size: {width}x{height}){mode_info}")

def show_instructions(image):
    """
    Display instructions on the image.
    
    Args:
        image (numpy.ndarray): The image to display instructions on.
    """
    instructions = [
        "Instructions:",
        "1. Click and drag to draw a rectangle.",
        "2. Regions auto-adjusted to multiples of 8",
        "3. Press 's' to save regions after last draw",
        "4. Press 'q' to quit and save."
    ]
    
    if normalize_mode:
        instructions.insert(2, "NORMALIZE MODE: All regions same size")
    
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
    global img, normalize_mode

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
    img = cv2.imread(args.image_path)
    if img is None:
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
    print(f"Image dimensions: {img.shape[1]}x{img.shape[0]}")
    print("\nInstructions:")
    print("1. Click and drag to draw a rectangle.")
    print("2. Regions will be automatically adjusted to have width and height as multiples of 8.")
    
    if normalize_mode:
        print("3. NORMALIZE MODE: All regions will have the same size (based on first region)")
        print("4. Press 's' to save regions.")
        print("5. Press 'q' to quit.")
    else:
        print("3. Press 's' to save regions.")
        print("4. Press 'q' to quit.")
    
    print("-" * 40)

    output_file = args.output

    while True:
        # Display the image with regions and instructions
        display_image = img.copy()
        show_instructions(display_image)
        cv2.imshow("Draw Regions", display_image)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF
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
                        print(f"All regions normalized to: {ref_width}x{ref_height}")
                else:
                    print("Warning: Some regions have invalid dimensions. Save cancelled.")
            else:
                print("No regions to save.")

    # Clean up
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()