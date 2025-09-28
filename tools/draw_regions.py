# draw_regions.py
import cv2
import json

# Global variables
regions = []
drawing = False
ix, iy = -1, -1

def normalize_region(x1, y1, x2, y2):
    """
    Normalize a region to ensure (x1, y1) is the top-left corner and (x2, y2) is the bottom-right corner.
    
    Args:
        x1, y1, x2, y2: Coordinates of the region.
        
    Returns:
        tuple: Normalized region (x1, y1, x2, y2).
    """
    if x1 > x2:
        x1, x2 = x2, x1  # Swap x1 and x2 if x1 > x2
    if y1 > y2:
        y1, y2 = y2, y1  # Swap y1 and y2 if y1 > y2
    return (x1, y1, x2, y2)

def draw_rectangle(event, x, y, flags, param):
    """Callback function to draw rectangles on the image."""
    global regions, drawing, ix, iy, img

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start drawing
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        # Update the rectangle while dragging
        if drawing:
            # Create a copy of the image to draw on
            display_image = img.copy()
            # Normalize the coordinates to ensure the rectangle is drawn correctly
            x1, y1, x2, y2 = normalize_region(ix, iy, x, y)
            # Draw the rectangle
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Show the updated image
            cv2.imshow("Draw Regions", display_image)

    elif event == cv2.EVENT_LBUTTONUP:
        # Finish drawing
        drawing = False
        # Normalize the coordinates to ensure the region is stored correctly
        x1, y1, x2, y2 = normalize_region(ix, iy, x, y)
        # Save the region (x1, y1, x2, y2)
        regions.append([x1, y1, x2, y2])
        # Draw the final rectangle on the original image
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow("Draw Regions", img)
        print(f"Region added: {[x1, y1, x2, y2]}")

def show_instructions(image):
    """
    Display instructions on the image.
    
    Args:
        image (numpy.ndarray): The image to display instructions on.
    """
    instructions = [
        "Instructions:",
        "1. Click and drag to draw a rectangle.",
        "2. Press 's' to save regions after last draw",
        "3. Press 'q' to quit and save."
    ]
    y_offset = 30
    for line in instructions:
        cv2.putText(image, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_offset += 20

def main():
    global img

    # Load the image
    image_path = "sample.jpg"  # Replace with your image path
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Unable to load image from {image_path}.")
        return

    # Create a window and set the mouse callback
    cv2.namedWindow("Draw Regions")
    cv2.setMouseCallback("Draw Regions", draw_rectangle)

    # Display helper prompt in the console
    print("=== Instructions ===")
    print("1. Click and drag to draw a rectangle.")
    print("2. Press 's' to save regions after last draw.")
    print("3. Press 'q' to quit and save.")

    while True:
        # Display the image with regions and instructions
        display_image = img.copy()
        show_instructions(display_image)
        cv2.imshow("Draw Regions", display_image)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # Quit
            if regions:
                # Prompt the user to save before quitting
                save = input("Do you want to save the regions before quitting? (y/n): ").strip().lower()
                if save == "y":
                    # Save regions as JSON
                    with open("regions.json", "w") as f:
                        json.dump(regions, f)
                    print("Regions saved to regions.json")
            break
        elif key == ord("s"):  # Save regions
            if regions:
                # Save regions as JSON
                with open("regions.json", "w") as f:
                    json.dump(regions, f)
                print("Regions saved to regions.json")
            else:
                print("No regions to save.")

    # Clean up
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()