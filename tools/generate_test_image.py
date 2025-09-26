# generate_test_image.py - UPDATED VERSION
from PIL import Image, ImageDraw, ImageFont
import numpy as np

# Create a 640x480 background with color markers
width, height = 640, 480
image = Image.new('RGB', (width, height), color=(0, 125, 255))  # Blue-green background
draw = ImageDraw.Draw(image)

# Your hardcoded debug crop zones
debug_zones = [
    (80, 233, 116, 307),   # Digit 1 
    (144, 235, 180, 307),  # Digit 2 
    (202, 234, 238, 308),  # Digit 3 
    (265, 233, 304, 306),  # Digit 4 
    (328, 232, 367, 311),  # Digit 5
    (393, 231, 433, 310),  # Digit 6 
    (460, 235, 499, 311),  # Digit 7 
    (520, 235, 559, 342)   # Digit 8 
]

# Correct digits for each zone
digits = "01234567"

# Add color test patterns in corners (for channel order verification)
# Top-left: Pure Red (255,0,0)
draw.rectangle([10, 10, 50, 50], fill=(255, 0, 0))
draw.text((55, 15), "R", fill=(255, 255, 255))

# Top-right: Pure Green (0,255,0)  
draw.rectangle([width-50, 10, width-10, 50], fill=(0, 255, 0))
draw.text((width-85, 15), "G", fill=(255, 255, 255))

# Bottom-left: Pure Blue (0,0,255)
draw.rectangle([10, height-50, 50, height-10], fill=(0, 0, 255))
draw.text((55, height-45), "B", fill=(255, 255, 255))

# Draw each digit with specific background colors for testing
colors = [
    (255, 100, 100),  # Reddish
    (100, 255, 100),  # Greenish  
    (100, 100, 255),  # Blueish
    (200, 200, 100),  # Yellow
    (200, 100, 200),  # Magenta
    (100, 200, 200),  # Cyan
    (150, 150, 150),  # Gray
    (200, 150, 100),  # Orange
]

# Try to load a bold font, fall back to default if not available
try:
    # Try different bold font options
    try:
        font = ImageFont.truetype("arialbd.ttf", 50)  # Arial Bold
    except:
        try:
            font = ImageFont.truetype("Arial Bold.ttf", 50)  # Another common name
        except:
            try:
                # Try to make regular font bold using fontconfig (if available)
                from fontTools.ttLib import TTFont
                font = ImageFont.truetype("arial.ttf", 50)
                # Note: PIL doesn't directly support making fonts bold, 
                # so we'll use a thicker stroke as a fallback
            except:
                font = ImageFont.load_default()
except:
    font = ImageFont.load_default()

for i, (zone, digit, color) in enumerate(zip(debug_zones, digits, colors)):
    x1, y1, x2, y2 = zone
    zone_width = x2 - x1
    zone_height = y2 - y1
    
    # Draw colored rectangle for digit background
    draw.rectangle([x1, y1, x2, y2], fill=color)
    
    # Draw the digit (in contrasting color)
    bbox = draw.textbbox((0, 0), digit, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    text_x = x1 + (zone_width - text_width) // 2
    text_y = y1 + (zone_height - text_height) // 2 - 5
    
    # Use white text with black outline for bold appearance
    draw.text((text_x, text_y), digit, fill=(255, 255, 255), font=font, stroke_width=2, stroke_fill=(0, 0, 0))
    
    # Draw zone border and label
    draw.rectangle([x1, y1, x2, y2], outline=(255, 0, 0), width=2)
    #draw.text((x1 + 5, y1 + 5), str(i+1), fill=(255, 0, 0), font=font)

# Add info text (also in bold)
try:
    info_font = ImageFont.truetype("arialbd.ttf", 20)
except:
    info_font = ImageFont.load_default()

draw.text((10, 60), f"Test Meter: {digits}", fill=(255, 255, 255), font=info_font)
draw.text((10, 110), "Corners: R, G, B markers", fill=(255, 255, 255), font=info_font)

# Save as JPEG
image.save("debug.jpg", "JPEG", quality=95)
print("Generated enhanced debug image with color markers and bold text")