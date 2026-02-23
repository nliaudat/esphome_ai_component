import os
import glob
import json
import argparse
import time
import random
import string
from PIL import Image, ImageOps
import piexif
import piexif.helper

def extract_exif_metadata(image_path):
    try:
        exif_dict = piexif.load(image_path)
        if "Exif" in exif_dict and piexif.ExifIFD.UserComment in exif_dict["Exif"]:
            user_comment = exif_dict["Exif"][piexif.ExifIFD.UserComment]
            if user_comment:
                try:
                    metadata_str = piexif.helper.UserComment.load(user_comment)
                    return json.loads(metadata_str)
                except Exception as e:
                    print(f"Error decoding user comment in {image_path}: {e}")
    except Exception as e:
        print(f"Error reading EXIF from {image_path}: {e}")
        
    return None

def process_images(input_folder, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    
    # app.py saves images into device-specific subdirectories, so we walk the tree
    image_paths = []
    for root, dirs, files in os.walk(input_folder):
        for file in files:
            if file.lower().endswith(('.jpg', '.jpeg')):
                image_paths.append(os.path.join(root, file))
                
    if not image_paths:
        print(f"No JPEG images found in {input_folder}")
        return

    print(f"Found {len(image_paths)} images. Processing...")
    
    processed_count = 0
    extracted_count = 0
    
    for img_path in image_paths:
        try:
            metadata = extract_exif_metadata(img_path)
            if not metadata:
                continue
                
            img = Image.open(img_path)
            # Apply EXIF rotation if present natively in EXIF
            img = ImageOps.exif_transpose(img)
            
            # Apply AI component rotation if embedded in custom JSON metadata
            if 'rot' in metadata:
                rot = float(metadata['rot'])
                if rot != 0:
                    # PIL rotate is counter-clockwise, ESP32 rotation is generally clockwise 
                    # We rotate by -rot (which means clockwise) to align with ESP32 inference logic
                    img = img.rotate(-rot, expand=True)
                    
            digits = metadata.get("digits", [])
            for digit_info in digits:
                val = digit_info.get("val", "unknown")
                conf = digit_info.get("conf", 0.0)
                box = digit_info.get("box", [0, 0, 0, 0])
                
                x, y, w, h = box
                
                if w > 0 and h > 0:
                    crop_box = (x, y, x + w, y + h)
                    zone_img = img.crop(crop_box)
                    
                    rel_dir = os.path.relpath(os.path.dirname(img_path), input_folder)
                    target_out_dir = os.path.join(output_folder, rel_dir)
                    os.makedirs(target_out_dir, exist_ok=True)
                    
                    random_str = ''.join(random.choices(string.ascii_lowercase + string.digits, k=3))
                    
                    # Get original timestamp from the filename if possible, otherwise use file modified time
                    orig_name = os.path.basename(img_path)
                    try:
                        if orig_name.startswith("collect_"):
                            timestamp = int(orig_name.split("_")[1])
                        else:
                            timestamp = int(os.path.getmtime(img_path))
                    except (IndexError, ValueError):
                        timestamp = int(os.path.getmtime(img_path))
                    
                    filename = f"{val}_{conf:.3f}_{timestamp}_{random_str}.jpg"
                    out_path = os.path.join(target_out_dir, filename)
                    
                    zone_img.save(out_path)
                    extracted_count += 1
            
            processed_count += 1
            
        except Exception as e:
            print(f"Error processing {img_path}: {e}")

    print(f"Done! Processed {processed_count} images, extracted {extracted_count} zones.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract zones from images based on EXIF metadata")
    parser.add_argument("--input", "-i", type=str, default="./uploads", help="Input directory containing images")
    parser.add_argument("--output", "-o", type=str, default="extracted", help="Output directory for extracted zones")
    
    args = parser.parse_args()
    process_images(args.input, args.output)
