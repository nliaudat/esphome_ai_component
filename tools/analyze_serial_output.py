# analyze_serial_output.py
import re
import numpy as np
from PIL import Image

def parse_zone_analysis(log_text):
    """Parse the detailed zone analysis from serial logs with the actual format"""
    zones = {}
    current_zone = None
    current_pixel = None
    zone_coordinates = {}  # Store zone coordinates by zone number
    zone_counter = {}  # Counter for zones with the same name
    
    for line in log_text.split('\n'):
        # Check for zone processing header
        zone_process_match = re.search(r'Processing zone (\d+): \[(\d+),(\d+),(\d+),(\d+)\]', line)
        if zone_process_match:
            zone_num, x1, y1, x2, y2 = zone_process_match.groups()
            zone_coordinates[zone_num] = {
                'x1': int(x1),
                'y1': int(y1),
                'x2': int(x2),
                'y2': int(y2),
                'width': int(x2) - int(x1),
                'height': int(y2) - int(y1)
            }
            continue
        
        # Check for zone header
        zone_match = re.search(r'ZONE_ANALYSIS:([^:]+):(\d+)x(\d+)x(\d+):normalized=(\w+)', line)
        if zone_match:
            name, width, height, channels, normalized = zone_match.groups()
            
            # Handle duplicate zone names by adding a counter
            if name in zone_counter:
                zone_counter[name] += 1
                unique_name = f"{name}_{zone_counter[name]}"
            else:
                zone_counter[name] = 1
                unique_name = name
            
            current_zone = {
                'name': name,
                'unique_name': unique_name,
                'dimensions': f"{width}x{height}x{channels}",
                'width': int(width),
                'height': int(height),
                'channels': int(channels),
                'normalized': normalized.lower() == 'true',
                'pixels': [],
                'stats': None,
                'coordinates': None
            }
            zones[unique_name] = current_zone
            current_pixel = None
            continue
        
        # Check for pixel header
        pixel_match = re.search(r'Pixel\[(\d+),(\d+)\]:', line)
        if pixel_match and current_zone:
            x, y = int(pixel_match.group(1)), int(pixel_match.group(2))
            current_pixel = {
                'x': x, 
                'y': y, 
                'channels': {}
            }
            current_zone['pixels'].append(current_pixel)
            continue
        
        # Check for channel data
        channel_match = re.search(r'Channel (\d+): ([\d.]+)', line)
        if channel_match and current_zone and current_pixel is not None:
            channel = int(channel_match.group(1))
            value = float(channel_match.group(2))
            current_pixel['channels'][channel] = value
            continue
        
        # Check for stats line (make sure it's for the current zone)
        stats_match = re.search(r'Stats: min=([\d.]+), max=([\d.]+), mean=([\d.]+)', line)
        if stats_match and current_zone:
            # Only add stats if they appear right after pixel data for this zone
            if current_zone['pixels']:  # Only if we have pixels for this zone
                current_zone['stats'] = {
                    'min': float(stats_match.group(1)),
                    'max': float(stats_match.group(2)),
                    'mean': float(stats_match.group(3))
                }
            continue
        
        # Reset current zone if we encounter a line that indicates end of zone analysis
        # Match lines that indicate the end of zone processing
        if current_zone and (
            'Zone processed successfully' in line or
            'process_zone duration:' in line or
            'JPEG zone processed successfully:' in line or
            re.search(r'Processing zone \d+:', line) or  # New zone starting
            re.search(r'ZONE_ANALYSIS:', line)  # New zone analysis starting
        ):
            current_zone = None
            current_pixel = None
    
    # Try to match zone coordinates with zone names
    for zone_name, zone_data in zones.items():
        # Try to extract zone number from name (e.g., "zone1" -> "1")
        zone_num_match = re.search(r'zone(\d+)', zone_data['name'].lower())
        if zone_num_match:
            zone_num = zone_num_match.group(1)
            if zone_num in zone_coordinates:
                zone_data['coordinates'] = zone_coordinates[zone_num]
        # Also check for other naming patterns
        elif 'final' in zone_data['name'].lower() and zone_coordinates:
            # If there's only one zone coordinate, assign it to the final output
            if len(zone_coordinates) == 1:
                zone_data['coordinates'] = next(iter(zone_coordinates.values()))
    
    return zones

def print_analysis(zones):
    """Print formatted analysis of all zones"""
    print(f"Found {len(zones)} zones:")
    
    for zone_unique_name, zone_data in zones.items():
        print(f"\n=== {zone_data['name']} ({zone_data['dimensions']}, normalized={zone_data['normalized']}) ===")
        
        if zone_data['coordinates']:
            coords = zone_data['coordinates']
            print(f"  Coordinates: [{coords['x1']},{coords['y1']},{coords['x2']},{coords['y2']}] "
                  f"(W:{coords['width']}, H:{coords['height']})")
        
        if zone_data['stats']:
            print(f"  Reported Stats: min={zone_data['stats']['min']:.6f}, max={zone_data['stats']['max']:.6f}, mean={zone_data['stats']['mean']:.6f}")
        else:
            print("  No reported stats available")
        
        print(f"  Total pixels: {len(zone_data['pixels'])}")
        
        # Show first few pixels as sample
        sample_pixels = min(5, len(zone_data['pixels']))
        for i in range(sample_pixels):
            pixel = zone_data['pixels'][i]
            print(f"  Pixel[{pixel['x']},{pixel['y']}]: {pixel['channels']}")
        
        if len(zone_data['pixels']) > sample_pixels:
            print(f"  ... and {len(zone_data['pixels']) - sample_pixels} more pixels")

def reconstruct_image(zones):
    """Reconstruct images from zone data, handling both normalized and raw values"""
    for zone_unique_name, zone_data in zones.items():
        if zone_data['pixels']:
            # Create empty image array
            img_array = np.zeros((zone_data['height'], zone_data['width'], zone_data['channels']), dtype=np.uint8)
            
            # Fill with pixel data
            for pixel in zone_data['pixels']:
                x, y = pixel['x'], pixel['y']
                if x < zone_data['width'] and y < zone_data['height']:
                    for channel in range(zone_data['channels']):
                        if channel in pixel['channels']:
                            value = pixel['channels'][channel]
                            # Convert normalized float to uint8 if needed
                            if zone_data['normalized']:
                                img_array[y, x, channel] = int(round(value * 255.0))
                            else:
                                img_array[y, x, channel] = int(round(value))
            
            # Create and save image
            if zone_data['channels'] == 1:
                img = Image.fromarray(img_array[:, :, 0], 'L')
            elif zone_data['channels'] == 3:
                img = Image.fromarray(img_array, 'RGB')
            else:
                print(f"Warning: Unsupported number of channels ({zone_data['channels']}) for {zone_unique_name}")
                continue
            
            # Use unique name for filename to avoid overwriting
            filename = f"{zone_unique_name}.png"
            img.save(filename)
            print(f"Saved reconstructed image: {filename}")

def calculate_zone_stats(zones):
    """Calculate actual statistics from the parsed pixel data"""
    for zone_unique_name, zone_data in zones.items():
        if zone_data['pixels']:
            all_values = []
            for pixel in zone_data['pixels']:
                for value in pixel['channels'].values():
                    all_values.append(value)
            
            if all_values:
                zone_data['calculated_stats'] = {
                    'min': min(all_values),
                    'max': max(all_values),
                    'mean': sum(all_values) / len(all_values),
                    'count': len(all_values)
                }

# Read and analyze serial log
with open('serial_log.txt', 'r') as f:
    log_text = f.read()

zones = parse_zone_analysis(log_text)

# Calculate stats from actual pixel data
calculate_zone_stats(zones)

# Print analysis
print_analysis(zones)

# Print calculated stats for comparison
print("\n=== Calculated Statistics from Pixel Data ===")
for zone_unique_name, zone_data in zones.items():
    if 'calculated_stats' in zone_data:
        stats = zone_data['calculated_stats']
        print(f"{zone_unique_name}: min={stats['min']:.1f}, max={stats['max']:.1f}, mean={stats['mean']:.1f}, pixels={stats['count']}")

# Reconstruct images
reconstruct_image(zones)