
import os
import re
import random
import subprocess
import shutil
import time

SOURCE_CONFIG = "config.yaml"
TEST_CONFIG = "test.yaml"
ITERATIONS = 4 

# Parameters to fuzz (Component -> [Keys])
# We will use simple regex replacement looking for "key: value" under specific indentation blocks if possible,
# or just globally if keys are unique enough.
# Given the structure, some keys like 'debug' appear in multiple places.
# We need to be careful to target specific blocks.

# Structure: Component Name in YAML -> List of keys to toggle
# Board keys for mutual exclusion 
# Structure: Config Filenames for board packages
# These local file names must match the include lines in config.yaml
BOARD_KEYS = [
    "board_AI-On-The-Edge-Cam_Esp32-S3.yaml",
    "board_freenove_esp32-s3-n8r8.yaml",
    "board_Seeed_Studio_XIAO_ESP32-S3_sense.yaml",
    "board_generic_esp32-s3-n16r8.yaml"
]

TARGETS = {
    # "substitutions": BOARD_KEYS, # Removed, we toggle includes now
    "tflite_micro_helper": ["debug"],
    "esp32_camera_utils": ["debug", "enable_rotation", "debug_memory"], 
    "flash_light_controller": ["debug"],
    "meter_reader_tflite": ["debug", "debug_memory", "generate_preview", "enable_rotation", "allow_negative_rates", "debug_image", "debug_image_out_serial"]
}

def read_config():
    with open(SOURCE_CONFIG, "r") as f:
        return f.readlines()

def randomize_config(lines):
    new_lines = []
    current_component = None
    
    # Pre-select a board for this iteration
    selected_board = random.choice(BOARD_KEYS)
    print(f"  [Board Selection] Selected: {selected_board}")
    
    for line in lines:
        original_line = line
        stripped = line.strip()
        
        # --- 1. Handle Board Selection (Packages Includes) ---
        is_board_line = False
        for board_file in BOARD_KEYS:
            if board_file in line:
                is_board_line = True
                if board_file == selected_board:
                    # Enable
                    new_lines.append(f"  - !include {board_file}\n")
                else:
                    # Disable
                    new_lines.append(f"  # - !include {board_file}\n")
                break 
        
        if is_board_line:
            continue

        # --- 2. Track Current Component ---
        if stripped.endswith(":") and not stripped.startswith("#") and not line.startswith(" "):
            comp = stripped[:-1]
            if comp in TARGETS:
                current_component = comp
            else:
                current_component = None
        
        # --- 3. Handle Component Keys ---
        modified = False
        if current_component:
            for key in TARGETS[current_component]:
                pattern = f"^(\\s+){re.escape(key)}: (.*)(#.*)?$"
                match = re.match(pattern, line)
                if match:
                    indent = match.group(1)
                    comment = match.group(3) if match.group(3) else ""
                    
                    new_bool = "true" if random.choice([True, False]) else "false"
                         
                    new_lines.append(f"{indent}{key}: {new_bool}{comment}\n")
                    print(f"  [{current_component}] {key}: {new_bool}")
                    modified = True
                    break
        
        if not modified:
            new_lines.append(original_line)
        
    return new_lines

def run_fuzz():
    print(f"Starting Fuzz Test: {ITERATIONS} iterations")
    print(f"Source: {SOURCE_CONFIG}")
    print(f"Target: {TEST_CONFIG}")
    
    lines = read_config()
    
    # First, simple Copy for baseline (optional, but good to ensure write works)
    # shutil.copyfile(SOURCE_CONFIG, TEST_CONFIG)
    
    results = []

    for i in range(1, ITERATIONS + 1):
        print(f"\n--- Iteration {i}/{ITERATIONS} ---")
        
        # Randomize
        new_lines = randomize_config(lines)
        with open(TEST_CONFIG, "w") as f:
            f.writelines(new_lines)
            
        print("Generated test.yaml with random parameters.")
        
        # Compile
        start_t = time.time()
        # esphome compile test.yaml
        cmd = ["esphome", "compile", TEST_CONFIG]
        
       
        try:
            print("Compiling...")
            # Stream output directly to console so user sees everything
            subprocess.run(cmd, check=True)
            duration = time.time() - start_t
            print(f"PASS ({duration:.1f}s)")
            results.append((i, "PASS", duration))
        except subprocess.CalledProcessError:
            duration = time.time() - start_t
            print(f"FAIL ({duration:.1f}s)")
            # Error output is already printed to console by the subprocess
            results.append((i, "FAIL", duration))

    print("\n--- Summary ---")
    for r in results:
        print(f"Iter {r[0]}: {r[1]} ({r[2]:.1f}s)")

if __name__ == "__main__":
    try:
        run_fuzz()
    except KeyboardInterrupt:
        print("\nAborted.")
