
import os
import re
import random
import subprocess
import shutil
import time
import threading
import sys

SOURCE_CONFIG = "config.yaml"
NUM_WORKERS = 3
ITERATIONS_PER_WORKER = 10

# Reusing fuzz logic from fuzz_build.py but adapted
BOARD_KEYS = [
    "board_AI-On-The-Edge-Cam_Esp32-S3.yaml",
    "board_freenove_esp32-s3-n8r8.yaml",
    "board_Seeed_Studio_XIAO_ESP32-S3_sense.yaml",
    "board_generic_esp32-s3-n16r8.yaml",
    "board_wrover_kit.yaml",
    "board_m5stack_psram.yaml",
    "board_esp32cam_aithinker.yaml"
]

TARGETS = {
    "tflite_micro_helper": ["debug"],
    "esp32_camera_utils": [
        "debug", "debug_memory", 
        "enable_rotation", "rotation", 
        "enable_scaler", "enable_cropper", "enable_drawing"
    ], 
    "flash_light_controller": ["debug"],
    "meter_reader_tflite": ["debug", "debug_memory", "generate_preview", "enable_rotation", "allow_negative_rates", "debug_image", "debug_image_out_serial"],
    "substitutions": ["camera_pixel_format", "name"]
}

PARAM_CHOICES = {
    "camera_pixel_format": ["JPEG", "RGB565", "RGB888", "YUV422", "YUV420", "RAW", "GRAYSCALE"],
    "rotation": ["0", "90", "180", "270", "15", "45.5"]
}

def read_config():
    with open(SOURCE_CONFIG, "r") as f:
        return f.readlines()

def randomize_config(lines, worker_id):
    new_lines = []
    current_component = None
    change_log = []
    
    # Pre-select a board for this iteration
    selected_board = random.choice(BOARD_KEYS)
    change_log.append(f"Selected Board: {selected_board}")
    
    processed_keys = set()
    
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
        # Allow detecting component even if commented out slightly, but rely on standard format
        if stripped.endswith(":") and not line.startswith(" "):
            # Check if it's a known component, even if commented
            # E.g. "  # esp32_camera_utils:" -> key is esp32_camera_utils
            clean_comp = stripped.replace("#", "").strip()[:-1]
            if clean_comp in TARGETS:
                current_component = clean_comp
                processed_keys = set() # Reset for new component
            else:
                current_component = None
        
        # --- 3. Handle Component Keys ---
        modified = False
        if current_component:
            for key in TARGETS[current_component]:
                # Regex to match "  key: value" or "  # key: value"
                pattern = f"^(\\s*)(#\\s*)?{re.escape(key)}: (.*)(#.*)?$"
                match = re.match(pattern, line)
                if match:
                    if key in processed_keys:
                        # Already handled this key for this component, skip duplicate source lines
                        modified = True # Mark as modified so we don't append original
                        break
                        
                    indent = match.group(1)
                    
                    if key == "name" and current_component == "substitutions":
                        # CRITICAL: Change name to ensure unique build path
                        new_val = f"s3cam-tflite-fuzz-{worker_id}"
                    elif key in PARAM_CHOICES:
                        new_val = random.choice(PARAM_CHOICES[key])
                    else:
                        new_val = "true" if random.choice([True, False]) else "false"
                    
                    # Construct new line
                    new_line = f"{indent}{key}: {new_val}\n"
                    new_lines.append(new_line)
                    modified = True
                    change_log.append(f"[{current_component}] Set {key} = {new_val}")
                    processed_keys.add(key)
                    break
        
        if not modified:
            new_lines.append(original_line)
        
    # Append safe wifi config
    new_lines.append("""
wifi:
  networks:
    - ssid: "fuzz_test_ap"
      password: "password12345678"
  ap:
    ssid: "fallback_fuzz"
    password: "password12345678"
""")
    return new_lines, change_log

def worker_task(worker_id):
    os.makedirs("fuzz_logs", exist_ok=True)
    target_file = f"fuzz_test_{worker_id}.yaml"
    
    lines = read_config()
    
    for i in range(1, ITERATIONS_PER_WORKER + 1):
        log_file = f"fuzz_logs/worker_{worker_id}_iter_{i}.log"
        new_lines, changes = randomize_config(lines, worker_id)
        
        with open(target_file, "w") as f:
            f.writelines(new_lines)
            
        print(f"[Worker {worker_id}] Iter {i}: Compiling...")
        
        start_t = time.time()
        cmd = ["esphome", "compile", target_file]
        
        with open(log_file, "w") as log:
            # Write config changes header
            log.write("="*40 + "\n")
            log.write(f"FUZZ CONFIGURATION (Worker {worker_id}, Iter {i})\n")
            log.write("="*40 + "\n")
            for c in changes:
                log.write(f"{c}\n")
            log.write("="*40 + "\n\n")
            log.flush()
            
            try:
                subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
                duration = time.time() - start_t
                print(f"[Worker {worker_id}] Iter {i}: PASS ({duration:.1f}s)")
            except subprocess.CalledProcessError:
                duration = time.time() - start_t
                print(f"[Worker {worker_id}] Iter {i}: FAIL ({duration:.1f}s) - See {log_file}")

if __name__ == "__main__":
    print(f"Starting Parallel Fuzz: {NUM_WORKERS} workers, {ITERATIONS_PER_WORKER} iters each.")
    threads = []
    for i in range(NUM_WORKERS):
        t = threading.Thread(target=worker_task, args=(i,))
        t.start()
        threads.append(t)
        
    for t in threads:
        t.join()
        
    print("All workers completed.")
