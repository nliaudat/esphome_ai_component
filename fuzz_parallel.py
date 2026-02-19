
import os
import re
import random
import subprocess
import shutil
import time
import threading
import sys

SOURCE_CONFIG = "config_test.yaml"
NUM_WORKERS = 3
ITERATIONS_PER_WORKER = 10

# Reusing fuzz logic from fuzz_build.py but adapted
BOARD_KEYS = [
    "boards/board_AI-On-The-Edge-Cam_Esp32-S3.yaml",
    "boards/board_freenove_esp32-s3-n8r8.yaml",
    "boards/board_Seeed_Studio_XIAO_ESP32-S3_sense.yaml",
    "boards/board_generic_esp32-s3-n16r8.yaml",
    "boards/board_wrover_kit.yaml",
    "boards/board_m5stack_psram.yaml",
    "boards/board_esp32cam_aithinker.yaml"
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

# Facultative components - can be randomly enabled/disabled
FACULTATIVE_COMPONENTS = [
    'meter_reader_tflite',
    'tflite_micro_helper',
    'esp32_camera_utils',
    'flash_light_controller',
    'value_validator',
    'data_collector',
]

# Component dependencies - if a component is enabled, these must also be enabled
COMPONENT_DEPENDENCIES = {
    'meter_reader_tflite': ['tflite_micro_helper', 'esp32_camera_utils', 'flash_light_controller', 'value_validator'],
    'data_collector': [],  # Optional for meter_reader_tflite
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
        modified = False  # Initialize at start of each iteration
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
        
        # --- 2.5. Replace key substitutions for fuzz testing ---
        if current_component == "substitutions":
            if stripped.startswith("id_prefix:"):
                indent = line[:line.index('id_prefix:')]
                new_lines.append(f"{indent}id_prefix: fuzz\n")
                change_log.append(f"[substitutions] Set id_prefix = fuzz")
                modified = True
            elif stripped.startswith("name:") and '#' not in line.split('name:')[0]:  # Avoid commented lines
                indent = line[:line.index('name:')]
                new_lines.append(f"{indent}name: fuzz-{worker_id}\n")
                change_log.append(f"[substitutions] Set name = fuzz-{worker_id}")
                modified = True
            elif stripped.startswith("friendly_name:"):
                indent = line[:line.index('friendly_name:')]
                new_lines.append(f"{indent}friendly_name: \"Fuzz {worker_id}\"\n")
                change_log.append(f"[substitutions] Set friendly_name = Fuzz {worker_id}")
                modified = True
        
        # --- 3. Handle Component Keys ---
        if not modified and current_component:
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
        
    # --- 4. Randomize Facultative Components ---
    # Randomly enable/disable components while respecting dependencies
    enabled_components = set()
    
    # Randomly decide which components to enable
    for component in FACULTATIVE_COMPONENTS:
        if random.choice([True, False]):  # 50% chance
            enabled_components.add(component)
            # Add dependencies
            if component in COMPONENT_DEPENDENCIES:
                for dep in COMPONENT_DEPENDENCIES[component]:
                    enabled_components.add(dep)
    
    # Ensure at least meter_reader_tflite is enabled with all dependencies
    # (config.yaml is designed for meter_reader_tflite)
    if 'meter_reader_tflite' not in enabled_components:
        enabled_components.add('meter_reader_tflite')
        for dep in COMPONENT_DEPENDENCIES['meter_reader_tflite']:
            enabled_components.add(dep)
    
    # Modify the external_components section
    modified_lines = []
    in_external_components = False
    in_components_list = False
    
    for line in new_lines:
        stripped = line.strip()
        
        if stripped.startswith('external_components:'):
            in_external_components = True
            modified_lines.append(line)
            continue
        
        if in_external_components and 'components:' in line:
            in_components_list = True
            # Replace with randomized component list
            indent = line[:line.index('components:')]
            components_str = ', '.join(sorted(enabled_components))
            modified_lines.append(f"{indent}components: [{components_str}]\n")
            change_log.append(f"[Components] Enabled: {components_str}")
            continue
        
        # Skip original components list
        if in_components_list and ']' in line:
            in_components_list = False
            continue
        
        if in_components_list:
            continue
        
        # Exit external_components section when we hit a new top-level key
        if in_external_components and not line.startswith(' ') and stripped and not stripped.startswith('#'):
            in_external_components = False
        
        # Copy all other lines
        modified_lines.append(line)
    
    # Append safe wifi config
    modified_lines.append("""
wifi:
  networks:
    - ssid: "fuzz_test_ap"
      password: "password12345678"
  ap:
    ssid: "fallback_fuzz"
    password: "password12345678"
""")
    return modified_lines, change_log


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
