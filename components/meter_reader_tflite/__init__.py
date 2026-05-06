"""Component to use TensorFlow Lite Micro to read a meter."""
import esphome.codegen as cg
import esphome.config_validation as cv
import os
import re
import zlib
from esphome.const import CONF_ID, CONF_MODEL, CONF_ROTATION, CONF_NAME, CONF_DISABLED_BY_DEFAULT, CONF_INTERNAL, CONF_ICON, CONF_FORCE_UPDATE, CONF_ENTITY_CATEGORY
from esphome.core import CORE, HexInt
from esphome.components import esp32, sensor, text_sensor, button
try:
    from esphome.components import value_validator
except ImportError:
    value_validator = None

import esphome.components.esp32_camera as esp32_camera
from esphome.cpp_generator import RawExpression
from esphome.components import globals
try:
    import esphome.components.flash_light_controller as flash_light_controller
except ImportError:
    flash_light_controller = None

try:
    import esphome.components.data_collector as data_collector
except ImportError:
    data_collector = None

CODEOWNERS = ["@nl"]
if CORE.target_platform == "esp32":
    DEPENDENCIES = ['esp32', 'tflite_micro_helper', 'esp32_camera_utils']
    # flash_light_controller and data_collector are optional; not added to DEPENDENCIES here
else:
    # On host, we mock utils and remove esp32 check
    DEPENDENCIES = ['tflite_micro_helper']

AUTO_LOAD = ['sensor']

CONF_CAMERA_ID = 'camera_id'
CONF_TENSOR_ARENA_SIZE = 'tensor_arena_size'
CONF_CONFIDENCE_THRESHOLD = 'confidence_threshold'
CONF_RAW_DATA_ID = 'raw_data_id'
CONF_DEBUG = 'debug'
CONF_DEBUG_IMAGE = 'debug_image'
CONF_DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL = 'debug_image_out_serial'
CONF_DEBUG_MEMORY = 'debug_memory'
CONF_VALIDATOR = 'validator'

# CONF_MODEL_TYPE = 'model_type' 
CONF_PREVIEW = 'preview_camera'
CONF_GENERATE_PREVIEW = 'generate_preview'
CONF_START_FLASH_CALIBRATION_BUTTON = 'start_flash_calibration_button'

CONF_FLASH_LIGHT_CONTROLLER = 'flash_light_controller'
CONF_DATA_COLLECTOR = 'data_collector'
CONF_COLLECT_LOW_CONFIDENCE = 'collect_low_confidence'
CONF_COLLECT_MIN_GLOBAL_CONFIDENCE = 'collect_min_global_confidence'
CONF_COLLECT_MIN_DIGIT_CONFIDENCE = 'collect_min_digit_confidence'

CONF_CROP_ZONES = 'crop_zones_global'

CONF_CAMERA_WINDOW = 'camera_window'

CONF_FRAME_REQUEST_TIMEOUT = 'frame_request_timeout'
CONF_UNLOAD_BUTTON = 'unload_button'
CONF_RELOAD_BUTTON = 'reload_button'

# Dynamic model config overrides (optional - auto-detected from .txt file)
CONF_INPUT_TYPE = 'input_type'
CONF_INPUT_CHANNELS = 'input_channels'
CONF_INPUT_WIDTH = 'input_width'
CONF_INPUT_HEIGHT = 'input_height'
CONF_OUTPUT_PROCESSING = 'output_processing'
CONF_SCALE_FACTOR = 'scale_factor'
CONF_INPUT_ORDER = 'input_order'
CONF_NORMALIZE = 'normalize'
CONF_INVERT = 'invert'

meter_reader_tflite_ns = cg.esphome_ns.namespace('meter_reader_tflite')
MeterReaderTFLite = meter_reader_tflite_ns.class_('MeterReaderTFLite', cg.PollingComponent)


def datasize_to_bytes(value):
    """Parse a data size string with units like KB, MB to bytes."""
    try:
        value = str(value).upper().strip()
        if value.endswith('KB'):
            return int(float(value[:-2]) * 1024)
        if value.endswith('MB'):
            return int(float(value[:-2]) * 1024 * 1024)
        if value.endswith('B'):
            return int(value[:-1])
        return int(value)
    except ValueError as e:
        raise cv.Invalid(f"Invalid data size: {e}") from e


def parse_model_txt_file(model_path):
    """Parse a model .txt file to extract configuration parameters.
    
    Returns a dict with auto-detected config values, or None if file not found.
    """
    txt_path = os.path.splitext(model_path)[0] + '.txt'
    if not os.path.exists(txt_path):
        return None
    
    with open(txt_path, 'r') as f:
        content = f.read()
    
    config = {}
    
    # Parse input type and shape from INPUT/OUTPUT SUMMARY section
    # Example: "Input 0:  [ 1 32 20  3]   <class 'numpy.float32'>"
    input_match = re.search(r'Input\s+0:\s+\[\s*\d+\s+(\d+)\s+(\d+)\s+(\d+)\].*?numpy\.(\w+)', content)
    if input_match:
        config['input_height'] = int(input_match.group(1))
        config['input_width'] = int(input_match.group(2))
        config['input_channels'] = int(input_match.group(3))
        dtype = input_match.group(4)
        config['input_type'] = 'float32' if dtype == 'float32' else 'uint8'
    
    # Parse output shape to determine class count
    # Example: "Output 0: [ 1 10]         <class 'numpy.float32'>"
    output_match = re.search(r'Output\s+0:\s+\[\s*\d+\s+(\d+)\]', content)
    if output_match:
        num_classes = int(output_match.group(1))
        # 10 classes → scale_factor=1.0, 100 classes → scale_factor=10.0
        if num_classes == 10:
            config['scale_factor'] = 1.0
        elif num_classes == 100:
            config['scale_factor'] = 10.0
        else:
            config['scale_factor'] = 1.0
    
    # Parse peak memory from peak analysis (if available)
    # Example: "Peak active memory: 70.96 KB"
    peak_match = re.search(r'Peak active memory:\s+([\d.]+)\s+KB', content)
    if peak_match:
        peak_kb = float(peak_match.group(1))
        # Use 1.5x safety margin, minimum 100KB
        arena_kb = max(100, int(peak_kb * 1.5))
        config['tensor_arena_size'] = arena_kb * 1024
    
    # Parse total operations count for MAX_OPERATORS
    # Example: "Total operations: 37"
    ops_match = re.search(r'Total operations:\s+(\d+)', content)
    if ops_match:
        total_ops = int(ops_match.group(1))
        # Add safety margin of 5 for the resolver
        config['max_operators'] = total_ops + 5
        print(f"  Auto-detected MAX_OPERATORS: {config['max_operators']} (from {total_ops} operations + 5 margin)")
    
    # Detect DELEGATE ops (incompatible with TFLite Micro) - informational only
    if re.search(r'\bDELEGATE\b', content):
        print(f"  Note: Model '{os.path.basename(txt_path)}' contains DELEGATE ops.")
        print(f"    DELEGATE ops are NOT compatible with TFLite Micro.")
        print(f"    Re-export the model with delegates disabled:")
        print(f"      converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]")
    
    return config


def infer_model_config_from_filename(model_filename):
    """Infer model config from filename heuristics when no .txt file exists."""
    config = {}
    name = os.path.splitext(model_filename)[0]
    
    # Detect channels from filename
    if '_GRAY' in name or '_GRAYSCALE' in name:
        config['input_channels'] = 1
        config['input_order'] = 'GRAY'
    elif '_RGB' in name:
        config['input_channels'] = 3
        config['input_order'] = 'RGB'
    elif '_BGR' in name:
        config['input_channels'] = 3
        config['input_order'] = 'BGR'
    else:
        config['input_channels'] = 3
        config['input_order'] = 'RGB'
    
    # Detect class count from filename
    if '_10cls_' in name or name.endswith('_10cls'):
        config['scale_factor'] = 1.0
    elif '_100cls_' in name or name.endswith('_100cls'):
        config['scale_factor'] = 10.0
    else:
        config['scale_factor'] = 1.0
    
    return config


# Use the standard ESPHome sensor configuration pattern
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MeterReaderTFLite),
    cv.Required(CONF_MODEL): cv.file_,
    cv.Optional(CONF_VALIDATOR): cv.use_id(value_validator.ValueValidator) if value_validator else cv.string,
    cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera) if CORE.target_platform == "esp32" else cv.string,
    # cv.Optional(CONF_MODEL_TYPE, default="class100-0180"): cv.string,  # Add model type selection
    cv.Optional(CONF_CONFIDENCE_THRESHOLD, default=0.85): cv.float_range(
        min=0.0, max=1.0
    ),
    # Make tensor_arena_size optional since it's auto-detected from .txt file
    cv.Optional(CONF_TENSOR_ARENA_SIZE): cv.Any(
        cv.All(
            datasize_to_bytes,
            cv.Range(min=50 * 1024, max=1000 * 1024)
        ),
        cv.string,  # Allow string like "110KB" for auto-detection override
    ),
    cv.GenerateID(CONF_RAW_DATA_ID): cv.declare_id(cg.uint8),
    cv.Optional(CONF_DEBUG, default=False): cv.boolean, 
    cv.Optional(CONF_DEBUG_IMAGE, default=False): cv.boolean, 
    cv.Optional(CONF_DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG_MEMORY, default=False): cv.boolean,
    cv.Optional("tensor_arena_size_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("tensor_arena_used_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("process_free_heap_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("process_free_psram_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("pool_job_efficiency_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("pool_result_efficiency_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("arena_efficiency_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("heap_fragmentation_sensor"): cv.use_id(sensor.Sensor),

    cv.Optional(CONF_FLASH_LIGHT_CONTROLLER): cv.use_id(flash_light_controller.FlashLightController) if flash_light_controller else cv.string,
    cv.Optional(CONF_DATA_COLLECTOR): cv.use_id(data_collector.DataCollector) if data_collector else cv.string,
    cv.Optional(CONF_COLLECT_LOW_CONFIDENCE, default=True): cv.boolean,
    cv.Optional(CONF_COLLECT_MIN_GLOBAL_CONFIDENCE, default=0.90): cv.float_range(min=0.0, max=1.0),
    cv.Optional(CONF_COLLECT_MIN_DIGIT_CONFIDENCE, default=0.90): cv.float_range(min=0.0, max=1.0),

    cv.Optional(CONF_CROP_ZONES): cv.use_id(globals.GlobalsComponent),
    # cv.Optional(CONF_CAMERA_WINDOW): cv.Any(
    # cv.Schema({  # Or detailed configuration
    #     cv.Optional('offset_x', default=0): cv.int_,
    #     cv.Optional('offset_y', default=0): cv.int_,
    #     cv.Optional('width'): cv.int_,
    #     cv.Optional('height'): cv.int_,
    #     })
    # ),
    # cv.Optional(CONF_AUTO_CAMERA_WINDOW, default=False): cv.boolean,
    cv.Optional(CONF_FRAME_REQUEST_TIMEOUT, default=15000): cv.int_range(min=1000, max=60000),
    
    # Dynamic model config overrides (optional - auto-detected from .txt file)
    cv.Optional(CONF_INPUT_TYPE): cv.enum({'uint8': 'uint8', 'float32': 'float32'}, lower=True),
    cv.Optional(CONF_INPUT_CHANNELS): cv.int_range(min=1, max=4),
    cv.Optional(CONF_INPUT_WIDTH): cv.int_range(min=8, max=512),
    cv.Optional(CONF_INPUT_HEIGHT): cv.int_range(min=8, max=512),
    cv.Optional(CONF_OUTPUT_PROCESSING): cv.enum({'direct_class': 'direct_class', 'softmax': 'softmax', 'argmax': 'argmax'}, lower=True),
    cv.Optional(CONF_SCALE_FACTOR): cv.float_range(min=0.1, max=100.0),
    cv.Optional(CONF_INPUT_ORDER): cv.enum({'RGB': 'RGB', 'BGR': 'BGR', 'GRAY': 'GRAY'}, upper=True),
    cv.Optional(CONF_NORMALIZE): cv.boolean,
    cv.Optional(CONF_INVERT): cv.boolean,
    
    cv.Optional("value_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("confidence_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("inference_logs"): cv.use_id(text_sensor.TextSensor),
    cv.Optional("main_logs"): cv.use_id(text_sensor.TextSensor),
    cv.Optional(CONF_START_FLASH_CALIBRATION_BUTTON): cv.use_id(button.Button),
    cv.Optional(CONF_GENERATE_PREVIEW, default=False): cv.boolean,
    cv.Optional("show_crop_areas", default=True): cv.boolean,
    cv.Optional("enable_flash_calibration", default=False): cv.boolean,
    cv.Optional(CONF_UNLOAD_BUTTON): cv.use_id(button.Button),
    cv.Optional(CONF_RELOAD_BUTTON): cv.use_id(button.Button),

    cv.Optional("total_inference_time_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("capture_to_publish_time_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("debug_timing", default=False): cv.boolean,
    # cv.Optional(CONF_PREVIEW): camera_component.CAMERA_SCHEMA.extend({
    #     cv.GenerateID(): cv.declare_id(MeterPreviewCamera),
    # }),
}).extend(cv.polling_component_schema('60s'))

async def to_code(config):
    """Code generation for the component."""

    # esp32.add_idf_component(
    #     name="espressif/esp-tflite-micro",
    #     # ref="~1.3.4" #https://github.com/espressif/esp-tflite-micro/issues/120
    #     ref="1.3.4" # fix to 1.3.4 cause 1.3.5 has bug
    # )
    
    # esp32.add_idf_component(
    #     name="espressif/esp-nn",
    #     ref="~1.1.2"
    # )
    
    # esp32.add_idf_component(
    #     name="espressif/esp_new_jpeg",
    #     ref="1.0.0"
    # )
        
    # cg.add_build_flag("-DTF_LITE_STATIC_MEMORY")
    # cg.add_build_flag("-DTF_LITE_DISABLE_X86_NEON")
    # cg.add_build_flag("-DESP_NN")
    # cg.add_build_flag("-DUSE_ESP32_CAMERA_CONV")
    # cg.add_build_flag("-DOPTIMIZED_KERNEL=esp_nn")

    var = cg.new_Pvariable(config[CONF_ID])
    cg.add_global(cg.RawStatement('#include "esphome/components/meter_reader_tflite/meter_reader_tflite.h"'))
    cg.add_global(cg.RawStatement('using namespace esphome::meter_reader_tflite;'))
    await cg.register_component(var, config)
    
    cg.add_define("USE_METER_READER_TFLITE")
    
    # Register validator
    if CONF_VALIDATOR in config:
        cg.add_define("USE_VALUE_VALIDATOR")
        v = await cg.get_variable(config[CONF_VALIDATOR])
        cg.add(var.set_validator(v))

    if CORE.target_platform == "esp32":
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))
    else:
        cg.add_define("USE_HOST_MOCK_CAMERA")
        cg.add_define("USE_HOST")
        # On host, we don't set a real camera object.
        pass
    
    model_path = CORE.relative_config_path(config[CONF_MODEL])
    model_filename = os.path.basename(str(model_path).replace("\\", "/"))
    model_type = os.path.splitext(model_filename)[0]  # Remove .tflite extension
       
    # Set model type from extracted filename
    cg.add(var.set_model_config(model_type))
    
    # Read the model file as binary data
    with open(model_path, "rb") as f:
        model_data = f.read()
        
    # Compute CRC32
    crc32_val = zlib.crc32(model_data) & 0xFFFFFFFF
    cg.add_define("MODEL_CRC32", HexInt(crc32_val)) 
    
    # Create a progmem array for the model data
    rhs = [HexInt(x) for x in model_data]
    prog_arr = cg.progmem_array(config[CONF_RAW_DATA_ID], rhs)
    
    cg.add(var.set_model(prog_arr, len(model_data)))
    cg.add(var.set_confidence_threshold(config[CONF_CONFIDENCE_THRESHOLD]))
    
    # ============================================================
    # Dynamic Model Configuration (auto-detect from .txt, override from YAML)
    # ============================================================
    # Priority: YAML overrides > .txt auto-detection > filename heuristics > defaults
    
    # Step 1: Try to parse .txt file for auto-detected config
    auto_config = parse_model_txt_file(model_path)
    if auto_config:
        print(f"  Auto-detected model config from '{model_filename}.txt':")
        for k, v in auto_config.items():
            print(f"    {k}: {v}")
    else:
        # Step 2: Fallback to filename heuristics
        auto_config = infer_model_config_from_filename(model_filename)
        print(f"  No .txt file found for '{model_filename}', using filename heuristics:")
        for k, v in auto_config.items():
            print(f"    {k}: {v}")
    
    # Step 3: Apply YAML overrides (user-provided values override auto-detected)
    yaml_overrides = {}
    if CONF_INPUT_TYPE in config:
        yaml_overrides['input_type'] = config[CONF_INPUT_TYPE]
    if CONF_INPUT_CHANNELS in config:
        yaml_overrides['input_channels'] = config[CONF_INPUT_CHANNELS]
    if CONF_INPUT_WIDTH in config:
        yaml_overrides['input_width'] = config[CONF_INPUT_WIDTH]
    if CONF_INPUT_HEIGHT in config:
        yaml_overrides['input_height'] = config[CONF_INPUT_HEIGHT]
    if CONF_OUTPUT_PROCESSING in config:
        yaml_overrides['output_processing'] = config[CONF_OUTPUT_PROCESSING]
    if CONF_SCALE_FACTOR in config:
        yaml_overrides['scale_factor'] = config[CONF_SCALE_FACTOR]
    if CONF_INPUT_ORDER in config:
        yaml_overrides['input_order'] = config[CONF_INPUT_ORDER]
    if CONF_NORMALIZE in config:
        yaml_overrides['normalize'] = config[CONF_NORMALIZE]
    if CONF_INVERT in config:
        yaml_overrides['invert'] = config[CONF_INVERT]
    
    if yaml_overrides:
        print(f"  YAML overrides applied:")
        for k, v in yaml_overrides.items():
            print(f"    {k}: {v}")
        auto_config.update(yaml_overrides)
    
    # Step 4: Set all config values on the C++ component
    # Use auto-detected values with sensible defaults for anything missing
    cg.add(var.set_input_type(auto_config.get('input_type', 'uint8')))
    cg.add(var.set_input_channels(auto_config.get('input_channels', 3)))
    cg.add(var.set_input_width(auto_config.get('input_width', 32)))
    cg.add(var.set_input_height(auto_config.get('input_height', 20)))
    cg.add(var.set_output_processing(auto_config.get('output_processing', 'direct_class')))
    cg.add(var.set_scale_factor(auto_config.get('scale_factor', 1.0)))
    cg.add(var.set_input_order(auto_config.get('input_order', 'RGB')))
    cg.add(var.set_normalize(auto_config.get('normalize', False)))
    cg.add(var.set_invert(auto_config.get('invert', False)))
    
    # Step 5: Set tensor arena size
    # Priority: YAML config > .txt peak analysis > default 100KB
    if CONF_TENSOR_ARENA_SIZE in config:
        arena_size = config[CONF_TENSOR_ARENA_SIZE]
        # If it's a string (like "110KB"), parse it
        if isinstance(arena_size, str):
            arena_size = datasize_to_bytes(arena_size)
        cg.add(var.set_tensor_arena_size(arena_size))
        print(f"  Tensor arena size: {arena_size} bytes (from YAML config)")
    elif 'tensor_arena_size' in auto_config:
        cg.add(var.set_tensor_arena_size(auto_config['tensor_arena_size']))
        print(f"  Tensor arena size: {auto_config['tensor_arena_size']} bytes (from .txt peak analysis)")
    else:
        # Default fallback
        cg.add(var.set_tensor_arena_size(100 * 1024))
        print(f"  Tensor arena size: {100 * 1024} bytes (default)")
    
    # Step 6: Set MAX_OPERATORS from .txt file analysis (with fallback)
    max_ops = auto_config.get('max_operators', 30)
    cg.add_build_flag(f"-DMAX_OPERATORS={max_ops}")
    print(f"  MAX_OPERATORS: {max_ops} (build flag)")
    
    if "show_crop_areas" in config:
        cg.add(var.set_show_crop_areas(config["show_crop_areas"]))
    
    # Get camera resolution from substitutions
    width, height = 640, 480  # Defaults
    substitutions = CORE.config.get("substitutions", {})
    if substitutions.get("camera_resolution"):
        res = substitutions["camera_resolution"]
        if 'x' in res:
            width, height = map(int, res.split('x'))
    
    pixel_format = substitutions.get("camera_pixel_format", "RGB888")
    cg.add(var.set_camera_image_format(width, height, pixel_format))
    
    # Find esp32_camera_utils instance to allow updating its helper sensors and for rotation detection
    camera_utils_id = None
    if 'esp32_camera_utils' in CORE.config:
        conf = CORE.config['esp32_camera_utils']
        # Handle list if multiple instances (though usually singleton or first one)
        if isinstance(conf, list) and len(conf) > 0:
            conf = conf[0]
            
        if CONF_ID in conf:
            camera_utils_id = conf[CONF_ID]

    if camera_utils_id:
        utils_var = await cg.get_variable(camera_utils_id)
        cg.add(var.set_esp32_camera_utils(utils_var))

    # Auto-enable rotation if esp32_camera_utils is present
    if camera_utils_id is not None:
        cg.add_define("DEV_ENABLE_ROTATION")
    
    cg.add_define("USE_SERVICE_DEBUG")

    if config.get(CONF_DEBUG_IMAGE, False):
        cg.add_define("DEBUG_METER_READER_TFLITE")
        cg.add(var.set_debug_mode(True))
        
        cg.add(var.set_camera_image_format(640, 480, "JPEG"))
        
        component_dir = os.path.dirname(os.path.abspath(__file__))
        debug_image_path = os.path.join(component_dir, "debug.jpg")
        
        if not os.path.exists(debug_image_path):
            raise cv.Invalid(f"Debug image not found at {debug_image_path}")
        else:
            with open(debug_image_path, "rb") as f:
                debug_image_data = f.read()
        
        debug_image_id = f"{config[CONF_ID]}_debug_image"
        cg.add_global(
            cg.RawStatement(
               f"static const uint8_t {debug_image_id}[] = {{{', '.join(f'0x{x:02x}' for x in debug_image_data)}}};"
            )
        )
        
        cg.add(
            var.set_debug_image(
                cg.RawExpression(debug_image_id),
                len(debug_image_data)
            )
        )
        
    if config.get(CONF_DEBUG, False):
        cg.add_define("DEBUG_METER_READER_TFLITE")
        cg.add(var.set_debug_mode(True))
        cg.add(var.set_debug(True))
        
    if config.get(CONF_DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL, False):
        cg.add_define("DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL")
     
    if config.get(CONF_GENERATE_PREVIEW, False):
        cg.add(var.set_generate_preview(True))
        
    if config.get(CONF_DEBUG_MEMORY, False):
        cg.add_define("DEBUG_METER_READER_MEMORY")
        cg.add(var.set_debug_memory_enabled(True))
        
        # Helper to create and register a sensor
        async def create_sensor(name, unit, accuracy_decimals=0, icon="mdi:memory"):
            # Create a manual ID for the new sensor
            sens_id = cv.declare_id(sensor.Sensor)(f"{config[CONF_ID]}_{name}")
            sens_conf = {
                CONF_ID: sens_id,
                CONF_NAME: name.replace("_", " ").title(),
                CONF_DISABLED_BY_DEFAULT: False,
                CONF_INTERNAL: False,
                CONF_ICON: icon,
                CONF_FORCE_UPDATE: False,
                CONF_ENTITY_CATEGORY: cv.entity_category("diagnostic"),
            }
            
            # sens = await sensor.new_sensor(sens_conf)
            sens = await sensor.new_sensor(sens_conf)
            
            cg.add(sens.set_unit_of_measurement(unit))
            cg.add(sens.set_accuracy_decimals(accuracy_decimals))
            # Icon is set via config now
            return sens

        # Tensor Arena Size
        s = await create_sensor("tensor_arena_size", "B", 0)
        cg.add(var.set_tensor_arena_size_sensor(s))
        
        # Tensor Arena Used
        s = await create_sensor("tensor_arena_used", "B", 0)
        cg.add(var.set_tensor_arena_used_sensor(s))
        
        # Process Free Heap
        s = await create_sensor("process_free_heap", "B", 0)
        cg.add(var.set_process_free_heap_sensor(s))
        
        # Process Free PSRAM
        s = await create_sensor("process_free_psram", "B", 0)
        cg.add(var.set_process_free_psram_sensor(s))


    # if CONF_PREVIEW in config:
    #     preview_conf = config[CONF_PREVIEW]
    #     preview_cam = cg.new_Pvariable(preview_conf[CONF_ID], var)
    #     await camera_component.register_camera(preview_cam, preview_conf)
    #     cg.add(var.set_preview_camera(preview_cam))
   
    # Check for web_server component to enable preview handler
    if 'web_server' in CORE.config:
        cg.add_define("USE_WEB_SERVER")
        
        # We need the WebServerBase component for add_handler
        # It is usually available as 'web_server_base' in config if web_server is used.
        if 'web_server_base' in CORE.config:
            ws_config = CORE.config['web_server_base']
            if isinstance(ws_config, list):
                ws_config = ws_config[0]
            ws_id = ws_config[CONF_ID]
            ws_var = await cg.get_variable(ws_id)
            cg.add(var.set_web_server(ws_var))
        else:
             # Fallback to default ID assumption
             ws_base_id = cv.declare_id(cg.esphome_ns.namespace('web_server_base').class_('WebServerBase'))('web_server_base')
             ws_var = await cg.get_variable(ws_base_id)
             cg.add(var.set_web_server(ws_var))

    # Handle crop zones (either global or local)
    if CONF_CROP_ZONES in config:
        crop_global = await cg.get_variable(config[CONF_CROP_ZONES])
        cg.add(var.set_crop_zones_global(crop_global))    

    # Set flash light controller if configured (optional)
    if CONF_FLASH_LIGHT_CONTROLLER in config:
        cg.add_define("USE_FLASH_LIGHT_CONTROLLER")
        flash_controller = await cg.get_variable(config[CONF_FLASH_LIGHT_CONTROLLER])
        cg.add(var.set_flash_controller(flash_controller))


    if CONF_DATA_COLLECTOR in config:
        dc = await cg.get_variable(config[CONF_DATA_COLLECTOR])
        cg.add(var.set_data_collector(dc))
        cg.add(var.set_collect_low_confidence(config[CONF_COLLECT_LOW_CONFIDENCE]))
        if CONF_COLLECT_MIN_GLOBAL_CONFIDENCE in config:
            cg.add(var.set_collect_min_global_confidence(config[CONF_COLLECT_MIN_GLOBAL_CONFIDENCE]))
        if CONF_COLLECT_MIN_DIGIT_CONFIDENCE in config:
            cg.add(var.set_collect_min_digit_confidence(config[CONF_COLLECT_MIN_DIGIT_CONFIDENCE]))
        cg.add_define("USE_DATA_COLLECTOR")
    
    # Handle optional camera window configuration
    # if CONF_CAMERA_WINDOW in config:
    #     window_config = config[CONF_CAMERA_WINDOW]
    #     if 'width' in window_config and 'height' in window_config:
    #         offset_x = window_config.get('offset_x', 0)
    #         offset_y = window_config.get('offset_y', 0)
    #         width = window_config['width']
    #         height = window_config['height']
            
    #         # Store window configuration as member variables
    #         cg.add(var.set_camera_window_offset_x(offset_x))
    #         cg.add(var.set_camera_window_offset_y(offset_y))
    #         cg.add(var.set_camera_window_width(width))
    #         cg.add(var.set_camera_window_height(height))
    #         cg.add(var.set_camera_window_configured(True)) 


    # Set timeout parameters
    if CONF_FRAME_REQUEST_TIMEOUT in config:
        cg.add(var.set_frame_request_timeout(config[CONF_FRAME_REQUEST_TIMEOUT]))
        

    # Optional: Debug memory sensors
    sensor_keys = [
        "tensor_arena_size_sensor", "tensor_arena_used_sensor",
        "process_free_heap_sensor", "process_free_psram_sensor",
        "pool_job_efficiency_sensor", "pool_result_efficiency_sensor",
        "arena_efficiency_sensor", "heap_fragmentation_sensor",
    ]
    for key in sensor_keys:
        if key in config:
            sens = await cg.get_variable(config[key])
            setter_name = f"set_{key}"
            cg.add(getattr(var, setter_name)(sens))
    
    
    if "value_sensor" in config:
        value_sensor = await cg.get_variable(config["value_sensor"])
        cg.add(var.set_value_sensor(value_sensor))
    
    if "confidence_sensor" in config:
        confidence_sensor = await cg.get_variable(config["confidence_sensor"])
        cg.add(var.set_confidence_sensor(confidence_sensor))
    
    if "inference_logs" in config:
        inference_logs = await cg.get_variable(config["inference_logs"])
        cg.add(var.set_inference_logs(inference_logs))
    
    if "main_logs" in config:
        main_logs = await cg.get_variable(config["main_logs"])
        cg.add(var.set_main_logs(main_logs))


    if CONF_START_FLASH_CALIBRATION_BUTTON in config:
        b = await cg.get_variable(config[CONF_START_FLASH_CALIBRATION_BUTTON])
        cg.add(b.set_on_press_callback(
            cg.Lambda(
                f"id({var.get_id()})->start_flash_calibration();"
            )
        ))

    if config.get("enable_flash_calibration", False):
        cg.add(var.set_enable_flash_calibration(True))

    cg.add_define("DEBUG_METER_READER_TIMING")
    cg.add(var.set_debug_timing(config.get("debug_timing", False)))

    if "total_inference_time_sensor" in config:
        s = await cg.get_variable(config["total_inference_time_sensor"])
        cg.add(var.set_total_inference_time_sensor(s))

    if "capture_to_publish_time_sensor" in config:
        s = await cg.get_variable(config["capture_to_publish_time_sensor"])
        cg.add(var.set_capture_to_publish_time_sensor(s))

    if CONF_UNLOAD_BUTTON in config:
        b = await cg.get_variable(config[CONF_UNLOAD_BUTTON])
        cg.add(var.set_unload_button(b))
    
    if CONF_RELOAD_BUTTON in config:
        b = await cg.get_variable(config[CONF_RELOAD_BUTTON])
        cg.add(var.set_reload_button(b))

