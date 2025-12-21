"""Component to use TensorFlow Lite Micro to read a meter."""
import esphome.codegen as cg
import esphome.config_validation as cv
import os
import zlib
from esphome.const import CONF_ID, CONF_MODEL, CONF_ROTATION, CONF_NAME, CONF_DISABLED_BY_DEFAULT, CONF_INTERNAL, CONF_ICON, CONF_FORCE_UPDATE, CONF_ENTITY_CATEGORY
from esphome.core import CORE, HexInt
from esphome.components import esp32, sensor, text_sensor, button

import esphome.components.esp32_camera as esp32_camera
from esphome.cpp_generator import RawExpression
from esphome.components import globals
try:
    import esphome.components.flash_light_controller as flash_light_controller
except ImportError:
    flash_light_controller = None

CODEOWNERS = ["@nl"]
if CORE.target_platform == "esp32":
    DEPENDENCIES = ['esp32', 'tflite_micro_helper', 'esp32_camera_utils', 'flash_light_controller']
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

# CONF_MODEL_TYPE = 'model_type' 
CONF_ROTATION = 'rotation'
CONF_PREVIEW = 'preview_camera'
CONF_GENERATE_PREVIEW = 'generate_preview'
CONF_START_FLASH_CALIBRATION_BUTTON = 'start_flash_calibration_button'

# Rotation options mapping
ROTATION_OPTIONS = {
    "0": 0,
    "90": 90,
    "180": 180,
    "270": 270,
}
CONF_FLASH_LIGHT_CONTROLLER = 'flash_light_controller'

CONF_CROP_ZONES = 'crop_zones_global'

CONF_CAMERA_WINDOW = 'camera_window'

CONF_ALLOW_NEGATIVE_RATES = 'allow_negative_rates'
CONF_MAX_ABSOLUTE_DIFF = 'max_absolute_diff'
CONF_FRAME_REQUEST_TIMEOUT = 'frame_request_timeout'
CONF_HIGH_CONFIDENCE_THRESHOLD = 'high_confidence_threshold'

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

# Use the standard ESPHome sensor configuration pattern
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MeterReaderTFLite),
    cv.Required(CONF_MODEL): cv.file_,
    cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera) if CORE.target_platform == "esp32" else cv.string,
    # cv.Optional(CONF_MODEL_TYPE, default="class100-0180"): cv.string,  # Add model type selection
    cv.Optional(CONF_CONFIDENCE_THRESHOLD, default=0.85): cv.float_range(
        min=0.0, max=1.0
    ),
    cv.Optional(CONF_ROTATION, default="0"): cv.float_range(min=0, max=360),
    # Make tensor_arena_size optional since it's now in model_config.h
    cv.Optional(CONF_TENSOR_ARENA_SIZE): cv.All( 
        datasize_to_bytes,
        cv.Range(min=50 * 1024, max=1000 * 1024)
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

    cv.Optional(CONF_FLASH_LIGHT_CONTROLLER): cv.use_id(flash_light_controller.FlashLightController) if flash_light_controller else cv.string,
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
    cv.Optional(CONF_ALLOW_NEGATIVE_RATES, default=False): cv.boolean,
    cv.Optional(CONF_MAX_ABSOLUTE_DIFF, default=100): cv.positive_int,
    cv.Optional(CONF_FRAME_REQUEST_TIMEOUT, default=15000): cv.int_range(min=1000, max=60000),
    cv.Optional(CONF_HIGH_CONFIDENCE_THRESHOLD, default=0.90): cv.float_range(min=0.5, max=1.0),
    cv.Optional("value_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("confidence_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("inference_logs"): cv.use_id(text_sensor.TextSensor),
    cv.Optional("main_logs"): cv.use_id(text_sensor.TextSensor),
    cv.Optional(CONF_START_FLASH_CALIBRATION_BUTTON): cv.use_id(button.Button),
    cv.Optional(CONF_GENERATE_PREVIEW, default=False): cv.boolean,
    cv.Optional("enable_rotation", default=False): cv.boolean,
    cv.Optional("show_crop_areas", default=True): cv.boolean,
    cv.Optional("enable_flash_calibration", default=False): cv.boolean,
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

    if CORE.target_platform == "esp32":
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))
    else:
        cg.add_define("USE_HOST_MOCK_CAMERA")
        cg.add_define("USE_HOST")
        # On host, we don't set a real camera object.
        pass
    
    model_path = config[CONF_MODEL]
    model_filename = os.path.basename(model_path)
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
    
    # Set tensor arena size - use config value if provided, otherwise use default
    # The actual size will be determined from model_config.h in the C++ code
    if CONF_TENSOR_ARENA_SIZE in config:
        cg.add(var.set_tensor_arena_size(config[CONF_TENSOR_ARENA_SIZE]))
    if "show_crop_areas" in config:
        cg.add(var.set_show_crop_areas(config["show_crop_areas"]))
    # else:
        # # Default will be handled in the C++ code based on model type from model_config.h
        # cg.add(var.set_tensor_arena_size(512 * 1024))  # 512KB default fallback
    
    # Get camera resolution from substitutions
    width, height = 640, 480  # Defaults
    substitutions = CORE.config.get("substitutions", {})
    if substitutions.get("camera_resolution"):
        res = substitutions["camera_resolution"]
        if 'x' in res:
            width, height = map(int, res.split('x'))
    
            width, height = map(int, res.split('x'))
    
    pixel_format = substitutions.get("camera_pixel_format", "RGB888")
    cg.add(var.set_camera_image_format(width, height, pixel_format))
    
    # Set image rotation
    # Priority 1: Check meter_reader_tflite specific rotation
    rotation_value = 0.0
    rotation_conf = config.get(CONF_ROTATION)
    
    # Priority 2: Check esp32_camera_utils configuration if not set locally or is "0" (str)
    # Note: rotation_conf comes as string from one_of validator if we used that, but now it matches float?
    # Actually validation below ensures it's float-compatible
    
    # But wait, we need to handle the fallback lookup
    # Check if rotation is effectively 0
    is_zero = False
    try:
        if float(rotation_conf) == 0:
            is_zero = True
    except:
        pass

    if is_zero:
        # Search for esp32_camera_utils in top-level config
        for comp_name, comp_config in CORE.config.items():
            if comp_name.startswith("esp32_camera_utils"):
                conf_list = comp_config if isinstance(comp_config, list) else [comp_config]
                for conf in conf_list:
                     if "rotation" in conf:
                         rotation_conf = conf["rotation"]
                         break
                         
    # print(f"DEBUG_ROTATION_CHECK: Final rotation value is '{rotation_conf}'")
    
    # Ensure it's a float
    try:
        rotation_value = float(rotation_conf)
    except:
        rotation_value = 0.0

    cg.add(var.set_rotation(rotation_value))
    
    if config.get("enable_rotation", False):
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

    # Set flash light controller if configured
    if CONF_FLASH_LIGHT_CONTROLLER in config:
        flash_controller = await cg.get_variable(config[CONF_FLASH_LIGHT_CONTROLLER])
        cg.add(var.set_flash_controller(flash_controller))
    
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


    # Set validation parameters
    if CONF_ALLOW_NEGATIVE_RATES in config:
        cg.add(var.set_allow_negative_rates(config[CONF_ALLOW_NEGATIVE_RATES]))
    if CONF_MAX_ABSOLUTE_DIFF in config:
        cg.add(var.set_max_absolute_diff(config[CONF_MAX_ABSOLUTE_DIFF]))
    
    # Set timeout and threshold parameters
    if CONF_FRAME_REQUEST_TIMEOUT in config:
        cg.add(var.set_frame_request_timeout(config[CONF_FRAME_REQUEST_TIMEOUT]))
    if CONF_HIGH_CONFIDENCE_THRESHOLD in config:
        cg.add(var.set_high_confidence_threshold(config[CONF_HIGH_CONFIDENCE_THRESHOLD]))
    

    
    
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

    # Find esp32_camera_utils instance to allow updating its helper sensors
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
    if CONF_START_FLASH_CALIBRATION_BUTTON in config:
        b = await cg.get_variable(config[CONF_START_FLASH_CALIBRATION_BUTTON])
        cg.add(b.set_on_press_callback(
            cg.Lambda(
                f"id({var.get_id()})->start_flash_calibration();"
            )
        ))

    if config.get("enable_flash_calibration", False):
         cg.add(var.set_enable_flash_calibration(True))

