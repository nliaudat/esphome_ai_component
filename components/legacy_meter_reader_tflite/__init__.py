"""Component to use TensorFlow Lite Micro to read a meter."""
import esphome.codegen as cg
import esphome.config_validation as cv
import os
import zlib
from esphome.const import CONF_ID, CONF_MODEL
from esphome.core import CORE, HexInt
from esphome.components import esp32, sensor, text_sensor
import esphome.components.esp32_camera as esp32_camera
from esphome.cpp_generator import RawExpression
from esphome.components import globals
from esphome.components import light

CODEOWNERS = ["@nl"]
DEPENDENCIES = ['esp32', 'camera']
AUTO_LOAD = ['sensor']

CONF_CAMERA_ID = 'camera_id'
CONF_TENSOR_ARENA_SIZE = 'tensor_arena_size'
CONF_CONFIDENCE_THRESHOLD = 'confidence_threshold'
CONF_RAW_DATA_ID = 'raw_data_id'
CONF_DEBUG = 'debug'
CONF_DEBUG_IMAGE = 'debug_image'
CONF_DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL = 'debug_image_out_serial'
# CONF_MODEL_TYPE = 'model_type' 

CONF_FLASH_LIGHT = 'flash_light'
CONF_FLASH_PRE_TIME = 'flash_pre_time'
CONF_FLASH_POST_TIME = 'flash_post_time'

CONF_CROP_ZONES = 'crop_zones_global'

CONF_CAMERA_WINDOW = 'camera_window'

CONF_ALLOW_NEGATIVE_RATES = 'allow_negative_rates'
CONF_MAX_ABSOLUTE_DIFF = 'max_absolute_diff'

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
    cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
    # cv.Optional(CONF_MODEL_TYPE, default="class100-0180"): cv.string,  # Add model type selection
    cv.Optional(CONF_CONFIDENCE_THRESHOLD, default=0.7): cv.float_range(
        min=0.0, max=1.0
    ),
    # Make tensor_arena_size optional since it's now in model_config.h
    cv.Optional(CONF_TENSOR_ARENA_SIZE): cv.All( 
        datasize_to_bytes,
        cv.Range(min=50 * 1024, max=1000 * 1024)
    ),
    cv.GenerateID(CONF_RAW_DATA_ID): cv.declare_id(cg.uint8),
    cv.Optional(CONF_DEBUG, default=False): cv.boolean, 
    cv.Optional(CONF_DEBUG_IMAGE, default=False): cv.boolean, 
    cv.Optional(CONF_DEBUG_OUT_PROCESSED_IMAGE_TO_SERIAL, default=False): cv.boolean,
    cv.Optional(CONF_FLASH_LIGHT): cv.use_id(light.LightState), 
    cv.Optional(CONF_FLASH_PRE_TIME, default=5000): cv.positive_int,
    cv.Optional(CONF_FLASH_POST_TIME, default=2000): cv.positive_int,
    # cv.Optional(CONF_FLASH_DURATION, default=2200): cv.positive_int, 
    cv.Optional(CONF_CROP_ZONES): cv.use_id(globals.GlobalsComponent),
    cv.Optional(CONF_CAMERA_WINDOW): cv.Any(
    cv.Schema({  # Or detailed configuration
        cv.Optional('offset_x', default=0): cv.int_,
        cv.Optional('offset_y', default=0): cv.int_,
        cv.Optional('width'): cv.int_,
        cv.Optional('height'): cv.int_,
        })
    ),
    # cv.Optional(CONF_AUTO_CAMERA_WINDOW, default=False): cv.boolean,
    cv.Optional(CONF_ALLOW_NEGATIVE_RATES, default=False): cv.boolean,
    cv.Optional(CONF_MAX_ABSOLUTE_DIFF, default=100): cv.positive_int,
    cv.Optional("value_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("confidence_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("inference_logs"): cv.use_id(text_sensor.TextSensor),
    cv.Optional("main_logs"): cv.use_id(text_sensor.TextSensor),
}).extend(cv.polling_component_schema('60s'))

async def to_code(config):
    """Code generation for the component."""

    esp32.add_idf_component(
        name="espressif/esp-tflite-micro",
        # ref="~1.3.4" #https://github.com/espressif/esp-tflite-micro/issues/120
        ref="1.3.4" # fix to 1.3.4 cause 1.3.5 has bug
    )
    
    esp32.add_idf_component(
        name="espressif/esp-nn",
        ref="~1.1.2"
    )
    
    esp32.add_idf_component(
        name="espressif/esp_new_jpeg",
        ref="0.6.1"
    )
        
    cg.add_build_flag("-DTF_LITE_STATIC_MEMORY")
    cg.add_build_flag("-DTF_LITE_DISABLE_X86_NEON")
    cg.add_build_flag("-DESP_NN")
    cg.add_build_flag("-DUSE_ESP32_CAMERA_CONV")
    cg.add_build_flag("-DOPTIMIZED_KERNEL=esp_nn")
    
    #memory debug
    # cg.add_build_flag("CONFIG_HEAP_TRACING_STANDALONE")
    # cg.add_build_flag("CONFIG_HEAP_TRACING_DEST")


    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cam = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(cam))
      
    # Extract model type from filename (without extension)
    model_path = config[CONF_MODEL]
    model_filename = os.path.basename(model_path)
    model_type = os.path.splitext(model_filename)[0]  # Remove .tflite extension
       
    # Set model type from extracted filename
    cg.add(var.set_model_config(model_type))
    
    # model_path = config[CONF_MODEL]
    
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
    # else:
        # # Default will be handled in the C++ code based on model type from model_config.h
        # cg.add(var.set_tensor_arena_size(512 * 1024))  # 512KB default fallback
    
    # Get camera resolution from substitutions
    width, height = 640, 480  # Defaults
    if CORE.config["substitutions"].get("camera_resolution"):
        res = CORE.config["substitutions"]["camera_resolution"]
        if 'x' in res:
            width, height = map(int, res.split('x'))
    
    pixel_format = CORE.config["substitutions"].get("camera_pixel_format", "RGB888")
    if pixel_format == "JPEG":   
        cg.add(var.set_camera_image_format(width, height, pixel_format))
    
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

    if CONF_CROP_ZONES in config:
        crop_global = await cg.get_variable(config[CONF_CROP_ZONES])
        cg.add(var.set_crop_zones_global(crop_global))    


        
    # Set flash light if configured
    if CONF_FLASH_LIGHT in config:
        flash_light = await cg.get_variable(config[CONF_FLASH_LIGHT])
        cg.add(var.set_flash_light(flash_light))
        
    if CONF_FLASH_PRE_TIME in config:
        cg.add(var.set_flash_pre_time(config[CONF_FLASH_PRE_TIME]))
    
    if CONF_FLASH_POST_TIME in config:
        cg.add(var.set_flash_post_time(config[CONF_FLASH_POST_TIME]))
    

    # Handle optional camera window configuration
    if CONF_CAMERA_WINDOW in config:
        window_config = config[CONF_CAMERA_WINDOW]
        if 'width' in window_config and 'height' in window_config:
            offset_x = window_config.get('offset_x', 0)
            offset_y = window_config.get('offset_y', 0)
            width = window_config['width']
            height = window_config['height']
            
            # Store window configuration as member variables
            cg.add(var.set_camera_window_offset_x(offset_x))
            cg.add(var.set_camera_window_offset_y(offset_y))
            cg.add(var.set_camera_window_width(width))
            cg.add(var.set_camera_window_height(height))
            cg.add(var.set_camera_window_configured(True))


    # Set validation parameters
    cg.add(var.set_allow_negative_rates(config[CONF_ALLOW_NEGATIVE_RATES]))
    cg.add(var.set_max_absolute_diff(config[CONF_MAX_ABSOLUTE_DIFF]))
    
    
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