"""Component to use TensorFlow Lite Micro to read a meter."""
import esphome.codegen as cg
import esphome.config_validation as cv
import os
import zlib
from esphome.const import CONF_ID, CONF_MODEL
from esphome.core import CORE, HexInt
from esphome.components import esp32, sensor
import esphome.components.esp32_camera as esp32_camera
from esphome.cpp_generator import RawExpression
from esphome.components import globals

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
CONF_MODEL_TYPE = 'model_type'  # New configuration option for model type

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
    cv.Optional(CONF_MODEL_TYPE, default="class100-0180"): cv.string,  # Add model type selection
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
    cv.Optional('crop_zones_global'): cv.use_id(globals.GlobalsComponent),
}).extend(cv.polling_component_schema('60s'))

async def to_code(config):
    """Code generation for the component."""
    # Add IDF component and build flags
    esp32.add_idf_component(
        name="espressif/esp-tflite-micro",
        ref="~1.3.4"
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
    
    # Set model type from configuration
    cg.add(var.set_model_config(config[CONF_MODEL_TYPE]))
    
    model_path = config[CONF_MODEL]
    
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
    else:
        # Default will be handled in the C++ code based on model type from model_config.h
        cg.add(var.set_tensor_arena_size(512 * 1024))  # 512KB default fallback
    
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

    if 'crop_zones_global' in config:
        crop_global = await cg.get_variable(config['crop_zones_global'])
        cg.add(var.set_crop_zones_global_string(crop_global.value()))