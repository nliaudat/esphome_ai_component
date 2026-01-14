import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
import esphome.components.esp32_camera as esp32_camera
from esphome.const import CONF_ID, CONF_OFFSET_X, CONF_OFFSET_Y, CONF_WIDTH, CONF_HEIGHT, CONF_NAME, CONF_DISABLED_BY_DEFAULT, CONF_INTERNAL, CONF_ICON, CONF_FORCE_UPDATE
from esphome.components import sensor
from esphome.core import CORE


DEPENDENCIES = ['esp32']

esp32_camera_utils_ns = cg.esphome_ns.namespace('esp32_camera_utils')
Esp32CameraUtils = esp32_camera_utils_ns.class_('Esp32CameraUtils', cg.Component)

CONF_CAMERA_WINDOW = 'camera_window'
CONF_CAMERA_ID = 'camera_id'
CONF_ROTATION = 'rotation'

CONF_SCALER = 'scaler'
CONF_CROPPER = 'cropper'

# Rotation options mapping
ROTATION_OPTIONS = {
    "0": 0,
    "90": 90,
    "180": 180,
    "270": 270,
}

CONF_DEBUG_MEMORY = 'debug_memory'


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Esp32CameraUtils),
    cv.Optional(CONF_CAMERA_WINDOW): cv.Schema({
        cv.Required(CONF_OFFSET_X): cv.int_,
        cv.Required(CONF_OFFSET_Y): cv.int_,
        cv.Required(CONF_WIDTH): cv.int_,
        cv.Required(CONF_HEIGHT): cv.int_,
    }),
    cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
    cv.Optional("debug", default=False): cv.boolean,
    # Allow arbitrary rotation (float), but also string for backward compat "90"
    cv.Optional(CONF_ROTATION, default=0.0): cv.float_,
    cv.Optional("enable_rotation", default=False): cv.boolean,
    cv.Optional(CONF_SCALER): cv.Schema({
        cv.Optional(CONF_WIDTH): cv.int_,
        cv.Optional(CONF_HEIGHT): cv.int_,
    }),
    cv.Optional(CONF_CROPPER): cv.Schema({
        cv.Optional(CONF_WIDTH): cv.int_,
        cv.Optional(CONF_HEIGHT): cv.int_,
        cv.Optional(CONF_OFFSET_X, default=0): cv.int_,
        cv.Optional(CONF_OFFSET_Y, default=0): cv.int_,
    }),
    cv.Optional("enable_scaler", default=True): cv.boolean,
    cv.Optional("enable_cropper", default=True): cv.boolean,
    cv.Optional("enable_drawing", default=True): cv.boolean,
    cv.Optional(CONF_DEBUG_MEMORY, default=False): cv.boolean,
    cv.Optional("camera_buffer_size_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("camera_free_psram_sensor"): cv.use_id(sensor.Sensor),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    esp32.add_idf_component(
        name="espressif/esp_new_jpeg",
        ref="1.0.0"
    )

    cg.add_build_flag("-DUSE_ESP32_CAMERA_CONV")

    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_CAMERA_WINDOW in config:
        conf = config[CONF_CAMERA_WINDOW]
        cg.add(var.set_camera_window_config(conf[CONF_OFFSET_X], conf[CONF_OFFSET_Y], conf[CONF_WIDTH], conf[CONF_HEIGHT]))
        
    if CONF_CAMERA_ID in config:
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))
    
    # Set image rotation
    cg.add(var.set_rotation(config[CONF_ROTATION]))
    
    # Enable modular features
    if config.get("enable_rotation", False) or config[CONF_ROTATION] != 0:
         cg.add_define("USE_CAMERA_ROTATOR")

    if config.get("enable_scaler", True) or CONF_SCALER in config:
        cg.add_define("USE_CAMERA_SCALER")
        if CONF_SCALER in config:
            conf = config[CONF_SCALER]
            if CONF_WIDTH in conf and CONF_HEIGHT in conf:
                cg.add(var.set_scaler_config(conf[CONF_WIDTH], conf[CONF_HEIGHT]))
        
    if config.get("enable_cropper", True) or CONF_CROPPER in config:
        cg.add_define("USE_CAMERA_CROPPER")
        if CONF_CROPPER in config:
            conf = config[CONF_CROPPER]
            if CONF_WIDTH in conf and CONF_HEIGHT in conf:
                cg.add(var.set_cropper_config(conf[CONF_WIDTH], conf[CONF_HEIGHT], conf[CONF_OFFSET_X], conf[CONF_OFFSET_Y]))
        
    if config.get("enable_drawing", True):
        cg.add_define("USE_CAMERA_DRAWING")

    if config.get("debug", False):
        cg.add_define("DEBUG_ESP32_CAMERA_UTILS")
        cg.add(var.set_debug(True))
        
    if 'web_server' in CORE.config:
        cg.add_define("USE_WEB_SERVER")

    if config.get(CONF_DEBUG_MEMORY, False):
        cg.add_define("DEBUG_ESP32_CAMERA_UTILS_MEMORY")
        
        # Helper to create and register a sensor
        async def create_sensor(name, unit, accuracy_decimals=0, icon="mdi:memory"):
            # Create a manual ID for the new sensor
            sens_id = cv.declare_id(sensor.Sensor)(f"{config[CONF_ID]}_{name}")
            sens_conf = {
                CONF_ID: sens_id,
                CONF_NAME: name.replace("_", " ").title(),
                CONF_DISABLED_BY_DEFAULT: False,
                CONF_INTERNAL: False,
                CONF_INTERNAL: False,
                CONF_ICON: icon,
                CONF_FORCE_UPDATE: False,
            }
            
            sens = await sensor.new_sensor(sens_conf)
            cg.add(sens.set_unit_of_measurement(unit))
            cg.add(sens.set_accuracy_decimals(accuracy_decimals))
            return sens

        # Buffer Size
        s = await create_sensor("camera_buffer_size", "B", 0)
        cg.add(var.set_camera_buffer_size_sensor(s))
        
        # Free PSRAM
        s = await create_sensor("camera_free_psram", "B", 0)
        cg.add(var.set_camera_free_psram_sensor(s))


