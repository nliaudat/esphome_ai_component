import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
import esphome.components.esp32_camera as esp32_camera
from esphome.const import CONF_ID, CONF_OFFSET_X, CONF_OFFSET_Y, CONF_WIDTH, CONF_HEIGHT

DEPENDENCIES = ['esp32', 'camera']

esp32_camera_utils_ns = cg.esphome_ns.namespace('esp32_camera_utils')
Esp32CameraUtils = esp32_camera_utils_ns.class_('Esp32CameraUtils', cg.Component)

CONF_CAMERA_WINDOW = 'camera_window'
CONF_CAMERA_ID = 'camera_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Esp32CameraUtils),
    cv.Optional(CONF_CAMERA_WINDOW): cv.Schema({
        cv.Required(CONF_OFFSET_X): cv.int_,
        cv.Required(CONF_OFFSET_Y): cv.int_,
        cv.Required(CONF_WIDTH): cv.int_,
        cv.Required(CONF_HEIGHT): cv.int_,
    }),
    cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    esp32.add_idf_component(
        name="espressif/esp_new_jpeg",
        ref="0.6.1"
    )
    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_CAMERA_WINDOW in config:
        conf = config[CONF_CAMERA_WINDOW]
        cg.add(var.set_camera_window_config(conf[CONF_OFFSET_X], conf[CONF_OFFSET_Y], conf[CONF_WIDTH], conf[CONF_HEIGHT]))
        
    if CONF_CAMERA_ID in config:
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))
