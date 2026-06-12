import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light
from esphome.const import CONF_ID

DEPENDENCIES = ['light']

flash_light_controller_ns = cg.esphome_ns.namespace('flash_light_controller')
FlashLightController = flash_light_controller_ns.class_('FlashLightController', cg.Component)

CONF_FLASH_LIGHT = 'flash_light'
CONF_FLASH_PRE_TIME = 'flash_pre_time'
CONF_FLASH_POST_TIME = 'flash_post_time'
CONF_FLASH_BRIGHTNESS = 'flash_brightness'
CONF_ENABLE_AUTOFOCUS = 'enable_autofocus'
CONF_AF_TIMEOUT = 'af_timeout'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(FlashLightController),
    cv.Optional(CONF_FLASH_LIGHT): cv.use_id(light.LightState),
    cv.Optional(CONF_FLASH_PRE_TIME, default="5s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_FLASH_POST_TIME, default="2s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_FLASH_BRIGHTNESS, default=1.0): cv.percentage,
    cv.Optional(CONF_ENABLE_AUTOFOCUS, default=False): cv.boolean,
    cv.Optional(CONF_AF_TIMEOUT, default="2s"): cv.positive_time_period_milliseconds,
    cv.Optional("idle_brightness", default=0.0): cv.percentage,
    cv.Optional("debug", default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add_define("USE_FLASH_LIGHT_CONTROLLER")

    if CONF_FLASH_LIGHT in config:
        flash = await cg.get_variable(config[CONF_FLASH_LIGHT])
        cg.add(var.set_flash_light(flash))
    
    if CONF_FLASH_PRE_TIME in config:
        cg.add(var.set_flash_pre_time(config[CONF_FLASH_PRE_TIME]))
        
    if CONF_FLASH_POST_TIME in config:
        cg.add(var.set_flash_post_time(config[CONF_FLASH_POST_TIME]))

    cg.add(var.set_flash_brightness(config[CONF_FLASH_BRIGHTNESS]))

    if config.get(CONF_ENABLE_AUTOFOCUS, False):
        cg.add_build_flag("-DCONFIG_CAMERA_AF_SUPPORT=1")
        cg.add(var.set_enable_autofocus(True))
        cg.add(var.set_af_timeout(config[CONF_AF_TIMEOUT]))

    if config.get("debug", False):
        cg.add(var.set_debug(True))

    idle = config.get("idle_brightness", 0.0)
    if idle > 0.0:
        cg.add(var.set_idle_brightness(idle))
