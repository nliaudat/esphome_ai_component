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

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(FlashLightController),
    cv.Optional(CONF_FLASH_LIGHT): cv.use_id(light.LightState),
    cv.Optional(CONF_FLASH_PRE_TIME, default="5s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_FLASH_POST_TIME, default="2s"): cv.positive_time_period_milliseconds,
    cv.Optional("debug", default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_FLASH_LIGHT in config:
        flash = await cg.get_variable(config[CONF_FLASH_LIGHT])
        cg.add(var.set_flash_light(flash))
    
    if CONF_FLASH_PRE_TIME in config:
        cg.add(var.set_flash_pre_time(config[CONF_FLASH_PRE_TIME]))
        
    if CONF_FLASH_POST_TIME in config:
        cg.add(var.set_flash_post_time(config[CONF_FLASH_POST_TIME]))

    if config.get("debug", False):
        cg.add(var.set_debug(True))
