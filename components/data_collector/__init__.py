import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32_camera"]

data_collector_ns = cg.esphome_ns.namespace("data_collector")
DataCollector = data_collector_ns.class_("DataCollector", cg.Component)

CONF_UPLOAD_URL = "upload_url"
CONF_WEB_SUBMIT = "web_submit"
CONF_USERNAME = "username"
CONF_PASSWORD = "password"
CONF_API_KEY = "api_key"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DataCollector),
        cv.Optional(CONF_UPLOAD_URL): cv.string,
        cv.Optional(CONF_USERNAME): cv.string,
        cv.Optional(CONF_PASSWORD): cv.string,
        cv.Optional(CONF_API_KEY): cv.string,
        cv.Optional(CONF_WEB_SUBMIT): switch.switch_schema(
            icon="mdi:cloud-upload",
            entity_category="config",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_UPLOAD_URL in config:
        cg.add(var.set_upload_url(config[CONF_UPLOAD_URL]))
    
    if CONF_USERNAME in config:
        cg.add(var.set_auth(config[CONF_USERNAME], config.get(CONF_PASSWORD, "")))
        
    if CONF_API_KEY in config:
        cg.add(var.set_api_key(config[CONF_API_KEY]))
    
    if CONF_WEB_SUBMIT in config:
        sens = await switch.new_switch(config[CONF_WEB_SUBMIT])
        cg.add(var.set_web_submit_switch(sens))
