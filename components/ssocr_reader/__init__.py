import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, esp32_camera, value_validator
from esphome.core import CORE
from esphome.const import (
    CONF_ID,
    CONF_NAME,
)

CONF_CAMERA_ID = "camera_id"

DEPENDENCIES = ["esp32_camera", "value_validator"]
AUTO_LOAD = ["esp32_camera_utils"]

ssocr_reader_ns = cg.esphome_ns.namespace("ssocr_reader")
SSOCRReader = ssocr_reader_ns.class_("SSOCRReader", cg.PollingComponent)

CONF_THRESHOLD_TYPE = "threshold_type"
CONF_THRESHOLD_LEVEL = "threshold_level"
CONF_CROP_X = "crop_x"
CONF_CROP_Y = "crop_y"
CONF_CROP_W = "crop_w"
CONF_CROP_H = "crop_h"
CONF_DIGIT_COUNT = "digit_count"
CONF_DECIMAL_POINT = "decimal_point"

CONF_VALUE = "value"
CONF_VALIDATOR = "validator"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SSOCRReader),
        cv.Optional(CONF_VALIDATOR): cv.use_id(value_validator.ValueValidator),
        cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
        cv.Optional(CONF_VALUE): sensor.sensor_schema(),
        cv.Optional("debug", default=False): cv.boolean,
        cv.Optional(CONF_THRESHOLD_TYPE, default="fixed"): cv.enum(
            {"fixed": 0, "otsu": 1}, lower=True
        ),
        cv.Optional(CONF_THRESHOLD_LEVEL, default=128): cv.int_range(min=0, max=255),
        cv.Optional(CONF_CROP_X, default=0): cv.int_range(min=0),
        cv.Optional(CONF_CROP_Y, default=0): cv.int_range(min=0),
        cv.Optional(CONF_CROP_W, default=0): cv.int_range(min=0),
        cv.Optional(CONF_CROP_H, default=0): cv.int_range(min=0),
        cv.Optional(CONF_DIGIT_COUNT, default=6): cv.int_range(min=1, max=16),
    }
).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if config.get("debug", False):
        cg.add(var.set_debug(True))

    # Validator
    if CONF_VALIDATOR in config:
        v = await cg.get_variable(config[CONF_VALIDATOR])
        cg.add(var.set_validator(v))

    if CONF_CAMERA_ID in config:
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))

    if CONF_VALUE in config:
        sens = await sensor.new_sensor(config[CONF_VALUE])
        cg.add(var.set_value_sensor(sens))

    cg.add(var.set_threshold_config(config[CONF_THRESHOLD_LEVEL]))
    cg.add(var.set_crop_config(config[CONF_CROP_X], config[CONF_CROP_Y], config[CONF_CROP_W], config[CONF_CROP_H]))
    cg.add(var.set_digit_config(config[CONF_DIGIT_COUNT]))

    # Inject resolution and format
    width, height = 640, 480
    pixel_format = "JPEG"
    
    substitutions = CORE.config.get("substitutions", {})
    if "camera_resolution" in substitutions:
        try:
             res = substitutions["camera_resolution"]
             if 'x' in res:
                w, h = map(int, res.split('x'))
                width, height = w, h
        except: pass
    
    if "camera_pixel_format" in substitutions:
        pixel_format = substitutions["camera_pixel_format"]

    cg.add(var.set_resolution(width, height))
    cg.add(var.set_pixel_format_str(pixel_format))
