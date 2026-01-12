import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, esp32_camera, esp32, value_validator

from esphome.const import CONF_ID, CONF_NAME
from esphome.core import CORE

DEPENDENCIES = ["esp32_camera", "value_validator"]
AUTO_LOAD = ["esp32_camera_utils"]

analog_reader_ns = cg.esphome_ns.namespace("analog_reader")
AnalogReader = analog_reader_ns.class_("AnalogReader", cg.PollingComponent)

CONF_CAMERA_ID = "camera_id"
CONF_DIALS = "dials"
CONF_VALUE_SENSOR = "value_sensor"
CONF_SCALE = "scale"
CONF_MIN_ANGLE = "min_angle"
CONF_MAX_ANGLE = "max_angle"
CONF_ANGLE_OFFSET = "angle_offset"
CONF_MIN_VALUE = "min_value"
CONF_MAX_VALUE = "max_value"
CONF_CROP_X = "crop_x"
CONF_CROP_Y = "crop_y"
CONF_CROP_W = "crop_w"
CONF_CROP_H = "crop_h"
CONF_VALIDATOR = "validator"
CONF_PAUSED = "paused"
CONF_AUTO_CONTRAST = "auto_contrast"
CONF_CONTRAST = "contrast"
CONF_DEBUG = "debug"

CONF_NEEDLE_TYPE = "needle_type"
CONF_TYPE_DARK = "DARK"
CONF_TYPE_LIGHT = "LIGHT"

CONF_ALGORITHM = "algorithm"
CONF_ALGO_LEGACY = "legacy"
CONF_ALGO_RADIAL = "radial_profile"
CONF_ALGO_HOUGH = "hough_transform"
CONF_ALGO_TEMPLATE = "template_match"
CONF_ALGO_AUTO = "auto"

DIAL_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.string, # String ID for logs
    cv.Optional(CONF_NEEDLE_TYPE, default=CONF_TYPE_DARK): cv.enum({
        CONF_TYPE_DARK: analog_reader_ns.enum("NEEDLE_TYPE_DARK"),
        CONF_TYPE_LIGHT: analog_reader_ns.enum("NEEDLE_TYPE_LIGHT"),
    }),
    cv.Optional(CONF_ALGORITHM, default=CONF_ALGO_LEGACY): cv.one_of(
        CONF_ALGO_LEGACY, CONF_ALGO_RADIAL, CONF_ALGO_HOUGH, CONF_ALGO_TEMPLATE, CONF_ALGO_AUTO, lower=True
    ),
    cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    cv.Optional(CONF_CROP_X, default=0): cv.int_,
    cv.Optional(CONF_CROP_Y, default=0): cv.int_,
    cv.Optional(CONF_CROP_W, default=64): cv.int_,
    cv.Optional(CONF_CROP_H, default=64): cv.int_,
    cv.Optional(CONF_MIN_ANGLE, default=0): cv.float_,
    cv.Optional(CONF_MAX_ANGLE, default=360): cv.float_,
    cv.Optional(CONF_ANGLE_OFFSET, default=0): cv.float_, 
    cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
    cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
    cv.Optional(CONF_AUTO_CONTRAST, default=False): cv.boolean,
    cv.Optional(CONF_CONTRAST, default=1.0): cv.float_,
})


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(AnalogReader),
    cv.Optional(CONF_VALIDATOR): cv.use_id(value_validator.ValueValidator),
    cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
    cv.Optional(CONF_VALUE_SENSOR): sensor.sensor_schema(),
    cv.Required(CONF_DIALS): cv.ensure_list(DIAL_SCHEMA),
    cv.Optional(CONF_PAUSED, default=False): cv.boolean,
    cv.Optional(CONF_DEBUG, default=False): cv.boolean,
}).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    cg.add_library(
        "analog_reader_lib",
        None,
        [
            "analog_reader.cpp",
            "detect_legacy.cpp",
            "multi_algorithm.cpp",
            "camera_coordinator.cpp",
            "flashlight_coordinator.cpp", 
        ],
    )
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    # Validator
    if CONF_VALIDATOR in config:
        v = await cg.get_variable(config[CONF_VALIDATOR])
        cg.add(var.set_validator(v))

    cam = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(cam))

    if config[CONF_PAUSED]:
        cg.add(var.set_pause_processing(True))
        
    if config[CONF_DEBUG]:
        cg.add(var.set_debug(True))

    if CONF_VALUE_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_VALUE_SENSOR])
        cg.add(var.set_value_sensor(sens))

    for dial in config[CONF_DIALS]:
        # Struct construction
        # We need to map Py config to C++ struct DialConfig
        # cpp: struct DialConfig { string id; float scale; ... }
        # generated: var.add_dial({id, scale, ...})
        
        # We can't pass a dict directly to add_dial typically in codegen unless we struct init.
        # cg.StructInitializer
        
        s = cg.StructInitializer(
            analog_reader_ns.struct("DialConfig"),
            ("id", dial[CONF_ID]),
            ("needle_type", dial[CONF_NEEDLE_TYPE]),
            ("algorithm", dial[CONF_ALGORITHM]),
            ("scale", dial[CONF_SCALE]),
            ("crop_x", dial[CONF_CROP_X]),
            ("crop_y", dial[CONF_CROP_Y]),
            ("crop_w", dial[CONF_CROP_W]),
            ("crop_h", dial[CONF_CROP_H]),
            ("min_angle", dial[CONF_MIN_ANGLE]),
            ("max_angle", dial[CONF_MAX_ANGLE]),
            ("angle_offset", dial[CONF_ANGLE_OFFSET]),
            ("min_value", dial[CONF_MIN_VALUE]),
            ("max_value", dial[CONF_MAX_VALUE]),
            ("auto_contrast", dial[CONF_AUTO_CONTRAST]),
            ("contrast", dial[CONF_CONTRAST]),
        )
        cg.add(var.add_dial(s))

    # Get camera resolution from substitutions
    width, height = 640, 480  # Defaults
    substitutions = CORE.config.get("substitutions", {})
    if substitutions.get("camera_resolution"):
        res = substitutions["camera_resolution"]
        if 'x' in res:
            width, height = map(int, res.split('x'))
    
    pixel_format = substitutions.get("camera_pixel_format", "RGB888")
    cg.add(var.set_camera_image_format(width, height, pixel_format))
