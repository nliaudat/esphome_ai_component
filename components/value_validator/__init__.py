import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import sensor, text_sensor

CODEOWNERS = ["@nliaudat"]
DEPENDENCIES = []
AUTO_LOAD = ["sensor", "text_sensor"]

value_validator_ns = cg.esphome_ns.namespace("value_validator")
ValueValidator = value_validator_ns.class_("ValueValidator", cg.Component)

CONF_ALLOW_NEGATIVE_RATES = "allow_negative_rates"
CONF_MAX_ABSOLUTE_DIFF = "max_absolute_diff"
CONF_MAX_RATE_CHANGE = "max_rate_change"
CONF_ENABLE_SMART_VALIDATION = "enable_smart_validation"
CONF_SMART_VALIDATION_WINDOW = "smart_validation_window"
CONF_HIGH_CONFIDENCE_THRESHOLD = "high_confidence_threshold"
CONF_MAX_HISTORY_SIZE = "max_history_size"
CONF_PER_DIGIT_CONFIDENCE_THRESHOLD = "per_digit_confidence_threshold"
CONF_STRICT_CONFIDENCE_CHECK = "strict_confidence_check"
CONF_MAX_CONSECUTIVE_REJECTIONS = "max_consecutive_rejections"
CONF_SMALL_NEGATIVE_TOLERANCE = "small_negative_tolerance"
CONF_PERSIST_STATE = "persist_state"
CONF_REJECTION_COUNT_SENSOR = "rejection_count_sensor"
CONF_RAW_READING_SENSOR = "raw_reading_sensor"
CONF_VALIDATOR_STATE_SENSOR = "validator_state_sensor"

CONFIG_SCHEMA = cv.All(cv.ensure_list(cv.Schema({
    cv.GenerateID(): cv.declare_id(ValueValidator),
    cv.Optional(CONF_ALLOW_NEGATIVE_RATES, default=False): cv.boolean,
    cv.Optional(CONF_MAX_ABSOLUTE_DIFF, default=100): cv.positive_int,
    cv.Optional(CONF_MAX_RATE_CHANGE, default=0.15): cv.float_range(min=0, max=10),
    cv.Optional(CONF_ENABLE_SMART_VALIDATION, default=True): cv.boolean,
    cv.Optional(CONF_SMART_VALIDATION_WINDOW, default=5): cv.positive_int,
    cv.Optional(CONF_HIGH_CONFIDENCE_THRESHOLD, default=0.90): cv.percentage,
    cv.Optional(CONF_MAX_HISTORY_SIZE, default="50kB"): cv.validate_bytes,
    cv.Optional(CONF_PER_DIGIT_CONFIDENCE_THRESHOLD, default=0.85): cv.percentage,
    cv.Optional(CONF_STRICT_CONFIDENCE_CHECK, default=False): cv.boolean,
    cv.Optional(CONF_MAX_CONSECUTIVE_REJECTIONS, default=10): cv.positive_int,
    cv.Optional(CONF_SMALL_NEGATIVE_TOLERANCE, default=5): cv.positive_int,
    cv.Optional(CONF_PERSIST_STATE, default=False): cv.boolean,
    cv.Optional(CONF_REJECTION_COUNT_SENSOR): sensor.sensor_schema(
        accuracy_decimals=0,
        icon="mdi:counter",
    ),
    cv.Optional(CONF_RAW_READING_SENSOR): sensor.sensor_schema(
        accuracy_decimals=0,
        icon="mdi:eye",
    ),
    cv.Optional(CONF_VALIDATOR_STATE_SENSOR): text_sensor.text_sensor_schema(
        icon="mdi:state-machine",
    ),
    cv.Optional("debug", default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)))

async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        
        # Construct config struct
        cg.add(var.set_allow_negative_rates(conf[CONF_ALLOW_NEGATIVE_RATES]))
        cg.add(var.set_max_absolute_diff(conf[CONF_MAX_ABSOLUTE_DIFF]))
        cg.add(var.set_max_rate_change(conf[CONF_MAX_RATE_CHANGE]))
        cg.add(var.set_enable_smart_validation(conf[CONF_ENABLE_SMART_VALIDATION]))
        cg.add(var.set_smart_validation_window(conf[CONF_SMART_VALIDATION_WINDOW]))
        cg.add(var.set_high_confidence_threshold(conf[CONF_HIGH_CONFIDENCE_THRESHOLD]))
        cg.add(var.set_max_history_size_bytes(conf[CONF_MAX_HISTORY_SIZE]))
        cg.add(var.set_per_digit_confidence_threshold(conf[CONF_PER_DIGIT_CONFIDENCE_THRESHOLD]))
        cg.add(var.set_strict_confidence_check(conf[CONF_STRICT_CONFIDENCE_CHECK]))
        cg.add(var.set_max_consecutive_rejections(conf[CONF_MAX_CONSECUTIVE_REJECTIONS]))
        cg.add(var.set_small_negative_tolerance(conf[CONF_SMALL_NEGATIVE_TOLERANCE]))
        cg.add(var.set_persist_state(conf[CONF_PERSIST_STATE]))
        cg.add(var.set_debug(conf["debug"]))
        
        # Optional diagnostic sensors
        if CONF_REJECTION_COUNT_SENSOR in conf:
            sens = await sensor.new_sensor(conf[CONF_REJECTION_COUNT_SENSOR])
            cg.add(var.set_rejection_count_sensor(sens))
        if CONF_RAW_READING_SENSOR in conf:
            sens = await sensor.new_sensor(conf[CONF_RAW_READING_SENSOR])
            cg.add(var.set_raw_reading_sensor(sens))
        if CONF_VALIDATOR_STATE_SENSOR in conf:
            sens = await text_sensor.new_text_sensor(conf[CONF_VALIDATOR_STATE_SENSOR])
            cg.add(var.set_validator_state_sensor(sens))

