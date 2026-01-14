import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@nliaudat"]
DEPENDENCIES = []

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
        cg.add(var.set_debug(conf["debug"]))
