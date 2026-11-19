import esphome.codegen as cg
from esphome.components import esp32_camera, sensor
import esphome.config_validation as cv

try:
    from esphome.components import value_validator
except ImportError:
    value_validator = None

try:
    from esphome.components import flash_light_controller
except ImportError:
    flash_light_controller = None

CONF_FLASH_LIGHT_CONTROLLER = "flash_light_controller"

from esphome.const import CONF_ID, CONF_RESOLUTION
import esphome.final_validate as fv
from esphome.types import ConfigType

DEPENDENCIES = ["esp32_camera"]
AUTO_LOAD = ["esp32_camera_utils"]

analog_gauge_reader_ns = cg.esphome_ns.namespace("analog_gauge_reader")
AnalogGaugeReader = analog_gauge_reader_ns.class_(
    "AnalogGaugeReader", cg.PollingComponent
)

CONF_CAMERA_ID = "camera_id"
CONF_DIALS = "dials"
CONF_VALUE_SENSOR = "value_sensor"
CONF_SCALE = "scale"
CONF_MIN_ANGLE = "min_angle"
CONF_MAX_ANGLE = "max_angle"
CONF_ANGLE_OFFSET = "angle_offset"
CONF_CLOCKWISE = "clockwise"
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
CONF_STACKED_DIGITS = "stacked_digits"

CONF_NEEDLE_TYPE = "needle_type"
CONF_TYPE_DARK = "DARK"
CONF_TYPE_LIGHT = "LIGHT"

CONF_ALGORITHM = "algorithm"
CONF_ALGO_LEGACY = "legacy"
CONF_ALGO_RADIAL = "radial_profile"
CONF_ALGO_HOUGH = "hough_transform"
CONF_ALGO_TEMPLATE = "template_match"
CONF_ALGO_AUTO = "auto"
CONF_VALUE = "value"
CONF_CONFIDENCE = "confidence"
CONF_ANGLE = "angle"
CONF_CALIBRATION = "calibration"
CONF_RAW = "raw"
CONF_MAPPED = "mapped"
CONF_MIN_SCAN_RADIUS = "min_scan_radius"
CONF_MAX_SCAN_RADIUS = "max_scan_radius"
CONF_DEADZONE_DIAMETER = "deadzone_diameter"
CONF_PROCESS_CHANNEL = "process_channel"
CONF_CHANNEL_GRAYSCALE = "GRAYSCALE"
CONF_CHANNEL_RED = "RED"
CONF_CHANNEL_GREEN = "GREEN"
CONF_CHANNEL_BLUE = "BLUE"

DIAL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.string,  # String ID for logs
        cv.Optional(CONF_NEEDLE_TYPE, default=CONF_TYPE_DARK): cv.enum(
            {
                CONF_TYPE_DARK: analog_gauge_reader_ns.enum("NEEDLE_TYPE_DARK"),
                CONF_TYPE_LIGHT: analog_gauge_reader_ns.enum("NEEDLE_TYPE_LIGHT"),
            }
        ),
        cv.Optional(CONF_PROCESS_CHANNEL, default=CONF_CHANNEL_GRAYSCALE): cv.enum(
            {
                CONF_CHANNEL_GRAYSCALE: analog_gauge_reader_ns.enum(
                    "PROCESS_CHANNEL_GRAYSCALE"
                ),
                CONF_CHANNEL_RED: analog_gauge_reader_ns.enum("PROCESS_CHANNEL_RED"),
                CONF_CHANNEL_GREEN: analog_gauge_reader_ns.enum(
                    "PROCESS_CHANNEL_GREEN"
                ),
                CONF_CHANNEL_BLUE: analog_gauge_reader_ns.enum("PROCESS_CHANNEL_BLUE"),
            }
        ),
        cv.Optional(CONF_ALGORITHM, default=CONF_ALGO_RADIAL): cv.one_of(
            CONF_ALGO_LEGACY,
            CONF_ALGO_RADIAL,
            CONF_ALGO_HOUGH,
            CONF_ALGO_TEMPLATE,
            CONF_ALGO_AUTO,
            lower=True,
        ),
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
        cv.Optional(CONF_CROP_X, default=0): cv.int_,
        cv.Optional(CONF_CROP_Y, default=0): cv.int_,
        cv.Optional(CONF_CROP_W, default=64): cv.int_,
        cv.Optional(CONF_CROP_H, default=64): cv.int_,
        cv.Optional(CONF_MIN_ANGLE, default=0): cv.float_,
        cv.Optional(CONF_MAX_ANGLE, default=360): cv.float_,
        cv.Optional(CONF_ANGLE_OFFSET, default=0): cv.float_,
        cv.Optional(CONF_CLOCKWISE, default=True): cv.boolean,
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
        cv.Optional(CONF_MIN_SCAN_RADIUS, default=0.3): cv.float_,
        cv.Optional(CONF_MAX_SCAN_RADIUS, default=0.9): cv.float_,
        cv.Optional(CONF_DEADZONE_DIAMETER, default=0.0): cv.float_,
        cv.Optional(CONF_AUTO_CONTRAST, default=True): cv.boolean,
        cv.Optional(CONF_CONTRAST, default=1.0): cv.float_,
        cv.Optional("target_color"): cv.hex_int,
        cv.Optional("color_tolerance", default=0.0): cv.float_,
        # Per-dial sensors
        cv.Optional(CONF_VALUE): sensor.sensor_schema(),
        cv.Optional(CONF_CONFIDENCE): sensor.sensor_schema(),
        cv.Optional(CONF_ANGLE): sensor.sensor_schema(),
        # Internal Calibration Mapping
        cv.Optional(CONF_CALIBRATION): cv.ensure_list(
            cv.Schema(
                {
                    cv.Required(CONF_RAW): cv.float_,
                    cv.Required(CONF_MAPPED): cv.float_,
                }
            )
        ),
    }
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AnalogGaugeReader),
        cv.Optional(CONF_VALIDATOR): cv.use_id(value_validator.ValueValidator)
        if value_validator
        else cv.string,
        cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
        cv.Optional(CONF_FLASH_LIGHT_CONTROLLER): cv.use_id(
            flash_light_controller.FlashLightController
        )
        if flash_light_controller
        else cv.string,
        cv.Optional(CONF_VALUE_SENSOR): sensor.sensor_schema(),
        cv.Required(CONF_DIALS): cv.ensure_list(DIAL_SCHEMA),
        cv.Optional(CONF_PAUSED, default=False): cv.boolean,
        cv.Optional(CONF_DEBUG, default=False): cv.boolean,
        cv.Optional(CONF_STACKED_DIGITS, default=False): cv.boolean,
    }
).extend(cv.polling_component_schema("60s"))

# Camera frame size and delivered format, resolved in final_validate where the
# referenced camera's own configuration is valid and known, and read back by
# to_code. Stored on the config rather than re-derived, so codegen never has to
# guess from substitutions.
_KEY_CAMERA_FRAME = "_analog_gauge_camera_frame"


def _resolution_dimensions(resolution: object) -> tuple[int, int]:
    """Map an esp32_camera resolution to its pixel dimensions.

    The numeric names in ``esp32_camera.FRAME_SIZES`` carry the pixel size; the
    aliases (QQVGA, VGA, ...) map to the same frame size.
    """
    name = str(resolution).upper()
    parts = name.split("X")
    if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
        return int(parts[0]), int(parts[1])

    try:
        frame_size = esp32_camera.FRAME_SIZES[name]
    except KeyError:
        raise cv.Invalid(f"Unknown camera resolution {resolution}") from None
    for numeric_name, candidate in esp32_camera.FRAME_SIZES.items():
        parts = numeric_name.split("X")
        # The enum values are codegen mock objects, one per attribute access, so
        # they are compared rendered rather than by identity.
        if len(parts) == 2 and parts[0].isdigit() and str(candidate) == str(frame_size):
            return int(parts[0]), int(parts[1])
    raise cv.Invalid(f"Unknown camera resolution {resolution}")


def _final_validate(config: ConfigType) -> ConfigType:
    """Resolve the frame size and delivered format from the referenced camera."""
    camera = fv.get_declaring_config(config[CONF_CAMERA_ID])
    width, height = _resolution_dimensions(camera[CONF_RESOLUTION])

    pixel_format = str(camera.get(esp32_camera.CONF_PIXEL_FORMAT, "JPEG"))
    # esp32_camera converts every non-JPEG frame to JPEG when jpeg_quality is
    # non-zero (USE_ESP32_CAMERA_JPEG_CONVERSION), so the listener receives JPEG
    # in that case even though the sensor does not.
    jpeg_quality = camera.get(esp32_camera.CONF_JPEG_QUALITY, 0)
    delivered = "JPEG" if pixel_format == "JPEG" or jpeg_quality != 0 else pixel_format

    # The needle detection runs on decoded 8-bit pixels; this component only
    # implements the JPEG decode path, so reject raw frames at config time rather
    # than silently doing nothing at runtime.
    if delivered != "JPEG":
        raise cv.Invalid(
            f"analog_gauge_reader needs JPEG frames, but the referenced camera "
            f"delivers {delivered}. Set pixel_format: JPEG, or set a jpeg_quality "
            f"so the camera converts the frame to JPEG.",
            path=[CONF_CAMERA_ID],
        )

    config[_KEY_CAMERA_FRAME] = (width, height, delivered)
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    cg.add_define("USE_ANALOG_GAUGE_READER")

    # Validator
    if CONF_VALIDATOR in config:
        v = await cg.get_variable(config[CONF_VALIDATOR])
        cg.add(var.set_validator(v))

    cam = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(cam))

    if CONF_FLASH_LIGHT_CONTROLLER in config and flash_light_controller:
        flash = await cg.get_variable(config[CONF_FLASH_LIGHT_CONTROLLER])
        cg.add(var.set_flash_light_controller(flash))

    if config[CONF_PAUSED]:
        cg.add(var.set_pause_processing(True))

    if config[CONF_DEBUG]:
        cg.add(var.set_debug(True))

    if config[CONF_STACKED_DIGITS]:
        cg.add(var.set_stacked_digits(True))

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
            analog_gauge_reader_ns.struct("DialConfig"),
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
            ("clockwise", dial[CONF_CLOCKWISE]),
            ("min_value", dial[CONF_MIN_VALUE]),
            ("max_value", dial[CONF_MAX_VALUE]),
            ("auto_contrast", dial[CONF_AUTO_CONTRAST]),
            ("contrast", dial[CONF_CONTRAST]),
            ("target_color", dial.get("target_color", 0)),
            ("use_color", "target_color" in dial),
            ("color_tolerance", dial["color_tolerance"]),
            ("process_channel", dial[CONF_PROCESS_CHANNEL]),
            ("min_scan_radius", dial[CONF_MIN_SCAN_RADIUS]),
            ("max_scan_radius", dial[CONF_MAX_SCAN_RADIUS]),
            ("deadzone_diameter", dial[CONF_DEADZONE_DIAMETER]),
            (
                "value_sensor",
                await sensor.new_sensor(dial[CONF_VALUE])
                if CONF_VALUE in dial
                else None,
            ),
            (
                "confidence_sensor",
                await sensor.new_sensor(dial[CONF_CONFIDENCE])
                if CONF_CONFIDENCE in dial
                else None,
            ),
            (
                "angle_sensor",
                await sensor.new_sensor(dial[CONF_ANGLE])
                if CONF_ANGLE in dial
                else None,
            ),
        )

        # Handle calibration mapping
        if CONF_CALIBRATION in dial:
            # Convert list of dicts to list of pairs for C++ vector<pair<float,float>>
            # Generate C++ initializer list string: {{r1, m1}, {r2, m2}, ...}
            cal_str = ", ".join(
                [
                    f"{{ {item[CONF_RAW]}f, {item[CONF_MAPPED]}f }}"
                    for item in dial[CONF_CALIBRATION]
                ]
            )
            s.args["calibration_mapping"] = cg.RawExpression(f"{{ {cal_str} }}")

        cg.add(var.add_dial(s))

    # Frame size and delivered format resolved in _final_validate from the
    # referenced camera's configuration.
    width, height, pixel_format = config[_KEY_CAMERA_FRAME]
    cg.add(var.set_camera_image_format(width, height, pixel_format))
