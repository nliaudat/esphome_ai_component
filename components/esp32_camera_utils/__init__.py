import logging
import re

import esphome.codegen as cg
from esphome.components import esp32, esp32_camera, sensor
from esphome.components.const import CONF_WINDOW
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_FORCE_UPDATE,
    CONF_HEIGHT,
    CONF_ICON,
    CONF_ID,
    CONF_INTERNAL,
    CONF_NAME,
    CONF_OFFSET_X,
    CONF_OFFSET_Y,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_WIDTH,
)
from esphome.core import CORE
import esphome.final_validate as fv

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["esp32"]

# sensor: esp32_camera_utils.h unconditionally includes sensor/sensor.h (debug memory sensors).
# globals: crop_zone_handler.h (via image_processor.h) includes globals/globals_component.h.
AUTO_LOAD = ["sensor", "globals"]

esp32_camera_utils_ns = cg.esphome_ns.namespace("esp32_camera_utils")
Esp32CameraUtils = esp32_camera_utils_ns.class_("Esp32CameraUtils", cg.Component)

CONF_CAMERA_WINDOW = "camera_window"
CONF_CAMERA_ID = "camera_id"
CONF_ROTATION = "rotation"

# Window coordinates are in the pixels of the esp32_camera `resolution`, not the
# full sensor, so they can be handed to the native esp32_camera window API.
WINDOW_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_OFFSET_X): cv.int_range(min=0),
        cv.Required(CONF_OFFSET_Y): cv.int_range(min=0),
        cv.Required(CONF_WIDTH): cv.int_range(min=1),
        cv.Required(CONF_HEIGHT): cv.int_range(min=1),
    }
)

CONF_SCALER = "scaler"
CONF_CROPPER = "cropper"

# Rotation options mapping
ROTATION_OPTIONS = {
    "0": 0,
    "90": 90,
    "180": 180,
    "270": 270,
}

CONF_DEBUG_MEMORY = "debug_memory"


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Esp32CameraUtils),
            # Prefer esp32_camera.window. camera_window is kept as a compatibility
            # alias and is applied through the same native window API.
            cv.Optional(CONF_CAMERA_WINDOW): WINDOW_SCHEMA,
            cv.Optional(CONF_WINDOW): WINDOW_SCHEMA,
            cv.Optional(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
            cv.Optional("debug", default=False): cv.boolean,
            # Allow arbitrary rotation (float), but also string for backward compat "90"
            cv.Optional(CONF_ROTATION, default=0.0): cv.float_,
            cv.Optional("enable_rotation", default=False): cv.boolean,
            cv.Optional(CONF_SCALER): cv.Schema(
                {
                    cv.Optional(CONF_WIDTH): cv.int_,
                    cv.Optional(CONF_HEIGHT): cv.int_,
                }
            ),
            cv.Optional(CONF_CROPPER): cv.Schema(
                {
                    cv.Optional(CONF_WIDTH): cv.int_,
                    cv.Optional(CONF_HEIGHT): cv.int_,
                    cv.Optional(CONF_OFFSET_X, default=0): cv.int_,
                    cv.Optional(CONF_OFFSET_Y, default=0): cv.int_,
                }
            ),
            cv.Optional("enable_scaler", default=True): cv.boolean,
            cv.Optional("enable_cropper", default=True): cv.boolean,
            cv.Optional("enable_drawing", default=True): cv.boolean,
            cv.Optional(CONF_DEBUG_MEMORY, default=False): cv.boolean,
            cv.Optional("camera_buffer_size_sensor"): cv.use_id(sensor.Sensor),
            cv.Optional("camera_free_psram_sensor"): cv.use_id(sensor.Sensor),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.require_esphome_version(2026, 11, 0),
    cv.has_at_most_one_key(CONF_WINDOW, CONF_CAMERA_WINDOW),
)


def _get_camera_config(source, camera_id=None):
    """Return the validated esp32_camera config matching camera_id, if any."""
    camera_conf = source.get("esp32_camera")
    if camera_conf is None:
        return None
    configs = camera_conf if isinstance(camera_conf, list) else [camera_conf]
    if not configs:
        return None
    if camera_id is None:
        return configs[0]
    for conf in configs:
        if str(conf.get(CONF_ID)) == str(camera_id):
            return conf
    return None


def _frame_dimensions(camera_conf):
    """Return the (width, height) of the camera frame, or None when unknown.

    `resolution` is a validated esp32_camera enum member, so its dimensions are
    read back from the member name (``ESP32_CAMERA_SIZE_640X480``).
    """
    if camera_conf is None:
        return None
    resolution = camera_conf.get(esp32_camera.CONF_RESOLUTION)
    if resolution is None:
        return None
    match = re.search(r"(\d+)X(\d+)", str(resolution))
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def _check_window_fits(config):
    """Reject a window that the frame would silently clamp.

    The native esp32_camera window is measured in `resolution` pixels and is
    clamped to the frame, so a window reaching past the frame is cropped
    differently than configured.
    """
    camera_conf = _get_camera_config(fv.full_config.get(), config.get(CONF_CAMERA_ID))
    window = config.get(CONF_WINDOW)
    if window is None:
        window = config.get(CONF_CAMERA_WINDOW)
    if window is None and camera_conf is not None:
        window = camera_conf.get(CONF_WINDOW)
    if window is None:
        return

    frame = _frame_dimensions(camera_conf)
    if frame is None:
        return

    frame_width, frame_height = frame
    right = window[CONF_OFFSET_X] + window[CONF_WIDTH]
    bottom = window[CONF_OFFSET_Y] + window[CONF_HEIGHT]
    if right <= frame_width and bottom <= frame_height:
        return

    raise cv.Invalid(
        f"Window at ({window[CONF_OFFSET_X]},{window[CONF_OFFSET_Y]}) sized "
        f"{window[CONF_WIDTH]}x{window[CONF_HEIGHT]} reaches ({right},{bottom}), which "
        f"is outside the {frame_width}x{frame_height} camera frame. Windows are "
        f"measured in esp32_camera.resolution pixels and are clamped to the frame, "
        f"which silently changes the crop. Raise esp32_camera.resolution to at least "
        f"{right}x{bottom}; the full sensor frame (1600x1200 for the OV2640) keeps "
        f"window coordinates in sensor pixels."
    )


def _final_validate(config):
    camera_conf = _get_camera_config(fv.full_config.get(), config.get(CONF_CAMERA_ID))
    if (
        camera_conf is not None
        and CONF_WINDOW in camera_conf
        and (CONF_WINDOW in config or CONF_CAMERA_WINDOW in config)
    ):
        raise cv.Invalid(
            "Set the camera window in esp32_camera.window, not in "
            "esp32_camera_utils; both would be applied at startup."
        )

    _check_window_fits(config)


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_ESP32_CAMERA_UTILS")
    # This component delegates window control to the native esp32_camera API.
    cg.add_define("USE_ESP32_CAMERA_WINDOW")

    esp32.add_idf_component(name="espressif/esp_new_jpeg", ref="1.0.2")

    cg.add_build_flag("-DUSE_ESP32_CAMERA_CONV")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    camera_id = config.get(CONF_CAMERA_ID)
    camera_conf = _get_camera_config(CORE.config, camera_id)
    if camera_id is None and camera_conf is not None:
        camera_id = camera_conf.get(CONF_ID)

    cam = None
    if camera_id is not None:
        cam = await cg.get_variable(camera_id)
        cg.add(var.set_camera(cam))

    # The native key lives on esp32_camera and is already applied by its codegen;
    # read it here so process_zone() knows the window offsets. camera_window is
    # kept as a compatibility alias and applied through the same native API.
    window = config.get(CONF_WINDOW)
    if window is None:
        window = config.get(CONF_CAMERA_WINDOW)
    if window is None and camera_conf is not None:
        window = camera_conf.get(CONF_WINDOW)

    if window is not None:
        cg.add(
            var.set_camera_window_config(
                window[CONF_OFFSET_X],
                window[CONF_OFFSET_Y],
                window[CONF_WIDTH],
                window[CONF_HEIGHT],
            )
        )

        if CONF_CAMERA_WINDOW in config or CONF_WINDOW in config:
            if cam is not None:
                cg.add(
                    cam.set_initial_window(
                        window[CONF_OFFSET_X],
                        window[CONF_OFFSET_Y],
                        window[CONF_WIDTH],
                        window[CONF_HEIGHT],
                    )
                )
            else:
                _LOGGER.warning(
                    "A window is set in esp32_camera_utils but no esp32_camera was "
                    "found; the window will not be applied."
                )

    # Set image rotation
    cg.add(var.set_rotation(config[CONF_ROTATION]))

    # Enable modular features
    if config.get("enable_rotation", False) or config[CONF_ROTATION] != 0:
        cg.add_define("USE_CAMERA_ROTATOR")

    if config.get("enable_scaler", True):
        cg.add_define("USE_CAMERA_SCALER")
        if CONF_SCALER in config:
            conf = config[CONF_SCALER]
            if CONF_WIDTH in conf and CONF_HEIGHT in conf:
                cg.add(var.set_scaler_config(conf[CONF_WIDTH], conf[CONF_HEIGHT]))

    if config.get("enable_cropper", True):
        cg.add_define("USE_CAMERA_CROPPER")
        if CONF_CROPPER in config:
            conf = config[CONF_CROPPER]
            if CONF_WIDTH in conf and CONF_HEIGHT in conf:
                cg.add(
                    var.set_cropper_config(
                        conf[CONF_WIDTH],
                        conf[CONF_HEIGHT],
                        conf[CONF_OFFSET_X],
                        conf[CONF_OFFSET_Y],
                    )
                )

    if config.get("enable_drawing", True):
        cg.add_define("USE_CAMERA_DRAWING")

    if config.get("debug", False):
        cg.add_define("DEBUG_ESP32_CAMERA_UTILS")
        cg.add(var.set_debug(True))

    if "web_server" in CORE.config:
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
                CONF_ICON: icon,
                CONF_FORCE_UPDATE: False,
                CONF_UNIT_OF_MEASUREMENT: unit,
            }

            sens = await sensor.new_sensor(sens_conf)
            cg.add(sens.set_accuracy_decimals(accuracy_decimals))
            return sens

        # Buffer Size
        s = await create_sensor("camera_buffer_size", "B", 0)
        cg.add(var.set_camera_buffer_size_sensor(s))

        # Free PSRAM
        s = await create_sensor("camera_free_psram", "B", 0)
        cg.add(var.set_camera_free_psram_sensor(s))

        # Min free PSRAM since boot (fragmentation / long-run growth watermark)
        s = await create_sensor("camera_min_free_psram", "B", 0)
        cg.add(var.set_camera_min_free_psram_sensor(s))

        # Min free internal RAM since boot (fragmentation / long-run growth watermark)
        s = await create_sensor("camera_min_free_internal", "B", 0)
        cg.add(var.set_camera_min_free_internal_sensor(s))
