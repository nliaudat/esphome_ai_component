"""Component to manage TFLite Micro models and inference runtime.

Supports image (single-shot) and audio (streaming) model types,
with local file, github shorthand, and http URL model sources.
"""

from pathlib import Path
import zlib

import esphome.codegen as cg
from esphome.components import esp32
from esphome.const import CONF_DEBUG, CONF_ID, CONF_RAW_DATA_ID
from esphome.core import CORE, HexInt

from . import schema as _schema
from .model_metadata import (
    _compute_max_operators,
    _parse_audio_config,
    infer_model_config_from_filename,
    parse_model_txt_file,
)
from .model_source import resolve_model_source
from .schema import (
    CONF_FEATURES_STEP_SIZE,
    CONF_FEATURE_COUNT,
    CONF_INPUT_CHANNELS,
    CONF_INPUT_HEIGHT,
    CONF_INPUT_ORDER,
    CONF_INPUT_TYPE,
    CONF_INPUT_WIDTH,
    CONF_INVERT,
    CONF_MODEL_TYPE,
    CONF_NORMALIZE,
    CONF_OUTPUT_PROCESSING,
    CONF_PROBABILITY_CUTOFF,
    CONF_SCALE_FACTOR,
    CONF_SLIDING_WINDOW_SIZE,
    CONF_TENSOR_ARENA_SIZE,
)

CODEOWNERS = ["@nliaudat"]

DOMAIN = "tflite_micro_helper"

# Attributes consumed by the ESPHome config loader.
CONFIG_SCHEMA = _schema.CONFIG_SCHEMA
MULTI_CONF = _schema.MULTI_CONF
DEPENDENCIES = _schema.DEPENDENCIES


async def to_code(config):
    if CORE.target_platform == "esp32":
        cg.add_define("USE_TFLITE_MICRO_HELPER")
        model_type = config[CONF_MODEL_TYPE]
        model_path, model_data = resolve_model_source(config)
        var = cg.new_Pvariable(config[CONF_ID])
        rhs = [HexInt(x) for x in model_data]
        prog_arr = cg.progmem_array(config[CONF_RAW_DATA_ID], rhs)
        cg.add(var.set_model(prog_arr, len(model_data)))
        # Per-instance CRC32 - MULTI_CONF safe, each TFLiteMicroHelper carries its own expected checksum
        crc32_val = zlib.crc32(model_data) & 0xFFFFFFFF
        cg.add(var.set_expected_crc32(crc32_val))
        cg.add(var.set_model_type(model_type))
        cg.add_build_flag("-DTF_LITE_STATIC_MEMORY")
        cg.add_build_flag("-DTF_LITE_DISABLE_X86_NEON")
        cg.add_build_flag("-DESP_NN")
        cg.add_build_flag("-DOPTIMIZED_KERNEL=esp_nn")
        # E7: MAX_OPERATORS is a process-global compile-time macro. With MULTI_CONF,
        # emitting one flag per instance would let the last one win. Compute the max
        # across ALL configured models so every instance is safe. Idempotent: every
        # to_code() call for this DOMAIN emits the same value.
        cg.add_build_flag(f"-DMAX_OPERATORS={_compute_max_operators()}")
        if config.get(CONF_DEBUG, False):
            cg.add_define("DEBUG_TFLITE_MICRO_HELPER")
            cg.add(var.set_debug(True))
        if model_type == "image":
            await _configure_image_model(config, var, model_path, model_data)
        elif model_type == "audio":
            await _configure_audio_model(config, var, model_path, model_data)
        esp32.add_idf_component(name="espressif/esp-tflite-micro", ref="1.3.7")
        esp32.add_idf_component(name="espressif/esp-nn", ref="1.2.3")
        if model_type == "audio":
            esp32.add_idf_component(
                name="esphome/esp-micro-speech-features", ref="1.2.3"
            )


async def _configure_image_model(entry, var, model_path, model_data):
    model_filename = Path(str(model_path).replace("\\", "/")).name
    auto_config = parse_model_txt_file(model_path)
    if auto_config:
        print(f"  Auto-detected image config from '{model_filename}.txt':")
        for k, v in auto_config.items():
            print(f"    {k}: {v}")
    else:
        auto_config = infer_model_config_from_filename(model_filename)
        print(
            f"  No .txt file found, using filename heuristics for '{model_filename}':"
        )
        for k, v in auto_config.items():
            print(f"    {k}: {v}")
    yaml_overrides = {}
    for yaml_key, auto_key in [
        (CONF_INPUT_TYPE, "input_type"),
        (CONF_INPUT_CHANNELS, "input_channels"),
        (CONF_INPUT_WIDTH, "input_width"),
        (CONF_INPUT_HEIGHT, "input_height"),
        (CONF_OUTPUT_PROCESSING, "output_processing"),
        (CONF_SCALE_FACTOR, "scale_factor"),
        (CONF_INPUT_ORDER, "input_order"),
        (CONF_NORMALIZE, "normalize"),
        (CONF_INVERT, "invert"),
    ]:
        if yaml_key in entry:
            yaml_overrides[auto_key] = entry[yaml_key]
    if yaml_overrides:
        auto_config.update(yaml_overrides)
    cg.add(var.set_input_type(auto_config.get("input_type", "uint8")))
    cg.add(var.set_input_channels(auto_config.get("input_channels", 3)))
    cg.add(var.set_input_width(auto_config.get("input_width", 32)))
    cg.add(var.set_input_height(auto_config.get("input_height", 20)))
    cg.add(
        var.set_output_processing(auto_config.get("output_processing", "direct_class"))
    )
    cg.add(var.set_scale_factor(auto_config.get("scale_factor", 1.0)))
    cg.add(var.set_input_order(auto_config.get("input_order", "RGB")))
    cg.add(var.set_normalize(auto_config.get("normalize", False)))
    cg.add(var.set_invert(auto_config.get("invert", False)))
    if CONF_TENSOR_ARENA_SIZE in entry:
        cg.add(var.set_tensor_arena_size(entry[CONF_TENSOR_ARENA_SIZE]))
    elif "tensor_arena_size" in auto_config:
        cg.add(var.set_tensor_arena_size(auto_config["tensor_arena_size"]))


async def _configure_audio_model(entry, var, model_path, model_data):
    cg.add_define("USE_TFLITE_STREAMING")
    auto_config = _parse_audio_config(model_path) or {}
    if CONF_TENSOR_ARENA_SIZE in entry:
        cg.add(var.set_tensor_arena_size(entry[CONF_TENSOR_ARENA_SIZE]))
    elif "tensor_arena_size" in auto_config:
        cg.add(var.set_tensor_arena_size(auto_config["tensor_arena_size"]))
    if CONF_PROBABILITY_CUTOFF in entry:
        cg.add(var.set_probability_cutoff(entry[CONF_PROBABILITY_CUTOFF]))
    elif "probability_cutoff" in auto_config:
        cg.add(var.set_probability_cutoff(auto_config["probability_cutoff"]))
    if CONF_SLIDING_WINDOW_SIZE in entry:
        cg.add(var.set_sliding_window_size(entry[CONF_SLIDING_WINDOW_SIZE]))
    elif "sliding_window_size" in auto_config:
        cg.add(var.set_sliding_window_size(auto_config["sliding_window_size"]))
    if CONF_FEATURES_STEP_SIZE in entry:
        cg.add(var.set_features_step_size(entry[CONF_FEATURES_STEP_SIZE]))
    elif "features_step_size" in auto_config:
        cg.add(var.set_features_step_size(auto_config["features_step_size"]))
    if CONF_FEATURE_COUNT in entry:
        cg.add(var.set_feature_count(entry[CONF_FEATURE_COUNT]))
    elif "feature_count" in auto_config:
        cg.add(var.set_feature_count(auto_config["feature_count"]))
