#!/usr/bin/env python3
"""Custom lint checks for ESPHome AI Component.

Adapted from ESPHome's ci-custom.py to match project structure.
"""

import argparse
import codecs
import collections
import fnmatch
import functools
import os.path
from pathlib import Path
import re
import sys
import time

import colorama

sys.path.append(str(Path(__file__).parent))

from helpers import filter_changed, git_ls_files, print_error_for_file, styled


def find_all(a_str, sub):
    """Yield (line, column) tuples for each occurrence of sub in a_str."""
    if not sub or sub not in a_str:
        # Optimization: If str is not in whole text, then do not try
        # on each line
        return
    for i, line in enumerate(a_str.split("\n")):
        column = 0
        while True:
            column = line.find(sub, column)
            if column == -1:
                break
            yield i, column
            column += len(sub)


file_types = (
    ".h",
    ".c",
    ".cpp",
    ".tcc",
    ".yaml",
    ".yml",
    ".ini",
    ".txt",
    ".ico",
    ".svg",
    ".png",
    ".py",
    ".html",
    ".js",
    ".md",
    ".sh",
    ".css",
    ".conf",
    ".cfg",
    ".woff",
    ".woff2",
    "",
)
cpp_include = ("*.h", "*.c", "*.cpp", "*.tcc")
py_include = ("*.py",)
ignore_types = (
    ".ico",
    ".png",
    ".woff",
    ".woff2",
    "",
    ".ttf",
    ".otf",
    ".pcf",
    ".apng",
    ".gif",
    ".webp",
    ".bin",
    ".wav",
    ".md",
    ".md_old",
    ".html",
    ".js",
    ".css",
    ".json",
    ".txt",
    ".rst",
    ".svg",
    ".jpg",
    ".jpeg",
    ".tflite",
)

LINT_FILE_CHECKS = []
LINT_CONTENT_CHECKS = []
LINT_POST_CHECKS = []
EXECUTABLE_BIT: dict[str, int] = {}

errors: collections.defaultdict[Path, list] = collections.defaultdict(list)


def add_errors(fname: Path, errs: list[tuple[int, int, str] | None]) -> None:
    """Add errors to the global error collection."""
    if not isinstance(errs, list):
        errs = [errs]
    for err in errs:
        if err is None:
            continue
        try:
            lineno, col, msg = err
        except ValueError:
            lineno = 1
            col = 1
            msg = err
        if not isinstance(msg, str):
            raise ValueError("Error is not instance of string!")
        if not isinstance(lineno, int):
            raise ValueError("Line number is not an int!")
        if not isinstance(col, int):
            raise ValueError("Column number is not an int!")
        errors[fname].append((lineno, col, msg))


def run_check(lint_obj, fname, *args):
    """Run a single lint check on a file."""
    include = lint_obj["include"]
    exclude = lint_obj["exclude"]
    func = lint_obj["func"]
    if include is not None:
        for incl in include:
            if fnmatch.fnmatch(fname, incl):
                break
        else:
            return None
    for excl in exclude:
        if fnmatch.fnmatch(fname, excl):
            return None
    return func(*args)


def run_checks(lints, fname, *args):
    """Run all applicable lint checks on a file."""
    for lint in lints:
        start = time.process_time()
        try:
            add_errors(fname, run_check(lint, fname, *args))
        except Exception:
            print(f"Check {lint['func'].__name__} on file {fname} failed:")
            raise
        duration = time.process_time() - start
        lint.setdefault("durations", []).append(duration)


def _add_check(checks, func, include=None, exclude=None):
    """Register a lint check function."""
    checks.append(
        {
            "include": include,
            "exclude": exclude or [],
            "func": func,
        }
    )


def lint_file_check(**kwargs):
    """Decorator to register a file-level lint check."""

    def decorator(func):
        _add_check(LINT_FILE_CHECKS, func, **kwargs)
        return func

    return decorator


def lint_content_check(**kwargs):
    """Decorator to register a content-level lint check."""

    def decorator(func):
        _add_check(LINT_CONTENT_CHECKS, func, **kwargs)
        return func

    return decorator


def lint_post_check(func):
    """Decorator to register a post-processing lint check."""
    _add_check(LINT_POST_CHECKS, func)
    return func


def lint_re_check(regex, **kwargs):
    """Decorator to register a regex-based lint check."""
    flags = kwargs.pop("flags", re.MULTILINE)
    prog = re.compile(regex, flags)
    decor = lint_content_check(**kwargs)

    def decorator(func):
        @functools.wraps(func)
        def new_func(fname, content):
            errs = []
            for match in prog.finditer(content):
                if "NOLINT" in match.group(0):
                    continue
                lineno = content.count("\n", 0, match.start()) + 1
                substr = content[: match.start()]
                col = len(substr) - substr.rfind("\n")
                err = func(fname, match)
                if err is None:
                    continue
                errs.append((lineno, col + 1, err))
            return errs

        return decor(new_func)

    return decorator


def lint_content_find_check(find, only_first=False, **kwargs):
    """Decorator to register a string-find-based lint check."""
    decor = lint_content_check(**kwargs)

    def decorator(func):
        @functools.wraps(func)
        def new_func(fname, content):
            find_ = find
            if callable(find):
                find_ = find(fname, content)
            errs = []
            for line, col in find_all(content, find_):
                err = func(fname, line, col, content)
                errs.append((line + 1, col + 1, err))
                if only_first:
                    break
            return errs

        return decor(new_func)

    return decorator


# ---------------------------------------------------------------------------
# LINT CHECKS
# ---------------------------------------------------------------------------


@lint_file_check(include=["*.ino"])
def lint_ino(fname):
    """Reject .ino files."""
    return "This file extension (.ino) is not allowed. Please use either .cpp or .h"


@lint_file_check(
    exclude=[f"*{f}" for f in file_types]
    + [
        ".clang-*",
        ".dockerignore",
        ".editorconfig",
        "*.gitignore",
        "LICENSE",
        "MANIFEST.in",
        "script/*",
    ]
)
def lint_ext_check(fname):
    """Reject unknown file extensions."""
    return (
        "This file extension is not a registered file type. If this is an error, please "
        "update the script/ci-custom.py script."
    )


@lint_file_check(
    exclude=[
        "**.sh",
        "script/*",
    ]
)
def lint_executable_bit(fname: Path) -> str | None:
    """Reject files with invalid executable bit (common Windows git issue)."""
    ex = EXECUTABLE_BIT.get(fname.as_posix(), 0)
    if ex != 0 and ex != 100644:
        return (
            f"File has invalid executable bit {ex}. If running from a windows machine please "
            "see disabling executable bit in git."
        )
    return None


@lint_content_find_check("\t", only_first=True)
def lint_tabs(fname, line, col, content):
    """Reject tab characters."""
    return "File contains tab character. Please convert tabs to spaces."


@lint_content_find_check("\r", only_first=True)
def lint_newline(fname, line, col, content):
    """Reject Windows CRLF newlines."""
    return "File contains Windows newline (CRLF). Please set your editor to Unix newline mode (LF)."


@lint_content_check(
    exclude=[
        "*.svg",
        "*.png",
        "*.ico",
        "*.woff",
        "*.woff2",
        "*.ttf",
        "*.otf",
        "*.gif",
        "*.webp",
        "*.bin",
    ]
)
def lint_end_newline(fname, content):
    """Require file to end with a newline."""
    if content and not content.endswith("\n"):
        return "File does not end with a newline, please add an empty line at the end of the file."
    return None


@lint_content_check()
def lint_ascii_only(fname, content):
    """Reject files containing non-ASCII characters."""
    for i, ch in enumerate(content):
        if ord(ch) > 127:
            # Find the line number
            lineno = content.count("\n", 0, i) + 1
            col = i - content.rfind("\n", 0, i)
            return (
                f"Non-ASCII character (U+{ord(ch):04X}: '{ch}') found at line {lineno}, column {col}. "
                "Please use ASCII-only characters in source files."
            )
    return None


CPP_RE_EOL = r".*?(?://.*?)?$"
PY_RE_EOL = r".*?(?:#.*?)?$"


def highlight(s):
    """Return a highlighted string for terminal output."""
    return f"\033[36m{s}\033[0m"


@lint_re_check(
    r"^#define\s+([a-zA-Z0-9_]+)\s+(0b[10]+|0x[0-9a-fA-F]+|\d+)\s*?(?:\/\/.*?)?$",
    include=cpp_include,
    exclude=[
        "components/tflite_micro_helper/model_handler.h",
    ],
)
def lint_no_defines(fname, match):
    """Reject #define for integer constants (use constexpr instead)."""
    s = highlight(f"static constexpr uint8_t {match.group(1)} = {match.group(2)};")
    return (
        "#define macros for integer constants are not allowed, please use "
        f"{s} style instead (replace uint8_t with the appropriate "
        "datatype). See also Google style guide."
    )


@lint_re_check(r"^\s*delay\((\d+)\);" + CPP_RE_EOL, include=cpp_include)
def lint_no_long_delays(fname, match):
    """Reject delay() calls >50ms."""
    duration_ms = int(match.group(1))
    if duration_ms < 50:
        return None
    return (
        f"{highlight(match.group(0).strip())} - long calls to delay() are not allowed "
        "in ESPHome because everything executes in one thread. Calling delay() will "
        "block the main thread and slow down ESPHome.\n"
        "If there's no way to work around the delay() and it doesn't execute often, please add "
        "a '// NOLINT' comment to the line."
    )


# Functions from Arduino framework that are forbidden to use directly
ARDUINO_FORBIDDEN = [
    "digitalWrite",
    "digitalRead",
    "pinMode",
    "shiftOut",
    "shiftIn",
    "radians",
    "degrees",
    "interrupts",
    "noInterrupts",
    "lowByte",
    "highByte",
    "bitRead",
    "bitSet",
    "bitClear",
    "bitWrite",
    "bit",
    "analogRead",
    "analogWrite",
    "pulseIn",
    "pulseInLong",
    "tone",
]
ARDUINO_FORBIDDEN_RE = r"[^\w\d](" + r"|".join(ARDUINO_FORBIDDEN) + r")\(.*"


@lint_re_check(
    ARDUINO_FORBIDDEN_RE,
    include=cpp_include,
    exclude=[
        "components/mqtt/custom_mqtt_device.h",
    ],
)
def lint_no_arduino_framework_functions(fname, match):
    """Reject direct use of Arduino framework functions."""
    nolint = highlight("// NOLINT")
    return (
        f"The function {highlight(match.group(1))} from the Arduino framework is forbidden to be "
        f"used directly in the codebase. Please use ESPHome's abstractions and equivalent "
        f"C++ instead.\n"
        f"\n"
        f"(If the function is strictly necessary, please add `{nolint}` to the end of the line)"
    )


@lint_re_check(
    r"[^\w\d]byte +[\w\d]+\s*=",
    include=cpp_include,
)
def lint_no_byte_datatype(fname, match):
    """Reject 'byte' datatype (use uint8_t instead)."""
    return (
        f"The datatype {highlight('byte')} is not allowed. "
        f"Please use {highlight('uint8_t')} instead."
    )


@lint_re_check(
    r"(?:std\s*::\s*string_view|#include\s*<string_view>)" + CPP_RE_EOL,
    include=cpp_include,
)
def lint_no_std_string_view(fname, match):
    """Reject std::string_view on embedded targets."""
    return (
        f"{highlight('std::string_view')} is not allowed in embedded ESPHome code. "
        f"It pulls in significant STL template machinery that bloats flash on "
        f"resource-constrained embedded targets.\n"
        f"Please use {highlight('const char *')} or {highlight('StringRef')} for simple cases.\n"
        f"(If strictly necessary, add `{highlight('// NOLINT')}` to the end of the line)"
    )


def lint_inclusive_language(fname, match):
    """Warn about non-inclusive language."""
    # From https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?id=49decddd39e5f6132ccd7d9fdc3d7c470b0061bb
    return (
        "Avoid the use of whitelist/blacklist/slave.\n"
        "Recommended replacements for 'master / slave' are:\n"
        "    '{primary,main} / {secondary,replica,subordinate}\n"
        "    '{initiator,requester} / {target,responder}'\n"
        "    '{controller,host} / {device,worker,proxy}'\n"
        "    'leader / follower'\n"
        "    'director / performer'\n"
        "\n"
        "Recommended replacements for 'blacklist/whitelist' are:\n"
        "    'denylist / allowlist'\n"
        "    'blocklist / passlist'"
    )


lint_re_check(
    r"(whitelist|blacklist|slave)" + PY_RE_EOL,
    include=py_include,
    exclude=["script/ci-custom.py"],
    flags=re.IGNORECASE | re.MULTILINE,
)(lint_inclusive_language)


lint_re_check(
    r"(whitelist|blacklist|slave)" + CPP_RE_EOL,
    include=cpp_include,
    flags=re.IGNORECASE | re.MULTILINE,
)(lint_inclusive_language)


@lint_re_check(r"[\t\r\f\v ]+$")
def lint_trailing_whitespace(fname, match):
    """Reject trailing whitespace."""
    return "Trailing whitespace detected"


# Heap-allocating helpers that cause fragmentation on long-running embedded devices.
# These return std::string and should be replaced with stack-based alternatives.
HEAP_ALLOCATING_HELPERS = {
    "base64_encode": "base64_encode_to() with a pre-allocated buffer",
    "format_bin": "format_bin_to() with a stack buffer",
    "format_hex": "format_hex_to() with a stack buffer",
    "format_hex_pretty": "format_hex_pretty_to() with a stack buffer",
    "format_mac_address_pretty": "format_mac_addr_upper() with a stack buffer",
    "get_mac_address": "get_mac_address_into_buffer() with a stack buffer",
    "get_mac_address_pretty": "get_mac_address_pretty_into_buffer() with a stack buffer",
    "str_lower_case": "manual tolower() with a stack buffer",
    "str_sanitize": "str_sanitize_to() with a stack buffer",
    "str_truncate": "removal (function is unused)",
    "str_until": "manual strchr()/find() with a StringRef or stack buffer",
    "str_upper_case": "removal (function is unused)",
    "str_snake_case": "removal (function is unused)",
    "str_sprintf": "snprintf() with a stack buffer",
    "str_snprintf": "snprintf() with a stack buffer",
    "value_accuracy_to_string": "value_accuracy_to_buf() with a stack buffer",
}


@lint_re_check(
    r"[^\w]("
    r"base64_encode(?!_)|"
    r"format_bin(?!_)|"
    r"format_hex(?!_)|"
    r"format_hex_pretty(?!_)|"
    r"format_mac_address_pretty|"
    r"get_mac_address_pretty(?!_)|"
    r"get_mac_address(?!_)|"
    r"str_lower_case|"
    r"str_sanitize(?!_)|"
    r"str_truncate|"
    r"str_until|"
    r"str_upper_case|"
    r"str_snake_case|"
    r"str_sprintf|"
    r"str_snprintf|"
    r"value_accuracy_to_string"
    r")\s*\(" + CPP_RE_EOL,
    include=cpp_include,
    exclude=[
        # Vendored third-party library
        "components/http_request/httplib.h",
    ],
)
def lint_no_heap_allocating_helpers(fname, match):
    """Reject heap-allocating string helpers in embedded code."""
    func = match.group(1)
    replacement = HEAP_ALLOCATING_HELPERS.get(func, "a stack-based alternative")
    return (
        f"{highlight(func + '()')} allocates heap memory. On long-running embedded devices, "
        f"repeated heap allocations fragment memory over time. Even infrequent allocations "
        f"become time bombs - the heap eventually cannot satisfy requests even with free "
        f"memory available.\n"
        f"Please use {replacement} instead.\n"
        f"(If strictly necessary, add `// NOLINT` to the end of the line)"
    )


@lint_re_check(
    r"[^\w](v?sprintf)\s*\(" + CPP_RE_EOL,
    include=cpp_include,
)
def lint_no_sprintf(fname, match):
    """Reject sprintf/vsprintf (use snprintf instead)."""
    func = match.group(1)
    safe_func = func.replace("sprintf", "snprintf")
    return (
        f"{highlight(func + '()')} is not allowed. It has no buffer size limit "
        f"and can cause buffer overflows.\n"
        f"Please use {highlight(safe_func + '(buf, sizeof(buf), fmt, ...)')} instead.\n"
        f"(If strictly necessary, add `// NOLINT` to the end of the line)"
    )


@lint_re_check(
    r"(?:(?<![*&.\w>:])to_string|std\s*::\s*to_string)\s*\(" + CPP_RE_EOL,
    include=cpp_include,
    exclude=[
        # Vendored library
        "components/http_request/httplib.h",
        # Inline template use in header - legitimate use
        "components/esp32_camera_utils/camera_window_control.h",
        # Debug logging uses to_string for sensor values
        "components/meter_reader_tflite/flashlight_coordinator.cpp",
        "components/meter_reader_tflite/meter_reader_tflite.cpp",
        "components/ssocr_reader/ssocr_reader.cpp",
        "components/value_validator/value_validator.cpp",
    ],
)
def lint_no_std_to_string(fname, match):
    """Reject std::to_string() in embedded code (heap alloc)."""
    return (
        f"{highlight('std::to_string()')} (including unqualified {highlight('to_string()')}) "
        f"allocates heap memory. On long-running embedded devices, repeated heap allocations "
        f"fragment memory over time.\n"
        f"\n"
        f"Please use stack-based alternatives:\n"
        f"  - {highlight('snprintf()')} with a stack buffer\n"
        f"  - Dedicated integer formatters from helpers.h\n"
        f"\n"
        f"(If strictly necessary, add `{highlight('// NOLINT')}` to the end of the line)"
    )


@lint_re_check(
    r"[^\w]((?:std::)?v?[fs]?scanf)\s*\(" + CPP_RE_EOL,
    include=cpp_include,
)
def lint_no_scanf(fname, match):
    """Reject scanf family (bloats flash)."""
    func = match.group(1)
    return (
        f"{highlight(func + '()')} is not allowed. The scanf family "
        f"pulls in unnecessary flash overhead.\n"
        f"Please use alternatives:\n"
        f"  - {highlight('parse_number<T>(str)')} for parsing integers/floats\n"
        f"  - {highlight('strtol()/strtof()')} for C-style number parsing\n"
        f"  - Manual parsing for simple fixed formats\n"
        f"(If strictly necessary, add `// NOLINT` to the end of the line)"
    )


@lint_re_check(
    r"[^\w]std\s*::\s*bind\s*\(" + CPP_RE_EOL,
    include=cpp_include,
)
def lint_no_std_bind(fname, match):
    """Reject std::bind (use lambdas instead)."""
    return (
        f"{highlight('std::bind()')} is not allowed in new code. "
        f"Lambdas are clearer, produce smaller binaries, and are more likely to fit within "
        f"the {highlight('std::function')} small-buffer optimization (avoiding heap allocation).\n"
        f"Please use a lambda instead.\n"
        f"  Before: {highlight('std::bind(&Class::method, this, std::placeholders::_1)')}\n"
        f"  After:  {highlight('[this](auto arg) { this->method(arg); }')}\n"
        f"(If strictly necessary, add `// NOLINT` to the end of the line)"
    )


LOG_MULTILINE_RE = re.compile(r"ESP_LOG\w+\s*\(.*?;", re.DOTALL)
LOG_BAD_CONTINUATION_RE = re.compile(r'\\n(?:[^ \\"\r\n\t]|"\s*\n\s*"[^ \\])')
LOG_PERCENT_S_CONTINUATION_RE = re.compile(r'\\n(?:%s|"\s*\n\s*"%s)')


@lint_content_check(include=cpp_include)
def lint_log_multiline_continuation(fname, content):
    """Check that multi-line log continuations start with whitespace."""
    errs = []
    for log_match in LOG_MULTILINE_RE.finditer(content):
        log_text = log_match.group(0)
        for bad_match in LOG_BAD_CONTINUATION_RE.finditer(log_text):
            # %s may expand to a whitespace prefix at runtime, skip those
            if LOG_PERCENT_S_CONTINUATION_RE.match(log_text, bad_match.start()):
                continue
            # Calculate line number from position in full content
            abs_pos = log_match.start() + bad_match.start()
            lineno = content.count("\n", 0, abs_pos) + 1
            col = abs_pos - content.rfind("\n", 0, abs_pos)
            errs.append(
                (
                    lineno,
                    col,
                    "Multi-line log message has a continuation line that does "
                    "not start with a space. The log viewer uses leading "
                    "whitespace to detect continuation lines and re-add the "
                    f"log tag prefix (e.g. {highlight('[C][component:042]:')}).\n"
                    "Either start the continuation with a space/indent, or "
                    "split into separate ESP_LOG* calls.",
                )
            )
    return errs


@lint_content_find_check(
    "ESP_LOG",
    include=["*.h", "*.tcc"],
    exclude=[
        # Debug helper headers - ESP_LOG in inline functions is acceptable
        "components/esp32_camera_utils/crop_zone_handler.h",
        "components/esp32_camera_utils/debug_utils.h",
        "components/esp32_camera_utils/rotator.h",
        "components/tflite_micro_helper/debug_utils.h",
        "components/tflite_micro_helper/op_resolver.h",
        "components/binary_sensor/binary_sensor.h",
        "components/button/button.h",
        "components/climate/climate.h",
        "components/cover/cover.h",
        "components/datetime/date_entity.h",
        "components/datetime/time_entity.h",
        "components/datetime/datetime_entity.h",
        "components/display/display.h",
        "components/event/event.h",
        "components/fan/fan.h",
        "components/i2c/i2c.h",
        "components/lock/lock.h",
        "components/mqtt/mqtt_component.h",
        "components/number/number.h",
        "components/one_wire/one_wire.h",
        "components/output/binary_output.h",
        "components/output/float_output.h",
        "components/nextion/nextion_base.h",
        "components/select/select.h",
        "components/sensor/sensor.h",
        "components/spi/spi.h",
        "components/stepper/stepper.h",
        "components/switch/switch.h",
        "components/text/text.h",
        "components/text_sensor/text_sensor.h",
        "components/valve/valve.h",
        "components/esp32_camera_utils/esp32_camera_utils.h",
        "esphome/core/component.h",
        "esphome/core/gpio.h",
        "esphome/core/log_const_en.h",
        "esphome/core/log.h",
        "tests/custom.h",
    ],
)
def lint_log_in_header(fname, line, col, content):
    """Reject ESP_LOG in header files."""
    return (
        "Found reference to ESP_LOG in header file. Using ESP_LOG* in header files "
        "is currently not possible - please move the definition to a source file (.cpp)"
    )


@lint_content_check(
    include=["*.h"],
    exclude=[
        # X-macro file - included multiple times with different macro definitions.
        # Adding #pragma once would break the X-macro include pattern.
        "components/tflite_micro_helper/tflm_operators.h",
    ],
)
def lint_pragma_once(fname, content):
    """Require #pragma once in header files."""
    if "#pragma once" not in content:
        return (
            "Header file contains no 'pragma once' header guard. Please add a "
            "'#pragma once' line at the top of the file."
        )
    return None


@lint_content_find_check('"esphome.h"', include=cpp_include, exclude=["tests/custom.h"])
def lint_esphome_h(fname, line, col, content):
    """Reject reference to 'esphome.h' (auto-generated)."""
    return (
        "File contains reference to 'esphome.h' - This file is "
        "auto-generated and should only be used for *custom* "
        "components. Please replace with references to the direct files."
    )


IDF_CONVERSION_FORBIDDEN = {
    "ARDUINO_ARCH_ESP32": "USE_ESP32",
    "ARDUINO_ARCH_ESP8266": "USE_ESP8266",
    "pgm_read_byte": "progmem_read_byte",
    "ICACHE_RAM_ATTR": "IRAM_ATTR",
    "esphome/core/esphal.h": "esphome/core/hal.h",
}
IDF_CONVERSION_FORBIDDEN_RE = r"(" + r"|".join(IDF_CONVERSION_FORBIDDEN) + r").*"


@lint_re_check(
    IDF_CONVERSION_FORBIDDEN_RE,
    include=cpp_include,
)
def lint_no_removed_in_idf_conversions(fname, match):
    """Reject outdated Arduino macros/conversions."""
    replacement = IDF_CONVERSION_FORBIDDEN[match.group(1)]
    return (
        f"The macro {highlight(match.group(1))} can no longer be used directly. "
        f"Please use {highlight(replacement)} instead."
    )


RAW_PIN_ACCESS_RE = (
    r"^\s(pinMode|digitalWrite|digitalRead)\((.*)->get_pin\(\),\s*([^)]+).*\)"
)


@lint_re_check(RAW_PIN_ACCESS_RE, include=cpp_include)
def lint_no_raw_pin_access(fname, match):
    """Reject raw pin access functions (use ESPHome abstractions)."""
    func = match.group(1)
    pin = match.group(2)
    mode = match.group(3)
    new_func = {
        "pinMode": "pin_mode",
        "digitalWrite": "digital_write",
        "digitalRead": "digital_read",
    }[func]
    new_code = highlight(f"{pin}->{new_func}({mode})")
    return f"Don't use raw {func} calls. Instead, use the `->{new_func}` function: {new_code}"


def relative_cpp_search_text(fname: Path, content) -> str:
    """Find absolute includes within the same component."""
    parts = fname.parts
    if len(parts) < 3 or parts[0] != "components":
        return None
    integration = parts[1]
    return f'#include "components/{integration}'


@lint_content_find_check(relative_cpp_search_text, include=["components/*.cpp"])
def lint_relative_cpp_import(fname, line, col, content):
    """Reject absolute includes within a component (must use relative)."""
    parts = fname.parts
    integration = parts[1] if len(parts) > 1 else ""
    return (
        "Component contains absolute import - Components must always use "
        "relative imports.\n"
        "Change:\n"
        f'  #include "components/{integration}/{integration}.h"\n'
        "to:\n"
        f'  #include "{integration}.h"\n\n'
    )


def relative_py_search_text(fname: Path, content: str) -> str:
    """Find absolute Python imports within the same component."""
    parts = fname.parts
    if len(parts) < 3 or parts[0] != "components":
        return None
    integration = parts[1]
    return f"components.{integration}"


@lint_content_find_check(
    relative_py_search_text,
    include=["components/*.py"],
    exclude=[
        "components/__init__.py",
    ],
)
def lint_relative_py_import(fname: Path, line, col, content):
    """Reject absolute Python imports within a component (must use relative)."""
    import_line = content.splitlines()[line]
    abspath = import_line[col:].split(" ")[0]
    current = str(fname).removesuffix(".py").replace(os.path.sep, ".")
    replacement = abspath
    # Simplify: suggest using . prefix for same-package imports
    if current.startswith("components."):
        pkg = current[len("components.") :].split(".")
        target = abspath.split(".")
        # Find common prefix and suggest relative
        i = 0
        while i < len(pkg) and i < len(target) and pkg[i] == target[i]:
            i += 1
        if i > 0:
            up = len(pkg) - i
            down = target[i:]
            replacement = "." * up + ".".join(down)
    newline = import_line.replace(abspath, replacement)
    return (
        "Component contains absolute import - Components must always use "
        "relative imports within the integration.\n"
        "Change:\n"
        f"    {import_line}\n"
        "to:\n"
        f"    {newline}\n"
    )


@lint_content_check(
    include=["components/*.h", "components/*.cpp", "components/*.tcc"],
    exclude=[
        "components/http_request/httplib.h",
        # X-macro file - included with TFLM_OP macro defined. No namespace possible.
        "components/tflite_micro_helper/tflm_operators.h",
        # Empty placeholder file
        "components/esp32_camera_utils/rotator.cpp",
    ],
)
def lint_namespace(fname: Path, content: str) -> str | None:
    """Verify C++ files have a namespace matching the integration name."""
    parts = fname.parts
    if len(parts) < 2 or parts[0] != "components":
        return None
    expected_name = parts[1]
    # Check for both old style and C++17 nested namespace syntax
    search_old = f"namespace {expected_name}"
    search_new = f"namespace esphome::{expected_name}"
    if search_old in content or search_new in content:
        return None
    return (
        "Invalid namespace found in C++ file. All integration C++ files should put all "
        "functions in a separate namespace that matches the integration's name. "
        f"Please make sure the file contains {highlight(search_old)} or {highlight(search_new)}"
    )


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------


def main():
    """Run all lint checks on the repository."""
    colorama.init()

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "files", nargs="*", default=[], help="files to be processed (regex on path)"
    )
    parser.add_argument(
        "-c", "--changed", action="store_true", help="Only run on changed files"
    )
    parser.add_argument(
        "--print-slowest", action="store_true", help="Print the slowest checks"
    )
    args = parser.parse_args()

    EXECUTABLE_BIT.update(git_ls_files())
    files = list(EXECUTABLE_BIT.keys())
    # Match against re
    file_name_re = re.compile("|".join(args.files))
    files = [p for p in files if file_name_re.search(p)]

    if args.changed:
        files = filter_changed(files)

    files.sort()

    for fname in files:
        fname = Path(fname)
        # Frozen component - exclude from all lint checks
        if "legacy_meter_reader_tflite" in fname.parts:
            continue
        run_checks(LINT_FILE_CHECKS, fname, fname)
        if fname.suffix in ignore_types:
            continue
        try:
            with codecs.open(fname, "r", encoding="utf-8") as f_handle:
                content = f_handle.read()
        except UnicodeDecodeError:
            add_errors(
                fname,
                "File is not readable as UTF-8. Please set your editor to UTF-8 mode.",
            )
            continue
        run_checks(LINT_CONTENT_CHECKS, fname, fname, content)

    run_checks(LINT_POST_CHECKS, Path("POST"))

    for f, errs in sorted(errors.items()):
        bold = functools.partial(styled, colorama.Style.BRIGHT)
        bold_red = functools.partial(styled, (colorama.Style.BRIGHT, colorama.Fore.RED))
        err_str = (
            f"{bold(f'{f}:{lineno}:{col}:')} {bold_red('lint:')} {msg}\n"
            for lineno, col, msg in errs
        )
        print_error_for_file(f, "\n".join(err_str))

    if args.print_slowest:
        lint_times = []
        for lint in LINT_FILE_CHECKS + LINT_CONTENT_CHECKS + LINT_POST_CHECKS:
            durations = lint.get("durations", [])
            lint_times.append((sum(durations), len(durations), lint["func"].__name__))
        lint_times.sort(key=lambda x: -x[0])
        for i in range(min(len(lint_times), 10)):
            dur, invocations, name = lint_times[i]
            print(f" - '{name}' took {dur:.2f}s total (ran on {invocations} files)")
        print(f"Total time measured: {sum(x[0] for x in lint_times):.2f}s")

    return len(errors)


if __name__ == "__main__":
    sys.exit(main())
