#!/usr/bin/env python3
"""Verify tflm_operators.h against the pinned esp-tflite-micro library header.

Catches operator table / library drift, such as the REDUCE_MIN regression where
a library-supported op was wrongly marked ``TFLM_OP_UNAVAILABLE``:

  - an op marked AVAILABLE whose ``Add*`` method does not exist in the library,
  - an op marked AVAILABLE whose kernel has no ``Register_*`` definition,
  - an op marked UNAVAILABLE that the library actually supports,
  - a library builtin op missing from the table entirely.

Usage:
    python script/verify_tflm_operators.py                 # auto-detect cache
    python script/verify_tflm_operators.py --header PATH   # explicit header
    python script/verify_tflm_operators.py --legacy        # also check legacy table

Exit codes:
    0 - table matches the library, or the library header is unavailable and the
        run was skipped (not failed)
    1 - mismatches found (or an invalid --header path)
    2 - library header not found, with --strict (nothing was verified)
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent
COMPONENT_DIR = ROOT / "components" / "tflite_micro_helper"
if str(COMPONENT_DIR) not in sys.path:
    sys.path.insert(0, str(COMPONENT_DIR))

import op_manifest  # noqa: E402

# Pinned dependency - matches esp32.add_idf_component(ref="1.3.7") in __init__.py
PACKAGE = "espressif__esp-tflite-micro_1.3.7"

# BuiltinOperator suffix -> Add* method, from methods whose body calls AddBuiltin()
_ADD_METHOD_RE = re.compile(
    r"TfLiteStatus\s+(Add\w+)\s*\((?:[^()]|\([^()]*\))*\)\s*\{[^{}]*"
    r"AddBuiltin\(\s*BuiltinOperator_(\w+)",
    re.DOTALL,
)
# Kernel registration definitions in kernels/*.cc
_REGISTER_DEF_RE = re.compile(r"TFLMRegistration\*?\s+Register_(\w+)\s*\(\s*\)\s*\{")
# Table entries (comment lines excluded by op_manifest; method name kept here)
_TABLE_METHOD_RE = re.compile(r"TFLM_OP_AVAILABLE\((\w+),\s*(\w+)\)")


def find_library_headers() -> list[Path]:
    """Locate micro_mutable_op_resolver.h in the ESP-IDF managed component cache."""
    cache = ROOT / ".esphome" / ".espressif"
    hits = []
    for service in sorted(cache.glob("service_*")):
        for component in sorted(service.glob(f"{PACKAGE}_*")):
            candidate = (
                component
                / "tensorflow"
                / "lite"
                / "micro"
                / "micro_mutable_op_resolver.h"
            )
            if candidate.exists():
                hits.append(candidate)
    return hits


def parse_library_ops(header: Path) -> dict[str, str]:
    """Map BuiltinOperator suffix -> Add* method from the library resolver header."""
    text = header.read_text(encoding="utf-8")
    return {m.group(2): m.group(1) for m in _ADD_METHOD_RE.finditer(text)}


def parse_kernel_registrations(header: Path) -> set[str]:
    """Collect Register_<OP> names that have definitions in the library kernels dir."""
    kernels = header.parent / "kernels"
    defined: set[str] = set()
    if not kernels.is_dir():
        return defined
    for source in kernels.rglob("*.cc"):
        text = source.read_text(encoding="utf-8", errors="replace")
        for m in _REGISTER_DEF_RE.finditer(text):
            defined.add(m.group(1))
    return defined


def verify_table(
    table: Path, library_ops: dict[str, str], kernels: set[str] | None
) -> list[str]:
    """Return a list of table/library mismatches (empty when consistent).

    ``kernels`` may be None when the kernels directory is not available next to
    the library header; in that case the link-safety check is skipped.
    """
    available, unavailable = op_manifest.parse_op_table(table)
    text = "\n".join(
        line
        for line in table.read_text(encoding="utf-8").splitlines()
        if not line.lstrip().startswith("//")
    )
    methods = dict(_TABLE_METHOD_RE.findall(text))
    issues = []
    for op in sorted(available):
        method = methods.get(op, "?")
        if op not in library_ops:
            issues.append(
                f"{table.name}: {op} marked AVAILABLE ({method}()) but the library "
                f"has no Add* method for it"
            )
        elif library_ops[op] != method:
            issues.append(
                f"{table.name}: {op} method mismatch - table uses {method}(), "
                f"library has {library_ops[op]}()"
            )
        elif kernels is not None and op not in kernels:
            issues.append(
                f"{table.name}: {op} marked AVAILABLE but no Register_{op}() "
                f"kernel definition found (link error at build time)"
            )
    for op in sorted(unavailable):
        if op in library_ops:
            issues.append(
                f"{table.name}: {op} marked UNAVAILABLE but the library supports "
                f"it via {library_ops[op]}()"
            )
    for op in sorted(set(library_ops) - available - unavailable):
        issues.append(
            f"{table.name}: {op} has library method {library_ops[op]}() but no "
            f"table entry"
        )
    return issues


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Verify tflm_operators.h against esp-tflite-micro."
    )
    parser.add_argument("--header", help="explicit path to micro_mutable_op_resolver.h")
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="also verify the legacy_meter_reader_tflite table",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit with code 2 when the library header is not available "
        "(default: exit 0 so pre-commit stays non-blocking on a clean checkout)",
    )
    args = parser.parse_args(argv)

    if args.header:
        header_path = Path(args.header)
        if not header_path.exists():
            print(f"[ERROR] --header file not found: {header_path}")
            return 1
        headers = [header_path]
    else:
        headers = find_library_headers()
        if not headers:
            print(
                "[SKIP] esp-tflite-micro 1.3.7 header not found in the ESP-IDF "
                "managed cache (.esphome/.espressif)."
            )
            print("       Run once after a successful build, or pass --header PATH.")
            return 2 if args.strict else 0
    header = headers[0]
    if len(headers) > 1:
        print(f"[INFO] Multiple cached versions found; using {header}")

    library_ops = parse_library_ops(header)
    kernels_dir = header.parent / "kernels"
    kernels = parse_kernel_registrations(header) if kernels_dir.is_dir() else None
    if kernels is None:
        print(
            "[INFO] kernels/ dir not found next to the header - skipping the "
            "kernel link-safety check"
        )
    print(f"[INFO] Library: {header}")
    print(
        f"[INFO] Builtin ops with Add method: {len(library_ops)}; "
        f"kernels with Register_* definition: "
        f"{len(kernels) if kernels is not None else 'n/a'}"
    )

    issues = verify_table(COMPONENT_DIR / "tflm_operators.h", library_ops, kernels)
    if args.legacy:
        issues += verify_table(
            ROOT / "components" / "legacy_meter_reader_tflite" / "tflm_operators.h",
            library_ops,
            kernels,
        )

    if not issues:
        print("[PASS] tflm_operators.h matches esp-tflite-micro 1.3.7")
        return 0
    print(f"[FAIL] {len(issues)} mismatch(es) between the table and the library:")
    for issue in issues:
        print(f"  - {issue}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
