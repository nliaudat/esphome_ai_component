#!/usr/bin/env python3
"""Pre-commit test runner for ESPHome AI Component.

Runs all lint/formatter checks from .pre-commit-config.yaml without requiring
pre-commit installation. Reports per-check pass/fail status.

Usage:
    python script/run_lint.py           # Check only (read-only)
    python script/run_lint.py --fix     # Auto-fix where possible
    python script/run_lint.py --changed # Only changed files
    python script/run_lint.py --ci      # CI mode (exit on first failure)
"""

import argparse
import pathlib
import shutil
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
EXIT_CODE = 0


def highlight(s):
    return f"\033[36m{s}\033[0m"


def color(status, text):
    if status == "PASS":
        return f"\033[32m{text}\033[0m"
    if status == "FAIL":
        return f"\033[31m{text}\033[0m"
    if status == "SKIP":
        return f"\033[33m{text}\033[0m"
    return text


def tool_available(name):
    return shutil.which(name) is not None


def run_check(name, args, cwd=None, fix=False):
    """Run a check and return (pass, output)."""
    if cwd is None:
        cwd = ROOT
    try:
        r = subprocess.run(
            args, capture_output=True, text=True, cwd=cwd, timeout=120, check=False
        )
        passed = r.returncode == 0
        return passed, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return False, "", "TIMEOUT (120s)"
    except FileNotFoundError:
        return False, "", "Tool not found"


def fix_crlf_trailing_ws():
    """Fix CRLF->LF and trailing whitespace in all tracked files."""
    count = 0
    for ext in (
        "*.h",
        "*.cpp",
        "*.tcc",
        "*.py",
        "*.c",
        "*.yaml",
        "*.yml",
        "*.md",
        "*.json",
        "*.cfg",
        "*.conf",
        "*.txt",
        "*.js",
        "*.html",
        "*.css",
    ):
        for f in ROOT.rglob(ext):
            # Skip hidden dirs and venv
            if any(p.startswith(".") and p != "." for p in f.parts):
                continue
            if "venv" in f.parts or "__pycache__" in f.parts:
                continue
            try:
                data = f.read_bytes()
                # CRLF -> LF
                if b"\r\n" in data or data.count(b"\r") > 0:
                    text = data.decode("utf-8")
                    text = text.replace("\r\n", "\n").replace("\r", "\n")
                    lines = [line_.rstrip() for line_ in text.split("\n")]
                    text = "\n".join(lines).strip("\n") + "\n"
                    f.write_text(text, encoding="utf-8")
                    count += 1
            except (UnicodeDecodeError, ValueError):
                pass  # binary file
    return count


def main():
    global EXIT_CODE
    parser = argparse.ArgumentParser(description="Run all pre-commit lint checks")
    parser.add_argument(
        "--fix", action="store_true", help="Auto-fix issues where possible"
    )
    parser.add_argument(
        "--changed", action="store_true", help="Only check changed files"
    )
    parser.add_argument(
        "--ci", action="store_true", help="CI mode: exit on first failure"
    )
    args = parser.parse_args()

    mode = "--fix" if args.fix else ""
    changed_flag = "--changed" if args.changed else ""
    ci_mode = args.ci

    print(highlight("=" * 70))
    print(highlight("  ESPHome AI Component --- Pre-commit Lint Runner"))
    if args.fix:
        print(highlight("  Mode: FIX (auto-correcting)"))
    if args.changed:
        print(highlight("  Mode: Only changed files"))
    print(highlight("=" * 70))
    print()

    checks = []

    # --- Ci-Custom ---
    checks.append(
        {
            "name": "ci-custom (CRLF, tabs, trailing WS, EOF, ASCII, #define, etc.)",
            "cmd": [sys.executable, "script/ci-custom.py"],
            "extra_args": [mode, changed_flag],
        }
    )

    # --- clang-format ---
    if tool_available("clang-format"):
        if args.fix:
            checks.append(
                {
                    "name": "clang-format (format C/C++)",
                    "cmd": ["clang-format", "-i", "-style=file"],
                    "glob": [
                        "components/**/*.h",
                        "components/**/*.cpp",
                        "components/**/*.tcc",
                    ],
                    "batch_files": True,
                }
            )
        else:
            checks.append(
                {
                    "name": "clang-format (check C/C++)",
                    "cmd": ["clang-format", "--dry-run", "-Werror", "-style=file"],
                    "glob": [
                        "components/**/*.h",
                        "components/**/*.cpp",
                        "components/**/*.tcc",
                    ],
                    "batch_files": True,
                }
            )
    else:
        checks.append({"name": "clang-format (not installed)", "skip": True})

    # --- clang-tidy ---
    if tool_available("clang-tidy") and not args.changed:
        checks.append(
            {
                "name": "clang-tidy (static analysis --- manual only)",
                "cmd": ["echo", "Run manually: clang-tidy components/*/*.cpp"],
                "manual": True,
            }
        )

    # --- Ruff ---
    if tool_available("ruff"):
        fix_flag = ["--fix"] if args.fix else []
        checks.append(
            {
                "name": f"ruff lint{' (fix)' if args.fix else ''}",
                "cmd": ["ruff", "check", "."] + fix_flag,
            }
        )
        if not args.fix:
            checks.append(
                {
                    "name": "ruff format check",
                    "cmd": ["ruff", "format", "--check", "."],
                }
            )
        else:
            checks.append(
                {
                    "name": "ruff format",
                    "cmd": ["ruff", "format", "."],
                }
            )
    else:
        checks.append({"name": "ruff (not installed)", "skip": True})

    # --- Flake8 ---
    if tool_available("flake8"):
        checks.append(
            {
                "name": "flake8 (docstrings)",
                "cmd": ["flake8", "."],
            }
        )
    else:
        checks.append({"name": "flake8 (not installed)", "skip": True})

    # --- YamlLint ---
    if tool_available("yamllint"):
        checks.append(
            {
                "name": "yamllint",
                "cmd": ["yamllint", "."],
            }
        )
    else:
        checks.append({"name": "yamllint (not installed)", "skip": True})

    # --- CRLF / trailing whitespace fix (if --fix) ---
    if args.fix:
        start = time.time()
        print("[CRLF/trailing WS fix] ", end="", flush=True)
        n = fix_crlf_trailing_ws()
        elapsed = time.time() - start
        print(f"{color('PASS', 'PASSED')} --- fixed {n} files ({elapsed:.1f}s)")

    # --- Run checks ---
    total = 0
    passed = 0
    failed = 0
    skipped = 0

    for check in checks:
        total += 1
        name = check["name"]

        if check.get("skip"):
            print(f"[{name}] {color('SKIP', 'SKIPPED')} (tool not available)")
            skipped += 1
            continue

        if check.get("manual"):
            print(f"[{name}] {color('SKIP', 'MANUAL')} --- {check['cmd'][2]}")
            skipped += 1
            continue

        if check.get("batch_files"):
            # Run clang-format on all matching files at once
            files = []
            for g in check["glob"]:
                files.extend(sorted(ROOT.glob(g)))
            # Exclude legacy/frozen directories from clang-format checks
            exclude_dirs = {"legacy_meter_reader_tflite"}
            files = [f for f in files if not any(p in exclude_dirs for p in f.parts)]
            if not files:
                print(f"[{name}] {color('SKIP', 'SKIPPED')} (no files match)")
                skipped += 1
                continue
            cmd = check["cmd"] + [str(f) for f in files]
        else:
            cmd = check["cmd"]

        start = time.time()
        ok, stdout, stderr = run_check(name, cmd)
        elapsed = time.time() - start

        if ok:
            print(f"[{name}] {color('PASS', 'PASSED')} ({elapsed:.1f}s)")
            passed += 1
        else:
            print(f"[{name}] {color('FAIL', 'FAILED')} ({elapsed:.1f}s)")
            if stdout:
                # Show first 20 lines of context
                lines = stdout.strip().split("\n")
                for line in lines[:20]:
                    print(f"  {line}")
                if len(lines) > 20:
                    print(f"  ... ({len(lines) - 20} more lines)")
            if stderr:
                for line in stderr.strip().split("\n")[:10]:
                    print(f"  stderr: {line}")
            failed += 1
            EXIT_CODE = 1
            if ci_mode:
                print(f"\n{color('FAIL', 'CI MODE: stopping on first failure')}")
                break

    # --- Summary ---
    print()
    print(highlight("=" * 70))
    print(
        highlight(
            f"  Results: {passed}/{total} passed, {failed} failed, {skipped} skipped"
        )
    )
    if failed > 0:
        print(highlight(f"  {failed} check(s) FAILED --- see above for details"))
    else:
        print(highlight("  ALL CHECKS PASSED"))
    if EXIT_CODE != 0 and not args.fix:
        print(
            highlight(
                "  Tip: Run with --fix to auto-correct CRLF, trailing WS, ruff, clang-format"
            )
        )
    print(highlight("=" * 70))

    return EXIT_CODE


if __name__ == "__main__":
    sys.exit(main())
