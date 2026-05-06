#!/usr/bin/env python3
"""
Regenerate all .txt model report files from .tflite files.

Scans the models/ directory for .tflite files and runs check_tflite_model.py
on each to regenerate the corresponding .txt report.

Usage:
    python tools/regenerate_txt_reports.py              # from project root
    python tools/regenerate_txt_reports.py --force       # force regenerate all
    python tools/regenerate_txt_reports.py --dry-run     # show what would be done
    python tools/regenerate_txt_reports.py --verbose     # show check_tflite_model.py output
"""

import argparse
import os
import subprocess
import sys
from datetime import datetime

# Paths relative to this script's location
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
MODELS_DIR = os.path.join(PROJECT_ROOT, "models")
CHECK_SCRIPT = os.path.join(SCRIPT_DIR, "check_tflite_model.py")


def find_tflite_files(models_dir):
    """Find all .tflite files in the models directory."""
    tflite_files = []
    if not os.path.isdir(models_dir):
        print(f"[ERROR] Models directory not found: {models_dir}")
        return tflite_files

    for fname in sorted(os.listdir(models_dir)):
        if fname.endswith(".tflite"):
            tflite_path = os.path.join(models_dir, fname)
            tflite_files.append(tflite_path)

    return tflite_files


def get_txt_path(tflite_path):
    """Get the corresponding .txt path for a .tflite file."""
    base = os.path.splitext(tflite_path)[0]
    return base + ".txt"


def needs_regeneration(tflite_path, txt_path):
    """Check if the .txt file needs to be regenerated."""
    if not os.path.exists(txt_path):
        return True

    tflite_mtime = os.path.getmtime(tflite_path)
    txt_mtime = os.path.getmtime(txt_path)

    return tflite_mtime > txt_mtime


def regenerate_report(tflite_path, txt_path, verbose=False):
    """Run check_tflite_model.py to regenerate the .txt report."""
    cmd = [
        sys.executable,
        CHECK_SCRIPT,
        tflite_path,
        "--output_file", txt_path,
        "--verbose",
    ]

    try:
        # Set PYTHONIOENCODING=utf-8 to handle emoji characters in output
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        result = subprocess.run(
            cmd,
            capture_output=False,  # Let output flow to terminal directly
            timeout=120,  # 2 minute timeout per model
            env=env,
        )

        if result.returncode != 0:
            return False, f"check_tflite_model.py exited with code {result.returncode}"

        return True, None

    except subprocess.TimeoutExpired:
        return False, "Timed out after 120 seconds"
    except FileNotFoundError:
        return False, f"check_tflite_model.py not found at {CHECK_SCRIPT}"
    except Exception as e:
        return False, str(e)


def main():
    parser = argparse.ArgumentParser(
        description="Regenerate all .txt model report files from .tflite files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force regeneration even if .txt is newer than .tflite",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be done without actually running",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show check_tflite_model.py output for each model",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="Only regenerate a specific model (filename without path, e.g. 'digit_recognizer_v27_10cls_RGB.tflite')",
    )

    args = parser.parse_args()

    # Find all .tflite files
    tflite_files = find_tflite_files(MODELS_DIR)

    if not tflite_files:
        print("[ERROR] No .tflite files found in models/")
        sys.exit(1)

    # Filter by specific model if requested
    if args.model:
        model_path = os.path.join(MODELS_DIR, args.model)
        if model_path in tflite_files:
            tflite_files = [model_path]
        else:
            print(f"[ERROR] Model not found: {args.model}")
            print(f"   Available models:")
            for f in tflite_files:
                print(f"     - {os.path.basename(f)}")
            sys.exit(1)

    print(f"[*] Found {len(tflite_files)} .tflite file(s) in models/")
    print()

    # Check if check_tflite_model.py exists
    if not os.path.exists(CHECK_SCRIPT):
        print(f"[ERROR] check_tflite_model.py not found at: {CHECK_SCRIPT}")
        sys.exit(1)

    stats = {"regenerated": 0, "skipped": 0, "failed": 0}

    for i, tflite_path in enumerate(tflite_files, 1):
        model_name = os.path.basename(tflite_path)
        txt_path = get_txt_path(tflite_path)
        txt_name = os.path.basename(txt_path)

        # Check if regeneration is needed
        if not args.force and not needs_regeneration(tflite_path, txt_path):
            print(f"[{i}/{len(tflite_files)}] [SKIP] {model_name} -> {txt_name}  (up to date)")
            stats["skipped"] += 1
            continue

        if args.dry_run:
            print(f"[{i}/{len(tflite_files)}] [DRY]  {model_name} -> {txt_name}  (would regenerate)")
            stats["skipped"] += 1
            continue

        # Regenerate
        print(f"[{i}/{len(tflite_files)}] [RUN]  {model_name} -> {txt_name} ...", end=" ", flush=True)

        success, error = regenerate_report(tflite_path, txt_path, verbose=args.verbose)

        if success:
            # Verify the file was created
            if os.path.exists(txt_path):
                size_kb = os.path.getsize(txt_path) / 1024
                print(f"[OK] ({size_kb:.1f} KB)")
                stats["regenerated"] += 1
            else:
                print(f"[WARN] (no output file)")
                stats["failed"] += 1
        else:
            print(f"[FAIL] {error}")
            stats["failed"] += 1

    # Summary
    print()
    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"  Total models:       {len(tflite_files)}")
    print(f"  Regenerated:        {stats['regenerated']}")
    print(f"  Skipped (up-to-date): {stats['skipped']}")
    print(f"  Failed:             {stats['failed']}")

    if stats["failed"] > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
