import argparse
import subprocess
import sys
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='Clean duplicates from training folder based on new inference detection.')
    parser.add_argument('--folder', default='training', help='Folder containing training images (default: training)')
    parser.add_argument('--threshold', type=int, default=5, help='Perceptual hash threshold (default: 5)')
    parser.add_argument('--keep', choices=['oldest', 'newest', 'first'], default='newest', help='Which file to keep (default: newest)')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be deleted without actually deleting')

    args = parser.parse_args()

    dedup_script = Path(__file__).parent / "2_deduplicate.py"
    
    if not dedup_script.exists():
        print(f"Error: Could not find {dedup_script}")
        sys.exit(1)

    # We call 2_deduplicate.py. Since training filenames are prefixed with [new_inference]_[new_confidence],
    # the 2_deduplicate.py script will naturally parse these values just like the extracted/ folder!
    cmd = [
        sys.executable,
        str(dedup_script),
        "--folder", args.folder,
        "--threshold", str(args.threshold),
        "--keep", args.keep,
        "--confidence", "0.0",  # We want to deduplicate ALL training images regardless of their new confidence
    ]
    
    if args.dry_run:
        cmd.append("--dry-run")
    else:
        cmd.append("--delete")

    print(f"Running duplicate cleanup on '{args.folder}'...")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing deduplicate script: {e}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()
