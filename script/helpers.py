"""Helper utilities for ESPHome AI Component lint scripts."""

import subprocess

import colorama


def styled(color, msg, reset=True):
    """Apply ANSI color/style to a message."""
    prefix = "".join(color) if isinstance(color, tuple) else color
    suffix = colorama.Style.RESET_ALL if reset else ""
    return prefix + msg + suffix


def print_error_for_file(file, body=None):
    """Print a formatted error block for a file.

    On Windows, non-ASCII characters in error messages may crash the cp1252
    console. Fall back to raw stderr write in that case.
    """
    try:
        print(
            styled(colorama.Fore.GREEN, "### File ")
            + styled((colorama.Fore.GREEN, colorama.Style.BRIGHT), str(file))
        )
        print()
        if body is not None:
            print(body)
            print()
    except UnicodeEncodeError:
        # Fallback: write raw bytes to stderr
        import sys as _sys

        try:
            _sys.stderr.buffer.write(
                ("### File " + str(file) + "\n\n" + (body or "") + "\n\n").encode(
                    "utf-8", errors="replace"
                )
            )
        except Exception:
            pass


def filter_changed(files):
    """Filter files to only those changed vs the default branch."""
    try:
        # Try to find the merge-base with main or dev
        for branch in ("main", "dev", "master"):
            for remote in ("origin",):
                try:
                    merge_base = subprocess.check_output(
                        [
                            "git",
                            "merge-base",
                            f"refs/remotes/{remote}/{branch}",
                            "HEAD",
                        ],
                        text=True,
                    ).strip()
                    changed = subprocess.check_output(
                        ["git", "diff", merge_base, "--name-only"], text=True
                    ).splitlines()
                    return [f for f in files if f in changed]
                except subprocess.CalledProcessError:
                    continue
    except Exception:
        pass
    # Fallback: return all files
    return files


def git_ls_files(patterns=None):
    """Run git ls-files and return {path: mode} dict."""
    command = ["git", "ls-files", "-s"]
    if patterns is not None:
        command.extend(patterns)
    with subprocess.Popen(command, stdout=subprocess.PIPE) as proc:
        output, _ = proc.communicate()
    lines = [x.split() for x in output.decode("utf-8").splitlines()]
    return {s[3].strip(): int(s[0]) for s in lines}
