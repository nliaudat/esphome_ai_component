#!/usr/bin/env python3
"""Fix ALL non-ASCII characters across component source files."""

import os

replacements = {
    "\u2014": "--",  # em dash
    "\u00b0": " deg",  # degree symbol
    "\u2192": "->",  # right arrow
    "\u26a0": "WARNING:",  # warning sign
    "\ufe0f": "",  # emoji variation selector
    "\u00b1": "+-",  # plus-minus sign
    "\u00d7": "x",  # multiplication sign
    "\u2013": "--",  # en dash
    "\u2500": "-",  # box drawing horizontal
    "\u2713": "OK",  # check mark
    "\u00b0": " deg",  # degree symbol (repeated for completeness)
}

count = 0
extensions = {".cpp", ".h", ".tcc", ".py"}
for root, dirs, files in os.walk("components"):
    dirs[:] = [
        d for d in dirs if not d.startswith(".") and d != "venv" and d != "__pycache__"
    ]
    for fname in files:
        ext = os.path.splitext(fname)[1].lower()
        if ext in extensions:
            path = os.path.join(root, fname)
            try:
                with open(path, encoding="utf-8") as f:
                    content = f.read()
                original = content
                for char, repl in replacements.items():
                    if char in content:
                        content = content.replace(char, repl)
                if content != original:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(content)
                    print(f"Fixed: {path}")
                    count += 1
            except UnicodeDecodeError:
                pass

print(f"Additional fixes: {count}")
