#!/usr/bin/env python3
"""Regenerate jetacker URDF from xacro source.

Usage: python tools/gen_urdf.py
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "models" / "jetacker" / "jetacker.urdf.xacro"
URDF = ROOT / "models" / "jetacker" / "jetacker.urdf"

result = subprocess.run(["xacro", str(XACRO)], capture_output=True, text=True)
if result.returncode != 0:
    print(result.stderr, file=sys.stderr)
    sys.exit(result.returncode)

# Strip all XML comments (contain = signs and paths that break ROS2 param parsing)
output = re.sub(r'\s*<!--.*?-->', '', result.stdout, flags=re.DOTALL)
# Collapse blank lines
output = re.sub(r'\n{3,}', '\n\n', output)

URDF.write_text(output)
print(f"Generated: {URDF}")
