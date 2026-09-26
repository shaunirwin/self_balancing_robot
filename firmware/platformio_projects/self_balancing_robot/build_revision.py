"""Embed the Git revision in recording headers when building from a checkout."""

import subprocess
from pathlib import Path

Import("env")

root = Path(env["PROJECT_DIR"]).resolve().parents[2]
try:
    revision = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
        stderr=subprocess.DEVNULL, text=True, timeout=2,
    ).strip()
except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
    revision = "unknown"
env.Append(CPPDEFINES=[("SBR_FIRMWARE_REVISION", '\\"' + revision + '\\"')])
