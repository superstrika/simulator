"""Locations of bundled files (works from a source checkout and from the frozen executable)."""
import os
import shutil
import sys

FROZEN = getattr(sys, "frozen", False)
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
USER_DIR = os.path.join(os.path.expanduser("~"), "RCJ-Sim")
LAST_MATCH = os.path.join(USER_DIR, "last_match.yaml")


def bundled_examples():
    """Examples shipped with the simulator (read-only inside the executable)."""
    if FROZEN:
        return os.path.join(getattr(sys, "_MEIPASS", os.path.dirname(sys.executable)), "examples")
    return os.path.join(os.path.dirname(PKG_DIR), "examples")


def examples_dir():
    """A folder of example robot programs the user can open and edit.

    From a source checkout this is the repository's examples/ folder.  The executable
    copies its bundled examples to ~/RCJ-Sim/examples the first time it runs (the
    bundled copy lives in a temporary folder that disappears on exit)."""
    src = bundled_examples()
    if not FROZEN:
        return src if os.path.isdir(src) else os.getcwd()
    dst = os.path.join(USER_DIR, "examples")
    if not os.path.isdir(dst) and os.path.isdir(src):
        os.makedirs(USER_DIR, exist_ok=True)
        shutil.copytree(src, dst)
    return dst if os.path.isdir(dst) else os.getcwd()
