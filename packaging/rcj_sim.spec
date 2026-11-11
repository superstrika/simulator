# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller build of the simulator as a single executable.

    pyinstaller packaging/rcj_sim.spec --noconfirm --clean

Robot programs are loaded from disk at runtime, so PyInstaller cannot see what
they import.  The standard library is therefore bundled as a whole (minus GUI /
test / packaging modules), so `import random`, `statistics`, `json`, ... work in
user code without a Python installation.
"""
import importlib.util
import os
import sys

from PyInstaller.utils.hooks import collect_data_files, collect_submodules

ROOT = os.path.dirname(SPECPATH)

EXCLUDE = {
    "antigravity", "this", "idlelib", "tkinter", "turtle", "turtledemo", "test", "lib2to3",
    "ensurepip", "venv", "pydoc", "pydoc_data", "distutils", "msilib", "pip", "setuptools",
}

# Python < 3.10 has no sys.stdlib_module_names: fall back to the commonly used modules.
FALLBACK = """
__future__ abc argparse array ast asyncio base64 binascii bisect calendar cmath codecs collections
colorsys concurrent configparser contextlib copy copyreg csv ctypes dataclasses datetime decimal difflib
email enum fractions functools getopt gettext glob graphlib gzip hashlib heapq hmac html http importlib
inspect io ipaddress itertools json locale logging math multiprocessing numbers operator os pathlib
pickle platform pprint queue random re sched secrets select selectors shlex shutil signal socket sqlite3
ssl statistics string struct subprocess sys tarfile tempfile textwrap threading time timeit traceback
types typing unicodedata unittest urllib uuid warnings weakref xml zipfile zlib
""".split()


def _skip_submodule(name):
    parts = name.split(".")
    return any(p in ("test", "tests", "idle_test") or p.startswith("test_") for p in parts)


def stdlib_imports():
    names = getattr(sys, "stdlib_module_names", None) or FALLBACK
    out = []
    for name in sorted(names):
        if name in EXCLUDE or name.startswith("_"):
            continue
        try:
            spec = importlib.util.find_spec(name)
        except Exception:
            continue
        if spec is None:
            continue
        out.append(name)
        if spec.submodule_search_locations:
            out += collect_submodules(name, filter=lambda m: not _skip_submodule(m))
    return out


hiddenimports = stdlib_imports() + collect_submodules("rcj_sim") + ["yaml", "pymunk", "pygame"]

a = Analysis(
    [os.path.join(SPECPATH, "rcj_sim_app.py")],
    pathex=[ROOT],
    binaries=[],
    datas=[(os.path.join(ROOT, "examples"), "examples")] + collect_data_files("pymunk"),
    hiddenimports=hiddenimports,
    hookspath=[],
    runtime_hooks=[],
    excludes=sorted(EXCLUDE),
    noarchive=False,
)
pyz = PYZ(a.pure)

# RCJ_ONEDIR=1 builds a folder (dist/rcj-sim/) instead of one file.  It starts much faster
# on macOS, where every library a one-file build unpacks is re-verified on each launch.
ONEDIR = os.environ.get("RCJ_ONEDIR") == "1"

exe_args = dict(
    name="rcj-sim",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,          # robot print()s and errors also go to the terminal
    disable_windowed_traceback=False,
)

if ONEDIR:
    exe = EXE(pyz, a.scripts, [], exclude_binaries=True, **exe_args)
    coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="rcj-sim")
else:
    exe = EXE(pyz, a.scripts, a.binaries, a.datas, [], runtime_tmpdir=None, **exe_args)
