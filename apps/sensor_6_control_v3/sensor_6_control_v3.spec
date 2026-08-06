# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path
import sys


app_dir = Path(SPECPATH).resolve()
project_root = app_dir.parents[1]
conda_bin = Path(sys.executable).resolve().parent / "Library" / "bin"

required_dlls = (
    "libcrypto-3-x64.dll",
    "libssl-3-x64.dll",
    "liblzma.dll",
    "LIBBZ2.dll",
    "libexpat.dll",
    "ffi.dll",
    "ffi-7.dll",
    "ffi-8.dll",
    "sqlite3.dll",
)
binaries = [
    (str(conda_bin / dll_name), ".")
    for dll_name in required_dlls
    if (conda_bin / dll_name).is_file()
]

datas = [
    (str(app_dir / "main_window_3.ui"), "."),
    (str(app_dir / "main_window.ui"), "."),
    (str(app_dir / "使用说明.txt"), "."),
    (str(app_dir / "picture" / "jlu.png"), "picture"),
    (str(app_dir / "picture" / "jlu.ico"), "picture"),
]

analysis = Analysis(
    [str(app_dir / "main.py")],
    pathex=[str(app_dir), str(project_root)],
    binaries=binaries,
    datas=datas,
    hiddenimports=["matplotlib.backends.backend_qtagg"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(analysis.pure)

exe = EXE(
    pyz,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="sensor_6_control_v3",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(app_dir / "picture" / "jlu.ico"),
)

collection = COLLECT(
    exe,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="sensor_6_control_v3",
)
