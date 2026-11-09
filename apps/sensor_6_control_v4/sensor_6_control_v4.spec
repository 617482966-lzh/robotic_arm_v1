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
    (str(app_dir / "main_window_4.ui"), "."),
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
    # Matplotlib/OpenPyXL inspect several optional libraries. They are not used
    # by this NumPy + QtAgg + XLSX app, even when installed in the build env.
    excludes=[
        "torch", "torchvision", "torchaudio", "scipy", "pandas",
        "numba", "sympy", "IPython", "notebook", "pytest",
    ],
    noarchive=False,
    optimize=0,
)

# This Qt build imports the unversioned ICU API supplied by Windows. A Poppler
# directory on the build machine's PATH can otherwise inject ICU78 (whose
# exports end in _78), causing QtWidgets to fail at startup with WinError 127.
# Keep OS ICU resolution intact rather than copying a system DLL into the app.
analysis.binaries = [
    entry for entry in analysis.binaries
    if Path(entry[0]).name.lower() != "icuuc.dll"
    and not (
        Path(entry[0]).name.lower().startswith("icudt")
        and Path(entry[0]).name.lower().endswith(".dll")
    )
]

pyz = PYZ(analysis.pure)

exe = EXE(
    pyz,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="sensor_6_control_v4",
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
    name="sensor_6_control_v4",
)
