"""Shared one-folder PyInstaller build for Linux and Windows."""
from pathlib import Path
import sys

root = Path.cwd().resolve()
if not (root / "pyproject.toml").is_file():
    raise RuntimeError("Run PyInstaller from the project root")

app_icon = root / "lss_ncap" / "assets" / "vinfast_app_icon.ico"
analysis = Analysis(
    [str(root / "packaging" / "desktop_entry.py")],
    pathex=[str(root)],
    binaries=[],
    datas=[(str(root / "lss_ncap" / "assets"), "lss_ncap/assets")],
    hiddenimports=["lss_ncap.ui.app"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(analysis.pure)
exe = EXE(
    pyz,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="LSS_NCAP_Tool",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    icon=str(app_icon) if sys.platform == "win32" else None,
)
bundle = COLLECT(
    exe,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    name="LSS_NCAP_Tool",
)
