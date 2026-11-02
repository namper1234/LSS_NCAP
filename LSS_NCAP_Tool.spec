"""Shared one-folder PyInstaller build for Linux and Windows."""
from pathlib import Path
import sys

root = Path.cwd().resolve()
if not (root / "pyproject.toml").is_file():
    raise RuntimeError("Run PyInstaller from the project root")

sprite = root / "lss_ncap" / "assets" / "top_down_blue_car.png"
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
    noarchive=False,Please improve the plotting system in my LKA/ELK NCAP simulation tool to provide a professional, MATLAB-like plotting experience.

**Requirements:**

1. **Interactive Legend:** Allow users to click any signal name in the legend to show or hide that signal without affecting the others. Visually distinguish hidden signals.
2. **Zoom In/Out:** Support mouse-wheel zooming, rectangle selection zoom, and intuitive pan/drag interactions.
3. **Reset View:** Add a clearly visible **Reset View** button to restore the original axis limits and auto-fit all data.
4. **Expanded Mode:** Allow plots to expand for detailed analysis. Ensure all interactions, especially zoom, pan, legend toggling, and Reset View, continue working correctly in expanded mode.
5. **Professional MATLAB-like Design:** Improve grid lines, axis labels, tick formatting, signal colors, legends, tooltips, and overall readability.
6. **Consistency:** Preserve plot visibility and interaction states when switching between normal and expanded modes. Reset View must work reliably even after multiple zoom operations.
7. **Code Quality:** Refactor the plotting components where necessary, following clean, modular, maintainable software engineering practices.

Keep all existing simulation calculations, signal data, and functionality unchanged. Focus on improving the plotting UI and user experience. Implement and test the changes directly in the existing project.
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
    icon=str(sprite) if sys.platform == "win32" else None,
)
bundle = COLLECT(
    exe,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    name="LSS_NCAP_Tool",
)
