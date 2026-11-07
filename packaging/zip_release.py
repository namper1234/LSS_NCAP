"""Create a portable ZIP from a tested PyInstaller one-folder bundle."""
from __future__ import annotations

import argparse
import zipfile
from pathlib import Path

from lss_ncap import __version__


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", choices=("linux", "windows"), required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    bundle = root / "dist" / "LSS_NCAP_Tool"
    executable = bundle / ("LSS_NCAP_Tool.exe" if args.platform == "windows"
                           else "LSS_NCAP_Tool")
    if not executable.is_file():
        parser.error(f"missing build: {executable}")
    archive = root / "dist" / f"LSS_NCAP_Tool-v{__version__}-{args.platform}-x86_64.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6) as output:
        for path in sorted(bundle.rglob("*")):
            if path.is_file():
                output.write(path, Path(bundle.name) / path.relative_to(bundle))
        output.write(root / "packaging" / "RELEASE_README.md",
                     Path(bundle.name) / "README.md")
    print(archive)


if __name__ == "__main__":
    main()
