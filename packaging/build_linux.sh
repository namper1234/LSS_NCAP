#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-python3}"
"$PYTHON" -m PyInstaller --noconfirm --clean \
    --distpath dist --workpath build/pyinstaller LSS_NCAP_Tool.spec
archive="$($PYTHON packaging/zip_release.py --platform linux | tail -n 1)"
if command -v xvfb-run >/dev/null 2>&1; then
    xvfb-run -a "$PYTHON" packaging/smoke_release.py \
        "$archive" --gui
elif [[ -n "${DISPLAY:-}" ]] && command -v xdpyinfo >/dev/null 2>&1 \
        && xdpyinfo >/dev/null 2>&1; then
    "$PYTHON" packaging/smoke_release.py \
        "$archive" --gui
else
    "$PYTHON" packaging/smoke_release.py "$archive"
    echo "GUI smoke check skipped: install xvfb-run or provide a display." >&2
fi
echo "$archive"
