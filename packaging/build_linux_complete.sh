#!/usr/bin/env bash
# Build and verify the Ubuntu desktop release from a fresh checkout.
set -euo pipefail

cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This script must run on Linux." >&2
    exit 1
fi

if [[ "${SKIP_SYSTEM_DEPS:-0}" != "1" ]]; then
    if ! command -v apt-get >/dev/null 2>&1; then
        echo "Automatic system setup requires Ubuntu/Debian apt-get." >&2
        echo "Install Python 3 venv, Tk, Xvfb, and xauth, then rerun with SKIP_SYSTEM_DEPS=1." >&2
        exit 1
    fi
    if [[ "$(id -u)" -eq 0 ]]; then
        apt-get update
        apt-get install -y python3-venv python3-tk xvfb xauth
    else
        if ! command -v sudo >/dev/null 2>&1; then
            echo "sudo is required to install system packages." >&2
            exit 1
        fi
        sudo apt-get update
        sudo apt-get install -y python3-venv python3-tk xvfb xauth
    fi
fi

VENV_DIR="${BUILD_VENV:-.venv}"
python3 -m venv "$VENV_DIR"
BUILD_PYTHON="$(cd "$VENV_DIR" && pwd)/bin/python"
"$BUILD_PYTHON" -m pip install -e . -r packaging/build-requirements.txt

if command -v xvfb-run >/dev/null 2>&1; then
    xvfb-run -a "$BUILD_PYTHON" -m unittest discover -s tests -v
else
    "$BUILD_PYTHON" -m unittest discover -s tests -v
    echo "Virtual display unavailable; Tk tests may be skipped." >&2
fi

PYTHON="$BUILD_PYTHON" bash packaging/build_linux.sh
echo "Ubuntu release: $(pwd)/dist/LSS_NCAP_Tool-linux-x86_64.zip"
