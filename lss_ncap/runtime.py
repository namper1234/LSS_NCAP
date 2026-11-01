"""Paths used by both source runs and frozen desktop releases."""
from __future__ import annotations

import os
import sys
from pathlib import Path

APP_NAME = "LSS_NCAP_Tool"


def resource_path(*parts: str) -> Path:
    """Return a bundled package resource (PyInstaller preserves __file__)."""
    return Path(__file__).resolve().parent.joinpath(*parts)


def user_log_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("LOCALAPPDATA") or Path.home() / "AppData" / "Local")
        return base / APP_NAME / "logs"
    base = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
    return base / APP_NAME / "logs"


def user_export_dir() -> Path:
    documents = Path.home() / "Documents"
    return documents if documents.is_dir() else Path.home()
