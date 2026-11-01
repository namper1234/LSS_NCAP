"""Test an extracted release with no import or cwd dependency on the source tree."""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import tempfile
import time
import zipfile
from pathlib import Path


def _run(executable: Path, cwd: Path, *args: str) -> None:
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["LOCALAPPDATA" if os.name == "nt" else "XDG_STATE_HOME"] = str(
        cwd / "state")
    result = subprocess.run([str(executable), *args], cwd=cwd,
                            env=environment, capture_output=True,
                            text=True, timeout=60)
    if result.returncode:
        raise AssertionError(
            f"{args} exited {result.returncode}\n{result.stdout}\n{result.stderr}")


def _csv(path: Path, columns: set[str]) -> None:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert columns <= set(reader.fieldnames or ()), path
        assert sum(1 for _ in reader) > 100, path


def _launch_default_gui(executable: Path, cwd: Path) -> None:
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["LOCALAPPDATA" if os.name == "nt" else "XDG_STATE_HOME"] = str(
        cwd / "state")
    process = subprocess.Popen([str(executable)], cwd=cwd, env=environment,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(2)
        assert process.poll() is None, "default desktop launch exited early"
    finally:
        process.terminate()
        process.wait(timeout=10)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("archive", type=Path)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="lss-ncap-release-") as directory:
        temporary = Path(directory)
        with zipfile.ZipFile(args.archive.resolve()) as archive:
            archive.extractall(temporary)
            if os.name != "nt":
                for member in archive.infolist():
                    mode = (member.external_attr >> 16) & 0o777
                    if mode and not member.is_dir():
                        (temporary / member.filename).chmod(mode)
        bundle = temporary / "LSS_NCAP_Tool"
        executable = bundle / ("LSS_NCAP_Tool.exe" if os.name == "nt"
                               else "LSS_NCAP_Tool")
        assert executable.is_file(), executable
        assert (bundle / "_internal" / "lss_ncap" / "assets" /
                "top_down_blue_car.png").is_file()
        assert (bundle / "_internal" / "lss_ncap" / "assets" /
                "top_down_green_gvt.png").is_file()
        outside = temporary / "outside"
        outside.mkdir()
        lka = outside / "lka.csv"
        elk = outside / "elk.csv"
        swa = outside / "swa.csv"
        _run(executable, outside, "--headless", "--output", str(lka))
        _csv(lka, {"steering_command_deg", "lookahead_distance_m"})
        assert (outside / "state" / "LSS_NCAP_Tool" / "logs" /
                "desktop.log").is_file()
        _run(executable, outside, "--headless", "--mode", "elk",
             "--elk-scenario", "oncoming", "--output", str(elk))
        _csv(elk, {"target_x_m", "target_y_m", "steering_command_deg"})
        _run(executable, outside, "--headless", "--swa-only",
             "--output", str(swa))
        _csv(swa, {"steering_wheel_angle_deg", "steering_wheel_angle_rad"})
        if args.gui:
            _run(executable, outside, "--smoke-gui")
            _launch_default_gui(executable, outside)
    print(f"Release smoke checks passed: {args.archive}")


if __name__ == "__main__":
    main()
