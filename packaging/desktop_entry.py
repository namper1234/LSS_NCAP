"""PyInstaller desktop entry point with per-user diagnostics."""
from __future__ import annotations

import logging
import sys
import tempfile
from logging.handlers import RotatingFileHandler
from pathlib import Path

from lss_ncap.runtime import APP_NAME, user_log_dir


def _configure_logging() -> Path:
    try:
        log_dir = user_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / "desktop.log"
        handler = RotatingFileHandler(log_path, maxBytes=1_000_000,
                                      backupCount=3, encoding="utf-8")
    except OSError:
        log_dir = Path(tempfile.gettempdir()) / APP_NAME / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / "desktop.log"
        handler = RotatingFileHandler(log_path, maxBytes=1_000_000,
                                      backupCount=3, encoding="utf-8")
    handler.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)s %(name)s: %(message)s"))
    logging.basicConfig(level=logging.INFO, handlers=[handler], force=True)
    return log_path


def _show_error(log_path: Path) -> None:
    message = ("LSS NCAP Tool could not start.\n\n"
               f"Diagnostic log: {log_path}")
    try:
        import tkinter as tk
        from tkinter import messagebox

        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("LSS NCAP Tool", message, parent=root)
        root.destroy()
    except Exception:
        if sys.stderr is not None:
            print(message, file=sys.stderr)


def _smoke_gui() -> None:
    """Exercise the frozen Tk viewer without requiring a person to click it."""
    import tkinter as tk

    from lss_ncap.scenarios.elk import ElkScenario
    from lss_ncap.ui.app import SimulationApp
    from lss_ncap.visualization.vehicle import EGO_SPRITE_PATH, GVT_SPRITE_PATH

    assert EGO_SPRITE_PATH.is_file() and GVT_SPRITE_PATH.is_file()
    root = tk.Tk()
    app = None
    try:
        app = SimulationApp(root=root)
        root.update()
        assert app.run_data.mode == "LKA"
        assert app.run_data.samples
        assert app.road_canvas.winfo_width() > 1
        app.chart_views[0].zoom(0.5, 0.5, zoom_in=True)
        app.redraw_charts()
        app.reset_chart_views()
        app.mode_var.set("ELK")
        app.elk_scenario_var.set("Oncoming Vehicle")
        app._scenario_selection_changed()
        root.update()
        assert app.run_data.mode == "ELK"
        assert app.run_data.scenario_key == ElkScenario.ONCOMING.value
        assert app.run_data.target_samples
    finally:
        if app is not None:
            app.close()
        else:
            root.destroy()


def main() -> int:
    log_path = _configure_logging()
    try:
        if sys.argv[1:] == ["--smoke-gui"]:
            _smoke_gui()
            return 0
        from lss_ncap.cli import main as cli_main

        return cli_main()
    except SystemExit as error:
        if error.code not in (None, 0):
            logging.getLogger(__name__).error("Application exited with code %s", error.code)
            if "--headless" not in sys.argv:
                _show_error(log_path)
        return error.code if isinstance(error.code, int) else 1
    except Exception:
        logging.getLogger(__name__).exception("Unhandled desktop startup failure")
        if "--headless" not in sys.argv:
            _show_error(log_path)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
