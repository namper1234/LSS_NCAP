# Repository Guidelines

## Project Structure & Module Organization

`lss_ncap/` contains the simulation package. Physics and control live in `dynamics.py`, `eps.py`, `steering.py`, and `simulation.py`; protocol paths and constraints live in `scenarios/`. The Tkinter viewer is in `ui/`, drawing helpers are in `visualization/`, and PNG sprites are in `assets/`. `lka_ncap_profile.py` is the compatibility entry point, while `lss_ncap/cli.py` powers the installed `lss-ncap` command. Regression tests are in `tests/test_*.py`. The root `lka_profile.csv` is a checked-in reference profile used by tests.

## Build, Test, and Development Commands

- `python3 -m pip install -e .` installs the package and CLI for local development (Python 3.10+).
- `python3 lka_ncap_profile.py` opens the desktop viewer.
- `python3 lka_ncap_profile.py --headless --vlat 0.4 --side left --output profile.csv` generates an LKA CSV without the viewer.
- `python3 lka_ncap_profile.py --headless --mode elk --elk-scenario oncoming --vlat 0.4 --output elk.csv` runs an ELK traffic scenario.
- `python3 -m unittest discover -s tests -v` runs the full regression suite.

## Coding Style & Naming Conventions

Follow the existing Python style: four-space indentation, `snake_case` functions and modules, `PascalCase` classes, and descriptive names with units such as `_m`, `_mps`, and `_deg`. Add type hints for public interfaces and keep simulation calculations deterministic. Keep scenario rules in `scenarios/`, model behavior in the core modules, and presentation behavior in `ui/` or `visualization/`. No formatter or linter is configured in `pyproject.toml`; match nearby code when editing.

## Testing Guidelines

Use the standard-library `unittest` framework. Name new files `tests/test_<feature>.py` and methods `test_<behavior>`. Cover physical behavior, scenario constraints, CSV compatibility, and viewer state where relevant. Run the full suite before proposing changes. If an intentional model change alters the reference CSV, regenerate it and explain the changed output.

## Commit & Pull Request Guidelines

Git history is unavailable in this checkout, so no repository-specific commit format can be verified. Use short, imperative commit subjects that describe the change. Pull requests should summarize behavior changes, list the test command and result, and include screenshots for viewer changes or sample CSV details for export changes. Link an issue when one exists.
