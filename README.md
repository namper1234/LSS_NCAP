# LSS Euro NCAP LKA and ELK simulation

A deterministic Euro NCAP LSS v4.2 LKA/ELK path-following simulation with a
two-lane Tkinter viewer. The simulator uses 25 ms controller/output samples and synchronized
5 ms bicycle-model and position-integration substeps.
Source runs require Python 3.10 or newer, Tkinter, and Pillow. Desktop ZIP
releases include Python and the application dependencies.

## Desktop releases

Download the ZIP for your platform from GitHub Releases, extract the entire
`LSS_NCAP_Tool` folder, and open `LSS_NCAP_Tool` on Ubuntu 22.04+ or
`LSS_NCAP_Tool.exe` on Windows 10/11. Keep `_internal` beside the executable;
it contains the Python runtime, Tcl/Tk, Pillow, and both vehicle sprites.
No Python installation or terminal command is needed to use the viewer.
The Ubuntu build needs a graphical desktop and standard system graphics/Tk
libraries. On some Linux file managers, mark the executable as launchable in
its Properties dialog before double-clicking.

Exports are saved wherever the user chooses. The save dialog initially opens
in Documents, or the home folder if Documents does not exist. Startup errors
are recorded in `desktop.log` under
`~/.local/state/LSS_NCAP_Tool/logs` (or `$XDG_STATE_HOME/LSS_NCAP_Tool/logs`)
on Linux and `%LOCALAPPDATA%\LSS_NCAP_Tool\logs` on Windows. A startup
failure also displays a dialog when a graphical display is available.

### Build and verify a release

Build on each target operating system; PyInstaller does not cross-compile.
Use Python 3.10+ on Ubuntu 22.04 and a 64-bit Python 3.11 on Windows.

```bash
# Ubuntu 22.04+
./packaging/build_linux_complete.sh
```

The complete Linux script installs Ubuntu system packages (using `sudo`),
creates `.venv`, installs Python dependencies, runs regression tests, builds
the executable, creates the ZIP, and smoke-tests it. On a preconfigured
Linux system without `sudo`, set `SKIP_SYSTEM_DEPS=1` before the command.

```powershell
# Windows 10/11
python -m venv .venv
.venv\Scripts\python.exe -m pip install -e . -r packaging/build-requirements.txt
./packaging/build_windows.ps1 -Python .venv\Scripts\python.exe
```

The scripts use the shared `LSS_NCAP_Tool.spec` and produce portable ZIP files
under `dist/`. Launch `dist/LSS_NCAP_Tool/LSS_NCAP_Tool` on Linux or
`dist\LSS_NCAP_Tool\LSS_NCAP_Tool.exe` on Windows; the executable under
`build/pyinstaller/` is an incomplete intermediate artifact. The scripts
extract each ZIP to a temporary directory and test LKA,
ELK oncoming, SWA export, and the bundled assets. With a display (or Xvfb on
Linux), they also start the packaged Tk viewer, draw plots, exercise zoom and
reset, and switch to ELK. Build failure or a failed smoke check stops the
script. Run `python3 -m unittest discover -s tests -v` before releasing.

GitHub Actions runs the tests and native builds on Ubuntu 22.04 and Windows
Server 2022. Pull requests and branch pushes retain the ZIPs as workflow
artifacts. Pushing a `v*` tag publishes both ZIPs to a GitHub Release only
after both native build and smoke jobs pass.

## Run

```bash
python3 lka_ncap_profile.py
python3 lka_ncap_profile.py --headless --vlat 0.4 --side left --output profile.csv
python3 lka_ncap_profile.py --headless --vlat 0.4 --swa-only --output swa_profile.csv
python3 lka_ncap_profile.py --headless --mode elk --elk-scenario oncoming --vlat 0.4 --output elk_oncoming.csv
python3 lka_ncap_profile.py --headless --mode elk --elk-scenario overtaking --intent intentional --target-speed 80 --vlat 0.6 --output elk_overtaking.csv
python3 -m unittest discover -s tests -v
```

The window provides a Function selector for LKA or ELK. ELK dynamically exposes
Road Edge, Solid Line, Oncoming Vehicle, and Overtaking Vehicle scenarios. It
restricts departure direction and lateral-speed choices to the selected protocol
case and enables lane-change intent and GVT speed only for overtaking. Road
layout, markings, traffic target, charts, telemetry, and CSV output update with
the selection.

The remaining controls provide lateral-speed target, departure direction, lane-line style,
and 3.5/3.7 m lane-width selectors. The road marking is 0.2 m wide at the
current display scale. Play, reset, full CSV export, compact SWA export, and bottom timeline scrubbing
are available. The live card reports the steering-wheel command, road-frame lateral
speed (`V_lat`), and vehicle heading (`ψ`). Scrubbing pauses playback; Play
continues from the selected 25 ms sample.

Steering-wheel angle, lateral velocity, and lateral trajectory tracking are
shown in separate synchronized chart panels. The steering chart shows the
command applied over each following 25 ms model interval and a separate NCAP
reference. The
trajectory chart overlays the scheduled NCAP reference lateral position and
the simulated vehicle's front-axle lateral position, with the
selected frame's tracking error shown in metres. The header's
Light/Dark Mode control changes the complete dashboard palette immediately and
does not reset playback, the selected scenario, or the current timeline frame.
Playback uses a monotonic wall clock, preventing animation drift when rendering
is briefly delayed. Space toggles play/pause, the arrow keys step one exact
25 ms sample, and Home/End jump to the beginning or final scenario frame.

Typography is managed through a single Inter-first type system. When Inter is
not installed, the application selects the first available professional UI
fallback from Segoe UI, SF Pro, Helvetica Neue, Noto Sans, and DejaVu Sans.
Titles, telemetry, controls, legends, captions, and canvas annotations share a
consistent size and weight scale in both light and dark themes.

Primary and secondary actions use reusable rounded dashboard buttons inspired
by modern automotive tooling: compact icon/text spacing, subtle elevation,
theme-aware borders, high-contrast primary color, and dedicated hover, pressed,
keyboard-focus, and Enter/Space activation states.

Mode and parameter fields use matching rounded selector buttons rather than
platform-native comboboxes. Each selector opens a floating, theme-aware menu
with generous row spacing, hover feedback, a current-value checkmark, keyboard
activation, and the same dynamic enable/disable behavior used by LKA and ELK.

Every chart has an independent interactive viewport. Scroll over a plot to zoom
around the pointer, drag with the primary mouse button to pan, and use Reset
View to restore all plots to their original time and value ranges. View changes
affect presentation only; simulation samples and exported data are unchanged.
Use **Expand** on the road view or any chart to fill the viewer with that panel;
**Restore** or Escape returns to the dashboard. The timeline stays available
while a panel is expanded.

The ego vehicle uses a transparent blue top-down sprite, while oncoming and
overtaking GVT traffic uses a separate dark-green sprite derived from the
provided references. Frames are scaled to the configured physical body length
and width and rotated to the simulated heading; transformed frames are cached
at 0.1° resolution for smooth, lightweight playback. A deterministic vector
model remains available as a non-Tk/test fallback. Collision detection continues
to use the same validated physical vehicle footprint.

## ELK traffic scenarios

All ELK cases use the same closed-loop Pure Pursuit controller and kinematic
bicycle vehicle model as LKA.
Unintentional departures use the 1200 m path at 0.2–0.6 m/s. Intentional
overtaking uses the protocol's 800 m path at 0.5–0.7 m/s. Road Edge is
passenger-side only, Oncoming and Overtaking are driver-side only, and Solid
Line supports both sides.

For Oncoming and Overtaking, the GVT path is 1.5 m beyond the centre dashed
line. The simulator computes its initial longitudinal coordinate from the
nominal encounter geometry:

- Oncoming at 72 km/h: front edges meet with 10% lateral overlap.
- Overtaking: the GVT leading edge aligns with the VUT rear axle, at either
  72 or 80 km/h.

The LSS document does not numerically define the ISO GVT footprint, so this
visual simulator uses the configured VUT rectangle for the GVT when computing
edge synchronization. Initial and live GVT coordinates appear in the UI and
the full `target_x_m`, `target_y_m`, `target_yaw_deg`, and `target_speed_mps`
series is included in traffic-scenario CSV files. Steering command, applied
angle, and the rest of the steering profile remain exported for every mode.

Traffic scenarios use continuous oriented-rectangle collision detection between
25 ms output samples. The first-contact instant is refined by bisection, both
vehicles are stopped at that contact pose, and the run is truncated immediately.
The timeline therefore ends at impact and displays `COLLISION DETECTED`; no
post-impact samples are generated in which the vehicles overlap or pass through
one another.

`v_lat_mps` in CSV is road-frame lateral speed. `body_lateral_speed_mps` is the
vehicle-body lateral component; it can approach zero while a yawed car is
moving sideways across the road.

## Vehicle configuration and steering profile

The viewer's grouped **Vehicle Configuration** panel edits wheelbase, length,
width, and steering ratio. VF8 NP REEV defaults are 2.950 m, 4.957 m, 2.013 m,
and 17.86 respectively. Press Enter or leave a field to regenerate the run.
Wheelbase sets the kinematic bicycle geometry; body length and width change
rendering and collision geometry. Vehicle mass remains internal at 2485 kg by
default; the kinematic bicycle equations do not depend on it.
The LKA **NCAP path** selector adds a tight-curve scenario with an 800 m radius
while retaining the standard 1200 m path and all existing ELK scenarios.
Changing these settings updates the road view and all three plots. Internally,
the controller uses `max(minimum distance, speed * look-ahead time)` with fixed
viewer defaults of 8 m and 1.1 s (22 m at 72 km/h).

The lateral-speed selection shapes the NCAP path. At each 25 ms sample, the
scheduled path state is evaluated at distance `speed * time`, yielding
`reference_x_m`, `reference_y_m`, heading, and geometric curvature. The NCAP
arc begins at **5.0 s**. At each 25 ms control tick, Pure Pursuit reads the
simulated front-axle position and heading, picks a point ahead on the NCAP
path, and computes a steering command from that position error. For rear-axle
steering geometry, the target comes from a reference station one wheelbase
earlier and the vehicle pose comes from the simulated rear axle. This avoids a
false opposite-direction target at the beginning of the NCAP arc.
For target lateral offset `y` and chord length `d`, the commanded curvature is
`2*y/d²`; the steering-wheel command is:

`SWA(t) = steering_ratio * atan(wheelbase * pursuit_curvature(t))`.

The commanded road-wheel angle drives the kinematic bicycle model over 5 ms
substeps. Its updated position and heading feed the next Pure Pursuit tick.
The trajectory chart shows the resulting front-axle
tracking. Front-axle road-frame lateral speed is
`v*sin(yaw) + wheelbase*yaw_rate*cos(yaw)`; the reported lateral velocity
therefore matches the plotted front-axle motion. The resulting time-based SWA
series can be replayed in DYNA4 for this scenario and parameter set.

CSV includes `time_s`, `reference_x_m`, `reference_y_m`,
`reference_curvature_per_m`, `profile_curvature_per_m` (the Pure Pursuit
commanded curvature), the look-ahead target coordinates and distance, and
`steering_command_deg` for DYNA4 profile replay. `steering_wheel_deg` and
`road_wheel_deg` record the applied command at the start of each sample;
the new command applies over `[t, t+25 ms)`. The checked-in `lka_profile.csv`
has been regenerated for the corrected rear-axle target geometry.
The **Export SWA** button and `--swa-only` option produce a compact CSV with
time plus steering-wheel angle in degrees and radians for mapping into an
external replay tool.

### Look-ahead simulation support

Look-ahead parameters remain in the simulation configuration and CSV export for
programmatic use. The reusable `lss_ncap.lookahead_tuning` module can evaluate
candidate times outside the viewer. The viewer uses the fixed defaults and does
not show look-ahead settings, tuning results, or target annotations.

### Project modules

| Module | Responsibility |
| --- | --- |
| `lss_ncap/config.py` | Validated physical, lane, and display settings |
| `lss_ncap/dynamics.py` | Kinematic bicycle vehicle model |
| `lss_ncap/scenarios/` | LKA/ELK paths, protocol constraints, and intervention policy |
| `lss_ncap/steering.py` | Pure Pursuit path-tracking controller |
| `lss_ncap/simulation.py` | Deterministic simulation and typed samples |
| `lss_ncap/lookahead_tuning.py` | Reusable coarse-to-fine simulation tuner and scoring |
| `lss_ncap/export.py` | CSV serialization |
| `lss_ncap/visualization/` | Road projection, vehicle geometry, and charts |
| `lss_ncap/ui/` | Tk application, theme, and playback state |
| `lka_ncap_profile.py` | Original CLI and import compatibility entry point |

The simulator generates the NCAP approach path and steering profile. A
production lane-support controller can still be connected through
`SimulationEngine.run_elk(..., lane_support=callback)`; without that callback,
the scheduled profile continues driving the vehicle so the scenario input and
vehicle encounter can be inspected independently.
