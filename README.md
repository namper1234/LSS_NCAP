# LSS Euro NCAP LKA and ELK simulation

Current release: **2.0.0**.

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
./packaging/build_windows.ps1
```

The Windows script creates `.venv`, installs dependencies, runs all tests,
builds the executable, creates the ZIP, and smoke-tests the packaged app.
If `python` is not the 64-bit Python 3.11+ interpreter, pass its path with
`-Python`. The scripts use the shared `LSS_NCAP_Tool.spec` and produce
portable ZIP files under `dist/`. Launch
`dist/LSS_NCAP_Tool/LSS_NCAP_Tool` on Linux or
`dist\LSS_NCAP_Tool\LSS_NCAP_Tool.exe` on Windows; the executable under
`build/pyinstaller/` is an incomplete intermediate artifact. The scripts
extract each ZIP to a temporary directory and test LKA,
ELK oncoming, SWA export, and the bundled assets. With a display (or Xvfb on
Linux), they also start the packaged Tk viewer, draw plots, exercise zoom and
reset, and switch to ELK. Build failure or a failed smoke check stops the
script. Run `python3 -m unittest discover -s tests -v` before releasing.

To request a native Windows build while working on Ubuntu, commit and push
the current branch, then run:

```bash
./packaging/build_windows_from_ubuntu.sh
```

The script obtains GitHub CLI when needed, starts the release workflow,
waits for its tests and smoke checks, and downloads the Windows ZIP to `dist/`.
Sign in with `gh auth login -h github.com --web` if prompted.

GitHub Actions runs the tests and native builds on Ubuntu 22.04 and Windows
Server 2022. Pull requests and branch pushes retain the ZIPs as workflow
artifacts. Pushing a `v*` tag publishes both ZIPs to a GitHub Release only
after both native build and smoke jobs pass.

## Run

```bash
python3 main.py
python3 main.py --headless --vlat 0.4 --side left --output profile.csv
python3 main.py --headless --vlat 0.47 --d2 1.3 --radius 725 --output custom_lka.csv
python3 main.py --headless --ncap-profile alternative_dim --vlat 0.8 --side right --output alternative.csv
python3 main.py --headless --vlat 0.4 --swa-only --output swa_profile.csv
python3 main.py --headless --mode elk --elk-scenario oncoming --vlat 0.4 --output elk_oncoming.csv
python3 main.py --headless --mode elk --elk-scenario overtaking --intent intentional --target-speed 80 --vlat 0.6 --output elk_overtaking.csv
python3 main.py --headless --mode elk --elk-scenario oncoming --vlat 0.53 --d2 0.37 --radius 675 --vut-speed 65 --target-speed 89 --gvt-marker-distance 1.7 --overlap 25 --gvt-length 4.2 --gvt-width 1.85 --output elk_custom.csv
python3 main.py --headless --mode elk --elk-scenario oncoming --gvt-type c2_suv --output elk_suv.csv
python3 main.py --headless --mode elk --elk-scenario oncoming --gvt-type custom --gvt-length 3.8 --gvt-width 1.7 --gvt-wheelbase 2.4 --output elk_custom_gvt.csv
python3 -m unittest discover -s tests -v
```

The window provides a Function selector for LKA or ELK. ELK dynamically exposes
Road Edge, Solid Line, Oncoming Vehicle, and Overtaking Vehicle scenarios. It
restricts departure direction to the selected protocol case and enables
lane-change intent for overtaking. Lateral speed, d2, radius, and VUT speed
are freely editable; traffic scenarios also expose GVT speed, marker offset,
overlap, and a GVT image selector with six predefined types plus Custom GVT. Road
layout, markings, traffic target, charts, telemetry, and CSV output update with
the selection.

The remaining controls provide lateral-speed target, departure direction, lane-line style,
3.5/3.7 m lane-width selectors, and an editable approach time. Changing the
approach time regenerates the scenario and moves the scheduled NCAP arc start.
Both modes provide editable **d2 (m)** and **R (m)** fields. Editing geometry
or lateral/vehicle speed automatically regenerates the reference path, Pure Pursuit
steering commands, vehicle trajectory, road view, and all three plots after a
short typing delay; Enter or leaving the field applies changes immediately.
Invalid inputs show an error in the status bar and retain the last valid run.
The LKA **Vehicle speed (m/s)** field defaults to 20 m/s (72 km/h).
ELK has independent **VUT (km/h)** and **GVT (km/h)** inputs; switching modes
preserves each mode's speed.

### ELK target types

The GVT selector lists the six supplied targets in reference-image order, followed
by **Custom GVT**. Each predefined option has a distinct **original schematic
thumbnail**, a type name, and its dimensions. These illustrations are not
manufacturer photographs or CAD models. Custom GVT uses the bundled
[green GVT sprite](lss_ncap/assets/top_down_green_gvt.png) for its thumbnail and road visualization.
The first type, **4activeMC-STA**, is selected initially.

| Target | CLI/API key | GVT length (m) | Width (m) | Wheelbase (m) |
|---|---|---:|---:|---:|
| [4activeMC-STA](https://www.4activesystems.at/4activemc-sta/) | `mc_sta` | 1.410 (estimate) | 0.540 | 1.210 |
| [4activeMC-AMT](https://www.4activesystems.at/4activemc-amt/) | `mc_amt` | 1.5215 (estimate) | 0.675 | 1.255 |
| [4activeMC-EMT](https://www.4activesystems.at/4activemc-emt/) | `mc_emt` | 1.720 (estimate) | 0.750 | 1.420 |
| [4activeC2-SVT](https://www.4activesystems.at/4activec2-svt/) | `c2_svt` | 2.800 | 1.550 | 1.800 |
| [4activeC2-GVT](https://www.4activesystems.at/4activec2-gvt/) | `c2_gvt` | 4.060 | 1.800 | 2.560 |
| [4activeC2-SUV](https://www.4activesystems.at/4activec2-suv/) | `c2_suv` | 4.350 | 1.805 (without mirrors) | 2.600 |

These dimensions and source links live in [the target catalogue](lss_ncap/targets.py).
The motorcycle references publish wheelbase and wheel diameter, **not overall
length**. Motorcycle length now auto-fills using the user-approved approximation:
**GVT length = wheelbase + wheel diameter / 2**. The diameter used is 0.400 m for
STA, 0.533 m for AMT (rear wheel, explicitly selected by the user; its front wheel
is 0.517 m), and 0.600 m for EMT. These estimates are not manufacturer overall
lengths or full front-to-rear wheel envelopes; they do not account for body overhang.
The dropdown and telemetry identify estimated lengths.

Motorcycle lengths remain editable for measured overrides, remembered separately
for each motorcycle type during the viewer session. Clearing or entering an
invalid length blocks generation, playback, and export with an explicit status
message; previously generated plots remain visible as the **last valid run**.
Car lengths remain read-only. GVT width and wheelbase are adjustable for every
predefined and custom target; catalogue values remain the defaults.
The controls consistently identify **GVT Length (m)**, **GVT Width (m)**, and
**GVT Wheelbase (m)** separately from the VUT vehicle configuration. All six
predefined wheelbases are populated from the catalogue and can be overridden
for simulation.

**Custom GVT** makes all three fields editable. Its initial dimensions come from
the existing vehicle configuration defaults, not a manufacturer reference.
Custom values are remembered separately when switching targets. Length and width
must be finite and positive. Wheelbase is optional (leave it blank if not known);
when entered, it must be finite, positive, and less than GVT length. It is stored
in the target model and exported to CSV. Because the GVT follows prescribed
body-centre motion, wheelbase alone does not change encounter timing or the
rectangular collision envelope; length and width determine those calculations.

Selecting a type regenerates encounter timing, initial longitudinal separation,
near-edge clearance, collision polygons, and the dimensionally scaled road target.
The user-entered GVT **centre-to-marker** offset is preserved: its near-edge
clearance changes with half the selected target width. Overlap remains a percentage
of **VUT width** for oncoming only (default 10%); infeasible oncoming overlaps
wider than the target are rejected. Overtaking has no requested overlap input.
Motorcycle runs are exploratory extensions of the ELK synchronization geometry,
not claims of Euro NCAP car-target protocol certification.

Headless runs select a predefined type or `custom` using `--gvt-type`. Motorcycle
types use the same formula-derived default lengths without requiring `--gvt-length`;
that option supplies a measured override. Fixed car dimensions cannot be
overridden. Omitting `--gvt-type` preserves the custom numeric
`--gvt-length`/`--gvt-width` API and CLI workflow. `--gvt-width` and
`--gvt-wheelbase` override the catalogue defaults for predefined targets as well.
Launching the GUI
with numeric dimensions and no named type selects Custom GVT. For Python callers:

```python
from lss_ncap import ElkParameters, SimulationEngine, target_type

target = target_type("c2_gvt")
length, width = target.dimensions()  # Bikes use estimates; pass confirmed_length_m to override.
parameters = ElkParameters(
    gvt_type_key=target.key, gvt_length_m=length, gvt_width_m=width)
run = SimulationEngine().run_elk("oncoming", parameters=parameters)

custom = ElkParameters(
    gvt_type_key="custom", gvt_length_m=3.8, gvt_width_m=1.7, gvt_wheelbase_m=2.4)
custom_run = SimulationEngine().run_elk("oncoming", parameters=custom)
```

Full traffic CSVs record the selected `gvt_type`, dimensions (including
`gvt_wheelbase_m`, blank if unspecified), manufacturer source
URL, and whether the length is `manufacturer`, `user_confirmed`, `custom`, or
`wheelbase_plus_half_wheel_diameter`. `gvt_length_wheel_diameter_m` records the
motorcycle diameter used for the catalogue estimate, even when the length is
overridden.

### Playback and charts

The road marking is 0.2 m wide at the
current display scale. Play, reset, full CSV export, compact SWA export, and bottom timeline scrubbing
are available. The timeline readout always shows the selected time, steering
command, NCAP reference, lateral speed, trajectory signals, and tracking error.
The road telemetry card also shows the current values throughout playback and scrubbing.
Scrubbing pauses playback; Play continues from the selected 25 ms sample.

Traffic telemetry shows **Initial centre dx**, the signed initial longitudinal
offset `GVT centre x - VUT body centre x`. Negative means the GVT centre starts
behind the VUT centre; positive means ahead. This is a road-frame X offset,
not a straight-line distance or bumper clearance, and stays fixed during playback.
The display converts the VUT front-axle coordinates using its axle-to-body-centre
offset and initial heading. Simulation coordinates, encounter synchronization,
and CSV coordinates remain unchanged.

Telemetry displays **d = d1 + d2 + VUT width/2**, the initial lateral distance
from the VUT centreline to the departure lane-marker centre (or boundary for
road-edge scenarios). It does not subtract half the marking width and is not
body-edge clearance. This value stays constant during playback and scrubbing,
and recalculates when geometry, speed, or VUT width changes. No distance label
or bracket is drawn on the road. Full CSV retains the per-sample signed
body-edge clearance `vut_lane_marker_clearance_m` for compatibility; this is a
different measurement from d.

Steering-wheel angle, lateral velocity, and lateral trajectory tracking are
shown in separate synchronized chart panels. The steering chart shows the
command applied over each following 25 ms model interval and a separate NCAP
reference. The lateral-velocity chart compares **Vehicle state** (solid yellow)
with **NCAP Reference** (dashed green). Both use the same time axis and signed
road-frame lateral velocity in m/s (positive left, negative right).
The reference is calculated directly as `v * sin(reference_heading(v * t))`:
zero during the straight approach, increasing in magnitude through the arc,
then constant along the straight departure. Radius and speed set the arc's
timing and slope; d2 sets the departure duration. The reference does not use
the vehicle's position, EPS response, or Pure Pursuit commands. The timeline
readout compares actual and reference lateral velocity at the selected time.
The
trajectory chart overlays the scheduled NCAP reference lateral position and
the simulated vehicle's front-axle lateral position. The selected frame's
tracking error appears in the timeline readout. The header's
Light/Dark Mode control changes the complete dashboard palette immediately and
does not reset playback, the selected scenario, or the current timeline frame.
Light mode uses a high-contrast white workspace, distinct control surfaces,
dark axis text, stronger grid lines, and signal colors selected for readability
against the plot background.
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

Every chart has an independent interactive viewport. Scroll over the plot to
zoom around the pointer, drag with the primary mouse button to pan, and
Shift-drag or right-drag to draw a zoom rectangle. Click a legend signal to
hide or show only that signal; hidden entries are muted and crossed out. Plot
hover does not reveal values. The bottom timeline slider is the sole signal
inspector, and its vertical cursor marks the same selected time on all plots.
Each chart has a visible **Reset View** button that auto-fits its axes while
keeping signal visibility. The dashboard **Reset View** resets all three axes.
View changes affect presentation only; simulation samples and exported data
are unchanged. Use **Expand** on the road view or any chart to fill the viewer
with that panel; **Restore** or Escape returns to the dashboard. Zoom, pan,
and signal visibility remain intact across expansion and restoration. The
timeline stays available while a panel is expanded.

The ego vehicle uses a transparent blue top-down sprite, while oncoming and
overtaking GVT traffic uses a separate dark-green sprite derived from the
provided references. Frames are scaled to the configured physical body length
and width and rotated to the simulated heading; transformed frames are cached
at 0.1° resolution for smooth, lightweight playback. A deterministic vector
model remains available as a non-Tk/test fallback. Collision detection continues
to use the same validated physical vehicle footprint.

## ELK traffic scenarios

All ELK cases use the same closed-loop Pure Pursuit controller and kinematic
bicycle vehicle model and commanded EPS response as LKA. Custom ELK uses
the same input-derived straight/arc/tangent geometry, not a protocol table.
Road Edge is
passenger-side only, Oncoming and Overtaking are driver-side only, and Solid
Line supports both sides.

### Inputs and nominal synchronization

The traffic controls expose lateral velocity (m/s), d2 (m), VUT/GVT speeds
(km/h), GVT marker distance (m), and, for oncoming only, target overlap (%).
The overlap field is hidden for overtaking. Radius and approach time remain
editable. **Oncoming overlap defaults to 10% of VUT body width**, not GVT
width. The GVT marker distance is explicitly its **centre path to the inner
edge of the centre dashed marking**, measured towards the adjacent lane,
as in LSS v4.2 §7.2.4.3.1/§7.2.4.4.1 (default 1.5 m).
VUT front-axle coordinates start at (0, 0); the marker coordinate is derived
from d1 + d2 + half VUT width. GVT coordinates are solved, not fixed.

Synchronization uses the independently scheduled reference path and rotated
VUT footprint, and the GVT's separately configured length and width:

- **Oncoming:** solve the reference station giving the requested projected
  lateral overlap, then align the vehicles' front edges longitudinally
  (§7.2.4.3.2). GVT motion is opposite to the VUT.
- **Overtaking:** solve first physical contact on the scheduled no-reaction
  trajectory while aligning the GVT leading edge with the VUT rear axle
  (§7.2.4.4.2). The rotated VUT polygon is clipped to the GVT's longitudinal
  span, so contact is solved at the overlapping part of the body, not at the
  VUT's forwardmost lateral corner. There is **no fixed/requested lateral
  overlap**. GVT motion has the same direction. Its speed
  must be at least the VUT's departure longitudinal speed and the computed
  initial GVT centre must be behind the VUT centre.
- In both cases, `GVT_initial_x = GVT_encounter_x - signed_GVT_speed * encounter_time`.
  The reference tangent is extended as necessary; there is no fixed baseline
  position or fixed five-second limit on encounter timing.

The protocol's 10% overlap is specified for oncoming only. Overtaking does not
use `ElkParameters.overlap_percent`; CLI `--overlap` is oncoming-only.
Arbitrary speeds, radii, dimensions, and
overlaps are **not a claim of protocol compliance**. The official source is
[Euro NCAP LSS Test Protocol v4.2, §§7.2.2–7.2.4](https://cdn.euroncap.com/cars/assets/euro_ncap_lss_test_protocol_v42_bfc543c53f.pdf).

GVT length and width default to the application's default vehicle dimensions,
but are independent editable inputs. No ISO target dimensions are inferred from
the protocol. Rendering, synchronization, TTC, and collision detection all use
the same configured VUT/GVT dimensions.

### Reference, no-intervention run, and controlled run

The viewer intentionally has **no built-in emergency intervention algorithm**:
it displays **No ELK intervention**, using the existing Pure Pursuit/vehicle
simulation. The nominal no-reaction encounter is calculated from the scheduled
path, independently of Pure Pursuit tracking error. Orange dashed vehicle
outlines (and an oncoming overlap bracket) show that nominal geometry, not a
prediction that the controlled vehicle will occupy those positions.
Encounter timing is displayed in the telemetry strip, without a floating
information panel over the road. If first contact stops the run before the
nominal time, telemetry shows the time difference. Both vehicles stay
at their actual impact positions; they are never moved or jumped into the orange
outlines. Overtaking displays Nominal Encounter Time, Actual Collision Time,
and Longitudinal Synchronization Error. Red polygons show actual collision
footprints, using the same geometry as collision detection; a red/white marker
identifies the first contact point.

Programmatic callers can supply `lane_support=callback` to `run_elk`.
The existing robot-release and commanded EPS behavior are retained. The actual
trajectory and collision test then use the controlled VUT; nominal geometry
and GVT initial positioning remain unchanged. A dashed magenta **No-ELK VUT**
trail shows the companion no-intervention simulation for comparison.
The nominal alignment time and actual first polygon-contact time are separately
calculated: Pure Pursuit tracking error or an external ELK controller can change
the actual event without changing reference geometry. For the default 72/72 km/h,
0.4 m/s, 1200 m radius, 1.5 m GVT offset, and 4.957 x 2.013 m vehicles with a
2.95 m VUT wheelbase (automatically centred d2 and approach = 5 s), overtaking's nominal
encounter is 9.830827 s and actual contact is approximately 9.830864 s.
At 72/80 km/h, actual contact is approximately 9.830860 s. Nominal leading-edge/
rear-axle synchronization error is zero to numerical precision in both cases.
The tiny actual timing difference is measured, never forced to zero.

The lateral-velocity chart compares Vehicle State with NCAP Reference in both
modes, with `v_ref_lat(t) = v * sin(path_heading(v * t))`. The reference does
not use the actual vehicle state, EPS delay, or steering commands.

Initial and live GVT coordinates appear in the UI and
the full `target_x_m`, `target_y_m`, `target_yaw_deg`, and `target_speed_mps`
series is included in traffic-scenario CSV files. Steering command, applied
angle, and the rest of the steering profile remain exported for every mode.
Custom exports also include target dimensions, requested oncoming overlap (blank
for overtaking), nominal
overlap in metres, nominal reference VUT/GVT poses, synchronization time,
and intervention-active state. `first_contact_x_m` and `first_contact_y_m`
store the actual polygon contact point; actual and nominal times remain distinct.
Overtaking `nominal_overlap_m` is the resulting road-frame projection overlap,
not a requested percentage or penetration depth.

The Python API accepts `run_elk(..., parameters=ElkParameters(...))`, exported
from `lss_ncap`. VUT speed comes from `SimulationConfig.speed_mps`; parameter
names document the GVT input units. Explicit `parameters=None` restores legacy
table-based paths and simulated-baseline synchronization. The corresponding
CLI option is `--headless --mode elk --elk-protocol-tables`; it cannot be
combined with custom geometry. Legacy `target_motion` enums are used only
with that compatibility mode.

Traffic scenarios use continuous oriented-rectangle collision detection between
25 ms output samples. SAT tests use rotated polygons, not centres or bounding
boxes. A motion-bounded interval search detects even complete passes between
nonintersecting endpoints and refines the earliest contact to a sub-nanosecond
sample interval. This accuracy applies to linearly interpolated sample poses,
not a claim of sub-nanosecond physical-model accuracy. Both
vehicles are stopped at that contact pose, and the run is truncated immediately.
The timeline therefore ends at impact and displays `COLLISION DETECTED`; no
post-impact samples are generated in which the vehicles overlap or pass through
one another.
Custom runs default to collision stopping rather than early protocol safety
termination; `safe_termination=True` can still request the existing §7.4.7
threshold logic programmatically.

`v_lat_mps` in CSV is road-frame lateral speed. `body_lateral_speed_mps` is the
vehicle-body lateral component; it can approach zero while a yawed car is
moving sideways across the road.
`reference_v_lat_mps` is the scheduled NCAP reference lateral velocity shown
in the velocity plot. The legacy `steering_area_deg_s` and
`v_lat_from_steering_mps` columns remain available for CSV/API compatibility,
but the SWA area estimate is no longer plotted or shown in the timeline readout.

## Vehicle configuration and steering profile

The viewer's grouped **Vehicle Configuration** panel edits wheelbase, length,
width, and steering ratio. VF8 NP REEV defaults are 2.950 m, 4.957 m, 2.013 m,
and 17.86 respectively. Press Enter or leave a field to regenerate the run.
In shorter windows the panel uses a compact inline layout so the ELK traffic
inputs and road remain visible; resizing taller restores the full grouped layout.
Wheelbase sets the kinematic bicycle geometry; body length and width change
rendering and collision geometry. Vehicle mass remains internal at 2485 kg by
default; the kinematic bicycle equations do not depend on it.
LKA geometry is configurable, not selected from a baseline test case. **R (m)**
is the circular reference arc's radius; **d2 (m)** is the additional lateral
distance along the straight departure after that arc. The default radius is
1200 m; d2 is calculated so the VUT is centred in the configured lane, with a
1.0 m fallback when that calculation is not possible. Radius must be finite and positive;
d2 must be finite and nonnegative (zero omits the tangent before the configured
departure tail). LKA accepts any finite lateral-speed target strictly between
zero and the configured vehicle speed, not just protocol table speeds.

For vehicle speed `v` and target lateral speed `vlat`, geometry is calculated as:

```text
psi = asin(vlat / v)
d1 = R * (1 - cos(psi))
arc length = R * psi
tangent length = d2 / sin(psi)
lane-marker offset = d1 + d2
automatic d2 = (lane width - VUT width) / 2 - d1
```

The departure side mirrors the geometry without changing these distances.
Road placement uses the calculated d1 + d2 offset, plus half the configured
vehicle width. Custom geometry is not a claim of protocol-table compliance.
For reproducing exact legacy Euro NCAP LSS v4.2 §7.2.3 rows, explicit
`--headless --ncap-profile standard` and
`--headless --ncap-profile alternative_dim` remain available. Table profiles
cannot be combined with geometry overrides; configurable ELK retains protocol
departure-side conventions. The Python `SimulationEngine.run_lka` and compatibility `generate`
functions accept keyword arguments `radius_m` and `d2_m`.
The checked-in `lka_profile.csv` uses the configurable defaults (vlat = 0.4 m/s,
R = 1200 m, automatically calculated d2 = 0.503476 m). It contains 360 samples.
Changing these settings updates the road view and all three plots. Internally,
the controller uses `max(minimum distance, speed * look-ahead time)` with fixed
viewer defaults of 8 m and 0.5 s (10 m at 72 km/h).

The lateral-speed, radius, and d2 inputs shape the NCAP path. At each 25 ms sample, the
scheduled path state is evaluated at distance `speed * time`, yielding
`reference_x_m`, `reference_y_m`, heading, and geometric curvature. The NCAP
arc begins at the configurable approach time, **5.0 s** by default. At each
25 ms control tick, Pure Pursuit reads the
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
| `main.py` | CLI and import compatibility entry point |

The simulator generates the NCAP approach path and steering profile. A
production lane-support controller can still be connected through
`SimulationEngine.run_elk(..., lane_support=callback)`; without that callback,
the scheduled profile continues driving the vehicle so the scenario input and
vehicle encounter can be inspected independently.
