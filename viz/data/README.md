# Web-app dataset (`viz/data/`)

> **Final preset refresh, 2026-10-17:** PERFORMANCE (eco_weight 0, lat_accel_frac 0.75) and ECO (0.8/0.75) for all five public courses and seeds 1-5 now come from the saved serial e3b8a28 validation telemetry. No laps were rerun. Each run has `code_commit` and `evidence`; other sweeps/baselines retain the historical bad18cd export. The top-level `commit` identifies the final preset code, not every historical run. Extra preset metrics retain overruns, budget-guard trips, peak rear slip, path error and obstacle hits. See [final validation](../../docs/STATUS.md).

Static JSON for the demo web app, which replays our laps step by step and compares driving modes and
parameters. The app cannot run Python, so every number here is precomputed by `tools/export_viz.py` and
fetched from `https://raw.githubusercontent.com/rmaytr/HackUAB/main/viz/data/`.

Regenerate it from `simulator/` (about 3 minutes with 4 parallel laps):

```powershell
.venv\Scripts\python.exe ..\tools\export_viz.py --jobs 4
.venv\Scripts\python.exe -m pytest ..\tests\test_viz_export.py
```

**This layout is a contract.** The app is built against these key names. `schema_version` changes whenever a key is
renamed or removed. A new key can be added without a version bump, so the app should ignore keys it does not know.
`tests/test_viz_export.py` checks every rule on this page.

## Conventions

- Units: metres, seconds, m/s, m/s², radians, Wh and kW.
- Positions use the **world frame**: ENU, x east, y north, yaw counter-clockwise from +x. These are the same
  coordinates the simulator uses, so a run's `x`/`y` overlay directly on its course's geometry.
- Numbers are rounded to 3 decimals (track geometry to 0.01 m).
- `null` means "not available". It can be a whole series (for example the baseline controller logs no `dbg_*` timings,
  so its `t_plan_ms` is an array of `null`) or a single value (for example `lap_time_s` of a lap that did not finish).
- Only the bicycle backend appears here. CARLA is half of the official score, but this dataset contains no CARLA laps.

## `index.json`: start here

| Key | Meaning |
|---|---|
| `schema_version` | `1` |
| `generated_at`, `commit` | UTC time of the latest export refresh and the code commit for the final PERFORMANCE/ECO presets; use each run's `code_commit` for historical sweeps/baselines |
| `backend_note` | One sentence on what was simulated |
| `assumptions` | `eta_drive`, `eta_regen` and `p_regen_max_kw` used for the energy figures. **They are assumed values for a generic EV, not Blazer EV specifications.** The simulator has no battery or regen, so energy is rebuilt from the wheel forces in telemetry. |
| `param_axes` | Every value in the grid: `eco_weight` (0 = fastest lap, 1 = most energy-saving) and `lat_accel_frac` (the fraction of tyre grip the planner uses for cornering). The eco_weight sweep and both presets use the `hackuab/params.py` default (currently 0.75); the other values are run at eco_weight 0 only. |
| `presets` | The two driving modes as parameter values: `performance` = `eco_weight 0` (the scored submission), `eco` = `eco_weight 0.8` |
| `courses` | One entry per course (see below) |
| `runs` | One entry per lap (see below) |
| `failures` | Laps that crashed or timed out during the export: `id`, `course`, `mode`, `params`, `error`. They have no run file. |

**`courses[]`**: `id`, `name`, `kind` (`"public"` for the released practice courses, `"random"` for `gen-track`
courses with seeds 1–5 and 3 obstacle groups), `length_m` (centerline length), `reference_time_s` (the
scorer's reference lap time), `closed` (a loop instead of point-to-point), `laps` and `file` (`tracks/<id>.json`).

**`runs[]`**:

| Key | Meaning |
|---|---|
| `id` | `<course>__baseline` or `<course>__<mode>__ew<eco_weight>__laf<lat_accel_frac>`, e.g. `practice_technical__eco__ew0.80__laf0.75` |
| `course` | A `courses[].id` |
| `mode` | `"baseline"` (the organisers' pure-pursuit example controller), `"performance"` (ours, eco_weight 0) or `"eco"` (ours, eco_weight > 0; 0.8 is the eco preset) |
| `backend` | `"bicycle"` |
| `params` | `{"eco_weight", "lat_accel_frac"}`. Both are `null` for the baseline. |
| `file` | `runs/<id>.json`, the time series |
| `metrics` | See below |

**`runs[].metrics`**:

| Key | Meaning |
|---|---|
| `finished`, `result` | Whether the lap finished, and `"finished"` or `"dnf: <reason>"` |
| `lap_time_s` | Start line to finish line (`null` on a DNF) |
| `adjusted_time_s` | Lap time plus the scorer's time penalties for cones, gates and off-course |
| `technical_points` | The harness score out of 75 for this one lap |
| `category_scores` | `completion_time`, `deviation`, `compute`, `dynamics`, each 0–100 |
| `penalties` | Counts: `cone_hits`, `off_course`, `missed_gates`, `controller_errors` |
| `p99_step_ms` | 99th-percentile controller step time (the deadline is 50 ms) |
| `energy_wh`, `wh_per_km` | Estimated battery energy for the whole run (traction ÷ `eta_drive` − regen) and per km driven |
| `peak_power_kw` | Highest tractive power at the wheels |
| `rms_jerk` | RMS of d(ax)/dt in m/s³ (ride smoothness) |
| `energy_breakdown_wh` | Where the energy went: `drag`, `rolling`, `brake_dissipated` (all braking work at the wheels), `regen_recovered` (the part returned to the battery under the assumptions). Also `tyre_scrub` (cornering slip loss) and `drive_loss` (drivetrain loss at `eta_drive`). |
| `energy_balance_residual_pct` | How well the energy books close, in % of traction energy. The meter is trusted only below 5 %; typical values are around 0.05 %. |

## `tracks/<course>.json`: course geometry (world frame)

| Key | Meaning |
|---|---|
| `centerline`, `left_boundary`, `right_boundary` | `[[x, y], ...]` resampled to about 1 m. On closed courses the first point is repeated at the end. |
| `cones` | `left` (blue), `right` (yellow), `obstacle` (orange cones the car only sees through perception) and `markers` (orange start/finish markers), each `[[x, y], ...]` |
| `start_line`, `finish_line` | `[[x, y], [x, y]]`: the left end, then the right end |
| `gates` | `[[[x, y], [x, y]], ...]`: scored gates every 20 m, in driving order |
| `bounds` | `xmin`, `xmax`, `ymin`, `ymax` around the boundaries and cones, for fitting the view |

## `runs/<id>.json`: one lap as columns, sampled at 10 Hz

`schema_version`, `id`, `course`, `dt` (0.1) and these arrays, all the same length (index `i` is one instant):

| Key | Meaning |
|---|---|
| `t` | Simulation time, s (the car starts from rest at t = 0, and timing starts at the start line) |
| `x`, `y`, `yaw` | Pose in the world frame |
| `speed`, `target_speed` | Actual speed and the speed the controller is aiming for, m/s (`target_speed` may be `null`) |
| `throttle`, `brake` | Applied pedals, 0–1, never both above 0 for our controller |
| `steer` | Applied steering, −1 (full right) to +1 (full left) |
| `ax`, `ay` | Longitudinal and lateral acceleration in the vehicle frame, m/s² |
| `progress` | Distance along the course from the spawn point, m |
| `path_err` | Distance between the car and the path it published 0.5 s earlier, m (the deviation score) |
| `energy_wh_cum` | Battery energy used since t = 0, Wh. It can go down while regen braking. The last value equals `metrics.energy_wh`. |
| `coast` | 1 while Eco mode is deliberately coasting (lift-and-coast before a braking zone), else 0. All `null` for `performance` and `baseline` runs, which do not log it. |
| `fallback` | 0 on a normal step. 1 means the planner failed and the controller drove a centerline plan instead. 2 means the tracker failed (or the road model was invalid) and it used the last-resort pure pursuit or braked. |
| `t_horizon_ms`, `t_plan_ms`, `t_track_ms`, `t_total_ms` | Our controller's time per stage (corridor, racing line + speed profile, tracking) and in total, ms. All `null` for the baseline. |

The final sample is recorded after the last control step, so its controller-logged values (`coast`, `fallback`, `t_*_ms`) are `null`.

`events`: `[{"t", "kind", "x", "y"}]` from the race monitor. `kind` is one of `start`, `finish`, `cone_hit`,
`missed_gate`, `off_course` and `dnf`.
