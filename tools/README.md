# Tools

Development and analysis scripts. None of them are part of the submitted controller (`hackuab/`). Unless noted, run
them from `simulator/` with the simulator's Python: `.venv\Scripts\python.exe ..\tools\<script>` on Windows,
`.venv/bin/python ../tools/<script>` on Linux/macOS. Each script's docstring has its full usage.

## Run and score

| Script | Purpose |
|---|---|
| [`run_mode.py`](run_mode.py) | Run one lap or the official `selftest` in a driving mode: `run_mode.py performance\|eco run <course>\|selftest` |
| [`strict.yaml`](strict.yaml) | Controller config that re-raises module errors instead of falling back (development) |
| [`eval.ps1`](eval.ps1) | Run a controller on the technical course plus random courses (PowerShell) |
| [`par_robust.py`](par_robust.py) | Parallel robustness sweep (perception noise, latency, low grip) using the `robustness_*.yaml` configs |
| [`par_sweep.py`](par_sweep.py) · [`sweep.py`](sweep.py) | Parallel / serial parameter sweeps over the evaluation courses |
| [`carla_remote_setup.sh`](carla_remote_setup.sh) | One-shot headless CARLA setup on a cloud GPU VM (Ubuntu) |

## Analyse a run

| Script | Purpose |
|---|---|
| [`runs_table.py`](runs_table.py) | One line per run: time, points, path error, category scores |
| [`points_gap.py`](points_gap.py) | Where points are lost, per scoring category |
| [`compare_runs.py`](compare_runs.py) | Where along the course one run gains or loses time against another |
| [`events.py`](events.py) | A run's penalty events and where they happened |
| [`slip.py`](slip.py) | Rear-slip episodes above the scorer's 6° threshold |
| [`stage_times.py`](stage_times.py) | p50 / p99 time of each pipeline stage |
| [`tele.py`](tele.py) | Print a window of telemetry |
| [`predict_replay.py`](predict_replay.py) | Cross-check the physics model (`predict.py`) against simulator telemetry |

## Energy and ECO mode

| Script | Purpose |
|---|---|
| [`energy.py`](energy.py) | Energy meter: reconstructs wheel forces from telemetry and reports Wh, Wh/km and the breakdown |
| [`eco_pareto.py`](eco_pareto.py) | Sweep `eco_weight` and build the lap time vs energy Pareto front |
| [`eco_figures.py`](eco_figures.py) | ECO demo figures from a Pareto sweep |

## Figures and the visualizer dataset

| Script | Purpose |
|---|---|
| [`make_readme_figures.py`](make_readme_figures.py) | Regenerate every figure in `docs/img/` from `viz/data/` (run from the repository root) |
| [`export_viz.py`](export_viz.py) | Run the lap grid and export the JSON dataset in `viz/data/` for the web visualizer |
| [`plot_run.py`](plot_run.py) · [`plot_laps.py`](plot_laps.py) | Per-course and per-selftest comparison figures against the baseline |

## Development harnesses

[`tracker_dev_controller.py`](tracker_dev_controller.py) and [`corridor_pp.py`](corridor_pp.py) are stand-in
controllers that test the tracker and the corridor in isolation.
