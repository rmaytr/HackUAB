# HackUAB sensor fusion — ready-to-test starter for Saisi and Viraj

**Updated:** 2026-10-17  |  **Status:** synthetic fusion working and unit-tested; full HackUAB controller and real CARLA capture must be validated on the team's machines.

**Repository:** https://github.com/rmaytr/HackUAB  
**Organizer submodule (do not edit):** `simulator/` / https://github.com/dszurek/ecocar-hackathon-sim  
**Code lives in:** `fusion/` outside the scored `hackuab/` folder.

## What this gives you

1. `fusion/schema.py`: common, versionable `.npz` FrameBundle: vehicle-coordinate LiDAR point cloud, calibrated forward-facing RGB image, optional radar polar detections, timestamps.
2. `fusion/synthetic.py`: generates synthetic LiDAR/RGB/radar observations of a straight cone-bounded course for **30 seconds / 600 frames** with a 20 Hz control-time sequence. **No CARLA or Windows needed.**
3. `fusion/fuse.py`: ground filter; grid-connected cone-sized LiDAR clusters; project centroids onto camera and classify colored patches; nearest-neighbor radar association; heuristic confidence and per-stage timings.
4. `fusion/bridge.py`: convert classified clusters to real organizer `ConeDetections` and project orange detections onto an actual `RoadModel` centerline as `ObstacleInfo` objects. `inject_fusion` returns a new `Observation` and `RoadModel`, leaving originals untouched. `label_from_road_prior` is an explicitly disclosed *offline* way to classify same-looking CARLA props using an existing RoadModel.
5. `fusion/demo.py`: benchmark 30 seconds of synthetic frames, save a comparison PNG and `.npz` replay, and optionally run **one static snapshot through the real complete HackUAB controller** (`horizon` + `planner`/`predict` + `tracker` + `controller`).
6. `fusion/animate.py`: make a two-panel camera and bird's-eye-view sensor GIF.
7. `fusion/carla_capture.py`: **optional unverified-on-hardware** sidecar to attach three sensors to an active CARLA vehicle and record synchronized `.npz` frames without advancing the world.
8. `tests/test_fusion_starter.py`: 10 offline tests, including synthetic detection, radar missingness, color uncertainty, frame saving, cone adapter, curved-centerline obstacle geometry, and CARLA class safety.

**Do not represent this as a scored controller improvement.** Organizers already provide processed `obs.road` and `obs.cones` (and no raw LiDAR/RGB/radar) inside the official sandbox. The bridge operates **offline**, for an innovation demonstration. Only files inside `hackuab/` are submitted for technical scoring; the fusion prototype cannot be imported by the scored controller.

## Saisi: run this on macOS in 5 minutes

First, save your current local work, confirm you're in `~/HackUAB`, and choose a safe feature branch. This ZIP is intended to be extracted **into your existing HackUAB repository root**, not into `simulator/`.

```bash
cd ~/HackUAB
git status --short --branch
# If your worktree is clean, and no branch of the same name exists:
git switch -c saisi/fusion-starter
# Put HackUAB_Fusion_Starter.zip in ~/Downloads, then:
unzip -n ~/Downloads/HackUAB_Fusion_Starter.zip -d .

# Validate core detector and bridge:
simulator/.venv/bin/python -m pytest tests/test_fusion_starter.py -q

# Benchmark a genuine 30-second synthetic timeline (600 frames) and make a PNG:
simulator/.venv/bin/python -m fusion.demo --frames 600 --png fusion_demo.png

# Optional: make an easy-to-share animation (Pillow is available with matplotlib):
simulator/.venv/bin/python -m fusion.animate --frames 120 --stride 3 --out fusion_demo.gif

# IMPORTANT: test on your ACTUAL up-to-date HackUAB controller once:
simulator/.venv/bin/python -m fusion.demo --frames 20 --full-controller --taper 14 --png fusion_full_controller.png
```

`--taper 14` is an **offline override for the test**. It does not edit `params.py`, and does not establish that 14 m is the best general taper. You can omit `--taper` to use the current `Params` default.

The full-controller one-step test uses a **synthetic straight RoadModel**, so it demonstrates wiring of the *actual* `HackUABController.step()` without driving an actual lap. It is not a closed-loop evaluation and is **not** a substitute for CARLA or bicycle `selftest`.

`unzip -n` skips pre-existing files, avoiding accidental overwrites. If the team has created its own `fusion/` files in the meantime, inspect and merge explicitly instead of overwriting them.

## Viraj: test on Windows, without building fusion from scratch

Pull the **fusion feature branch** after Saisi pushes it, or unpack the ZIP into your HackUAB repository root on a clean branch. No work inside `simulator/` should change.

From PowerShell at `HackUAB\`:

```powershell
simulator\.venv\Scripts\python.exe -m pytest tests\test_fusion_starter.py -q
simulator\.venv\Scripts\python.exe -m fusion.demo --frames 600 --png fusion_demo.png
simulator\.venv\Scripts\python.exe -m fusion.demo --frames 20 --full-controller --taper 14 --png fusion_full_controller.png
```

### Optional CARLA recording

Requirements: Windows or Linux; CARLA server 0.9.16 and compatible CARLA Python package; RGB requires off-screen **rendering**, not CARLA no-render mode. The recorder uses a level forward-facing sensor mount with known fixed positions; if you change mounting yaw/pitch/roll, **recalibrate** the projection. We have **not run this sidecar against CARLA yet**.

Terminal 1, inside `simulator/`:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_carla.ps1 -Headless
.venv\Scripts\python.exe -m autocross check
.venv\Scripts\python.exe -m autocross run -c ..\hackuab\controller.py -t tracks\practice_technical.yaml --backend carla --realtime
```

Terminal 2, from **HackUAB root**, start capture while the CARLA run is active (or start it a few seconds before, so it waits for the vehicle):

```powershell
simulator\.venv\Scripts\python.exe -m fusion.carla_capture --out fusion_capture --seconds 45 --max-frames 80
```

Then analyze one captured `.npz` file from the HackUAB root:

```powershell
simulator\.venv\Scripts\python.exe -m fusion.demo --recorded fusion_capture\frame_XXXXXXXX.npz --png carla_fusion.png
```

Replace `frame_XXXXXXXX.npz` with the actual filename. **Do not use `--full-controller` with an arbitrary recorded CARLA frame**: a matched real `RoadModel`/`Observation` from that simulation step is required. The supplied demo deliberately refuses this unsafe comparison.

**CARLA semantic-color warning:** The organizer's CARLA configuration uses `static.prop.constructioncone` for left, right, and obstacle cone roles. They may all *look the same*. A raw RGB orange pixel does **not** prove an object is an unmapped orange obstacle. Therefore `fuse_frame` marks semantic color `-1` (unknown) on CARLA frames by default. If a time-aligned RoadModel is available, `bridge.label_from_road_prior(fused, road)` can infer the probable role from boundary positions; such a demo is **road-prior-assisted**, not pure camera classification.

The optional radar associates a **radial relative velocity** to a cluster when a return is nearby. It is not a complete 2D object-velocity tracker. Radar may miss small stationary cones. The simple color classifier only handles synthetic/visibly color-coded cones and is not a neural detector.

## Integration contract and rules

| Stage | Input/output | File owner |
|---|---|---|
| Sensor fusion | FrameBundle -> list of `(xy, class, radial_velocity, confidence)` | New `fusion/` tool |
| Adapter | Classified list -> `ConeDetections` and `RoadModel.obstacles` (new `Observation`) | `fusion/bridge.py` |
| Corridor | `build_corridor(obs.road, vehicle, params)` -> `Corridor` | Saisi: `hackuab/horizon.py` |
| Racing line | `plan(corridor, obs, vehicle, params)` -> path and speed profile; invokes verifier | Rodion: `hackuab/planner.py` |
| Vehicle prediction | Dynamic AWD rollout/verification | Rodion: `hackuab/predict.py` |
| Tracking | `track(plan, obs, vehicle, params, state)` -> steer/throttle/brake | Viraj: `hackuab/tracker.py` |
| Entry point | `HackUABController.step(obs)` -> `Command` including `path` and diagnostics | Viraj: `hackuab/controller.py` |

Coordinate with Rodion/Viraj about the reported centerline fallback on invalid corridors before changing any scored safety behavior. This prototype **does not** fix the fallback or the code's cone-taper selection.

Coordinate file ownership: `CLAUDE.md` originally assigned `fusion/` to Saisi, but Viraj has taken the CARLA/testing side. Avoid conflicting changes and use a small pull request.

## Local verification already completed

Executed in the code-generation environment (Python 3.13 with NumPy; **not** on the team's CARLA server or Mac):

- `python -m pytest tests/test_fusion_starter.py -q`: **10 passed**.
- `python -m fusion.demo --frames 600`: median **~1.7 ms** CPU fusion, p95 **~2.1 ms**, max **~6.5 ms** on this environment; synthetic exact-color match within 0.5 m: **6489/7101 (~91%)**; median synthetic position error **~0.015 m**.
- `python -m fusion.animate --frames 90 --stride 3 --out fusion_demo.gif`: a GIF was generated.

**These metrics are for the deliberately simple synthetic generator.** They are not real sensor accuracy, not official benchmark performance, and not a measured increase to technical scoring. Additional CPU timings on Mac/Windows and CARLA validation are needed.

## Best next tasks (in priority order)

1. Saisi: run the ten tests and 600-frame benchmark on Mac, then the offline `--full-controller` single-observation smoke test on the real repo. Report a traceback if there is an integration mismatch.
2. Saisi: push only these new fusion files on a separate branch/PR. Do not add videos, recordings, `.venv`, `runs`, or keys to Git. Stage explicitly:
   `git add fusion tests/test_fusion_starter.py FUSION_HANDOFF.md`.
3. Viraj: when his PC frees up, run the same tests and validate a recorded CARLA `.npz` frame. Repair any CARLA callback/extrinsic mismatch before trusting measurements.
4. Viraj/Rodion: for an end-to-end CARLA replay, save **time-aligned RoadModel/Observation** from the driving run, apply `inject_fusion` (and any disclosed road-prior classification) offline, and compare full-controller outputs. Do not infer course geometry from these raw readings unless that subsystem is actually implemented.
5. Pitch: use side-by-side camera/BEV visualization with stage latency and state clearly which portions were synthetic versus real CARLA.

**Code freeze discipline:** This is a judge-facing innovation prototype. Don't jeopardize the already-scoring controller by importing it or changing the official simulator. Run complete official tests before shipping any changes to `hackuab/`.
