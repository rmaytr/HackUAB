# ROADMAP: HackBama_01 EcoCAR Autocross (goal: 1st place)

Written 11:20 AM. **Code freeze 5:00 PM.** Demo 3–5 min. Read [CLAUDE.md](../CLAUDE.md) first, because it has
the API, rules and module contracts. This file says **who does what, when, and how**.

## Where we stand

| | Baseline (to beat) | Our target |
|---|---|---|
| `practice_technical` lap (bicycle) | 39.47 s (reference 31.60 s) | **≤ 33 s** |
| Cone hits / missed gates | 4 (all unmapped obstacle cones) / 0 | **0 / 0** |
| p99 step time | 1.43 ms | **< 15 ms** (we have room for physics prediction) |
| Technical points | 57.5 / 75 | **≥ 68 / 75** |
| CARLA vs bicycle lap difference | n/a | **< 2.5 %** |

Where the points are: the technical course is 60 % of the score, and Innovation (25) is judged.
So the order is **clean > fast > clever-and-explained**. The 4 obstacle cones alone cost the baseline 8 points.

## Team & ownership

| | Rodion | Saisi | Viraj |
|---|---|---|---|
| **Owns** | `hackuab/planner.py`, CARLA, `hackuab/predict.py` (physics model) | `hackuab/horizon.py`, `fusion/` (sensor-fusion track) | `hackuab/controller.py`, `hackuab/params.py`, `hackuab/tracker.py`, `tools/` |
| **Theme** | Racing line, speed profile, physics-based prediction | Corridor + obstacles, then lidar/radar/camera fusion | Integration, control, testing harness, demo visuals + slides |
| **Branch** | `rodion/*` | `saisi/*` | `viraj/*` |

`docs/` (README, pitch, write-ups) is shared: one person edits a file at a time, and you announce it in the chat.

---

## Timeline & checkpoints

At every checkpoint: 10-minute standup, merge green branches into `main`, run `selftest` on `main`.

| Time | Phase | Exit criteria |
|---|---|---|
| **11:20–11:50** | 0. Skeleton | `hackuab/` skeleton on `main` that reproduces the baseline and passes `selftest`. CARLA download started. Contracts frozen. |
| **11:50–1:30** | 1. MVP | Each module's MVP is merged. **0 cone hits on `practice_technical`** and faster than the baseline. |
| **1:30** | Checkpoint 1 | Lap time + penalties table on `practice_technical` + 3 random seeds |
| **1:30–3:30** | 2. Innovation | Physics prediction, min-curvature line, friction ellipse, fusion pipeline, first CARLA runs |
| **2:30** | Checkpoint 2 | Mid-point merge; decide what to cut (see cut list) |
| **3:30–4:15** | 3. Robustness | `batch` sweep, ≥ 5 random seeds, both backends, tuning only |
| **4:15** | **Feature freeze** | From here on: bug fixes, tuning, README, demo, slides |
| **4:15–5:00** | 4. Ship | Final `selftest`, submission folder, demo recording, slides, README |
| **5:00** | **Code freeze** | `main` tagged `submission`. |

---

## Phase 0: skeleton (11:20–11:50) · Viraj, with Rodion's Claude session doing the first pass

**What:** `hackuab/controller.py`, `params.py`, `horizon.py`, `planner.py`, `tracker.py`, `predict.py`, each
a working stub that follows the contracts in CLAUDE.md. Together they reproduce the baseline
(centerline + pure pursuit + curvature speed). Everyone then replaces their own stub without blocking anyone else.

**How:**
1. Port `simulator/controllers/baseline_pure_pursuit.py` into the module split: `horizon.build_corridor`
   returns the centerline corridor, `planner.plan` returns the centerline plus the curvature speed profile,
   and `tracker.track` does pure pursuit + PI control.
2. `controller.py` wraps every stage in `try/except`. On error, NaN or a stage over budget, it falls back to the
   baseline logic and sets `debug["fallback"]=1`. It times each stage into `debug`.
3. Copy the bicycle-model constants from `simulator/config/sim.yaml → vehicle.bicycle` into `params.py`
   (mass, wheelbase, cg_to_front, cornering_stiffness_per_load 14.0, mu 0.97, steer_time_constant 0.06,
   max_drive_force, max_brake_force, drag_area) and note where each came from. These are vehicle
   parameters, not course data, so this is allowed.
4. `selftest` passes and the lap time matches the baseline (±0.5 s). Push to `main`.

---

## Rodion: planner, physics-based prediction, CARLA

### R1. Get CARLA running (start the download NOW; it's about 20 GB unpacked)
**Hardware warning:** Rodion's laptop has **Intel Iris Xe integrated graphics (no NVIDIA GPU)**, and the harness is
tested on RTX 30-series cards. Plan for it:
- **Plan A:** if Saisi's or Viraj's laptop has an NVIDIA GPU, run the CARLA *server* there. Rodion runs the harness
  against it with `--set carla.host=<their LAN IP>` (ports 2000–2002 must be reachable; hackathon Wi-Fi may block
  them, so try a phone hotspot).
- **Plan B:** run on the Iris Xe with `scripts\start_carla.ps1 -Headless` (Low quality) and `--set carla.render=false`.
  It's slow but fine for scoring runs.
- **Plan C (decide by 2:30):** bicycle only. CARLA is half the official score, but the judges run it themselves.
  Our job is to make sure the car *behaves the same* there, so lean on the fitted bicycle model (`compare`).

**How:** download the CARLA 0.9.16 Windows release, unzip to `C:\CARLA_0.9.16`, then run
`scripts\start_carla.ps1 -Headless`, `python -m autocross check`, and a baseline run on `hackathon_test`.
Then run `compare -c ..\hackuab\controller.py -t tracks\practice_technical.yaml` to check our controller
is within 2.5 % on both backends.

### R2. Racing line: `planner.py`
**What:** a minimum-curvature line inside the safe corridor, re-solved every step over the 40 m horizon.
**How (numpy only, under 5 ms):**
1. Resample the corridor at uniform Δs ≈ 1 m (`np.interp` on `s`), giving about 40 points.
2. The unknowns are lateral offsets `n_i` along the normals, bounded by `[-right_i, +left_i]`.
3. Curvature is approximately linear in `n` for small offsets: κ ≈ κ_c + A·n, where A is the second-difference matrix / Δs².
   Minimise ‖κ_c + A n‖² + λ‖D n‖² + μ·(n_0 − current_offset)²: a small banded linear solve
   (`np.linalg.solve` on a 40×40 matrix takes about 50 µs). Clip to the bounds, then repeat 2–3 times re-solving with
   active bounds fixed (a cheap projected solve).
4. Smoothness across steps: warm-start from last step's offsets (shifted by distance travelled) so the published
   path doesn't jump. That protects the 15-point path-deviation score.
5. Horizon end: pull `n` back toward 0 over the last ~8 m, so we don't commit to an apex we can't see.

### R3. Speed profile + physics-based prediction: `planner.py` + `predict.py`
**What:** speeds the car can physically achieve, checked by simulating the car forward. This is our headline innovation.
**How:**
1. `v_max(s) = sqrt(a_lat / max(|κ|, ε))`, then a backward pass (braking `max_decel`, friction-ellipse
   reduced) and a forward pass (`max_accel`, also power-limited: `F = min(F_drive, P/v)`).
2. **Visibility guard:** at s = 40 m, cap v so we could stop or turn into a worst-case hairpin (R ≈ 8 m) just past
   the horizon: `v_end ≤ sqrt(2·a_brake·d_margin + a_lat·R_min)`.
3. **`predict.py`, a vectorised dynamic bicycle model** (same equations as `simulator/autocross/backends/bicycle.py`:
   linear tyres with saturation at μ·Fz, first-order steering lag, drag). The function is
   `rollout(state, plan, tracker_law, horizon=1.5 s, dt=0.05)` and returns the predicted poses, lateral offset,
   friction usage and yaw rate.
4. **Predictive verifier:** roll the car out along the plan with the tracker's control law. If the predicted friction
   usage is over 0.95 or the predicted car leaves the corridor anywhere, scale `v` down locally and re-run
   (at most 3 iterations, about 1 ms each).
5. **Stretch: sampling MPC.** 32 candidate (steer offset, accel) sequences × 30 steps evaluated in one vectorised
   rollout. Cost = time + corridor penalty + friction penalty. Pick the best one and fall back to the plan if none is safe.
6. Log `pred_friction_max`, `pred_offset_max`, `v_scale` and `n_iters` to `debug`. They make great demo plots.

**Done when:** 0 cones and 0 off-course on `practice_technical` + 5 seeds, lap ≤ 33 s, p99 < 15 ms.

---

## Saisi: corridor, obstacles, then sensor fusion

### S1. Safe corridor + obstacle avoidance: `horizon.py` (by 1:30; this is where the baseline lost its 8 points)
**How:**
1. Corridor bounds = `preview_width_left/right − (vehicle.width/2 + 0.3 m)`, clipped to ≥ 0.2 m.
2. For each `ObstacleInfo`, compute the free gap on each side: left gap = `left_bound − lateral_max`, right
   gap = `lateral_min + right_bound`. Pick the **wider** side (it must be at least the car width + margin, otherwise brake).
3. Shrink the corridor on the blocked side over `[distance − 6 m, distance + 4 m]` with a smooth ramp (cosine taper)
   so the planner gets a drivable funnel instead of a step.
4. Keep the choice **sticky** across steps (don't flip sides on noise): hysteresis, plus remembering the obstacle's
   position by transforming it with our own motion between steps.
5. Unit-test it on synthetic `RoadModel`s (python-testing-patterns skill): an obstacle left, right, centered, two in a row, and in a corner.

**Done when:** 0 cone hits on `practice_obstacles`, `practice_technical`, and 3 seeds with `--obstacles 4`.

### S2. Sensor-fusion track: `fusion/` (1:30–4:15; separate from the scored controller)
**Why:** the coordinator said their team is working on fusing **lidar + radar ("sensors") + camera** quickly and
cheaply, with good visualisation. Building a working version is the "beyond the brief" story that wins over judges.
*Confirm with her that the third sensor is radar, and ask what output they care about (object list? BEV map?).*

**Constraint:** the controller sandbox can't import CARLA or take raw sensors, so this is a **standalone tool**
next to the controller. Then we close the loop by converting its output into `ConeDetections` and feeding it to our planner.

**How:**
1. **One data format:** `FrameBundle = {t, ego_pose, lidar (N,4: xyz+intensity), radar (M,4: depth, az, alt,
   velocity), image (H,W,3), K, extrinsics}` saved as `.npz`.
2. **Two sources:**
   - *CARLA source* (if CARLA works): spawn the ego with `sensor.camera.rgb`, `sensor.lidar.ray_cast`
     (32 ch) and `sensor.other.radar` in synchronous mode, and record ~30 s.
   - *Synthetic source* (build this FIRST, since it doesn't depend on CARLA): generate frames from a track YAML.
     Cones become lidar point clusters with noise and dropout, radar returns get Doppler from ego speed, and the camera
     is a pinhole render of coloured cones. That's enough to demo and benchmark.
3. **Fast pipeline (numpy, target < 10 ms per frame on 1 core):**
   - Transform every point cloud into the vehicle frame with **one 4×4 matmul** per sensor.
   - Ground removal (z threshold, plus a 3-point RANSAC plane only if the scene is sloped).
   - Voxel downsample with `np.unique` on integer voxel keys, which is O(N) with no Python loops.
   - Clustering: grid-hash connected components on a 0.25 m BEV grid.
   - **Camera association:** project cluster centroids with `K[R|t]` and sample the image colour in a small
     window. That classifies blue/yellow/orange cones without any neural net.
   - **Radar association:** nearest-cluster gating (Mahalanobis or simple radius) attaches radial velocity.
   - Output: a fused object list `(x, y, class, velocity, confidence)`, then `ConeDetections`.
4. **Benchmark:** a per-stage ms table, and naive per-point loops vs our vectorised version (the speed-up is a slide).
5. **Visualisation:** one composite frame. Left: the camera image with lidar points coloured by depth and fused
   labels. Right: a BEV with lidar (grey), radar arrows (velocity), fused objects, and our planned path. Export a GIF/MP4
   (matplotlib animation + pillow).

**Done when:** 30 s of synthetic frames run end-to-end under 10 ms per frame, the composite visual is exported, and the
fused cones are fed back into the planner offline.

---

## Viraj: integration, tracking control, test harness, demo

### V1. Skeleton + integration: `controller.py`, `params.py` (Phase 0, then ongoing)
Own `main`'s health: review PRs, run `selftest` at every checkpoint, keep the fallback path working.

### V2. Tracking controller: `tracker.py`
**How:**
1. **Delay + slew compensation (physics prediction, part 2):** before steering, predict where the car will be when
   the command actually applies (`actuation_delay_steps + 1` steps, kinematic bicycle with the *applied* steer), and track
   from that predicted pose. Respect `steer_rate_limit` (2.0/s) when choosing the steer, so we never saturate it
   (that saturation costs realism points).
2. Lateral: pure pursuit with `Ld = clip(Lmin + kv·v, 4, 15)` on the **plan**, plus curvature feedforward
   `δ = atan(L·κ_plan(Ld))`. A/B it against Stanley on `practice_technical`.
3. Longitudinal: feedforward accel from the speed profile + PI. **Throttle and brake are never both > 0.**
4. **Friction ellipse limiter:** `a_lon ≤ a_lon_max·sqrt(1 − (a_lat/a_lat_max)²)`, using the *predicted* a_lat.
5. **Online slip adaptation:** `yaw_expected = v/L·tan δ`. If |yaw_rate − yaw_expected| stays above the threshold for
   0.3 s, lower `a_lat_max` by 5 %, and recover 1 % per second. Log `slip`, `a_lat_max_est`.

### V3. Test harness: `tools/`
- `tools/eval.py`: runs our controller on `practice_technical`, `practice_obstacles`, `hackathon_test` and N random
  seeds, and prints one table (lap, cones, gates, p99, technical points) plus the diff vs. the baseline and vs. the last run.
- `tools/plot_run.py`: from `telemetry.csv`, overlays the racing line vs. the centerline (speed-coloured), a **g-g diagram
  with the friction ellipse** (shows we stay inside grip), the step-time histogram, and planned vs. actual speed.
- Run `batch` (robustness sweep) at 3:30 and fix whatever breaks.

### V4. Demo + slides (lead from 4:15; everyone contributes)
See the demo plan below.

---

## Demo plan (4 minutes)

1. **0:00–0:30 Problem:** no map, 40 m of sight, 50 ms budget, unseen tracks. Show the baseline clipping cones.
2. **0:30–1:45 Our car:** a side-by-side recording (CARLA if we have it, otherwise `--view`) on `practice_technical`
   with the racing line drawn. Then the table: baseline vs. ours (lap, cones, p99, points).
3. **1:45–2:45 The physics:** the predicted rollout vs. the actual path, the g-g diagram inside the friction ellipse,
   the visibility guard braking for an unseen hairpin, and slip adaptation on a low-grip run (`--perturb`).
4. **2:45–3:30 Beyond the brief:** the sensor-fusion composite GIF, the benchmark (ms per frame, speed-up), and fused
   cones driving our planner.
5. **3:30–4:00 Robustness + close:** the `batch` sweep results across noise, latency and low grip. Close with
   "it generalises because it's physics, not tuning".

Record the videos **before 4:45**. Never demo live without a recorded backup.

## Submission checklist (4:15–5:00)

- [ ] `selftest -c ..\hackuab\controller.py` passes, with **no sandbox violations** (both backends if CARLA works)
- [ ] `hackuab/` contains only what the controller needs (no `tools/`, no `fusion/`), and imports are numpy + stdlib only
- [ ] No debug prints, no file writes, `on_start` < 60 s
- [ ] README: what it does, how to run it, the innovation list with plots, and results tables
- [ ] Innovation write-up (`docs/INNOVATION.md`): each feature, why it matters, and evidence (plot/number)
- [ ] `main` tagged `submission` at 5:00

## Cut list (if we're behind at 2:30, cut in this order)

1. Sampling MPC (keep the predictive verifier)
2. The fusion CARLA source (keep the synthetic source + visuals)
3. Stanley A/B (keep pure pursuit + feedforward)
4. Slip adaptation
5. CARLA validation runs (the bicycle model is the fallback)

**Never cut:** 0 cones/gates, `selftest` passing, the recorded demo, the README/innovation write-up.

## Risks

| Risk | Mitigation |
|---|---|
| CARLA won't run on integrated graphics | Plans A/B/C in R1. Decide by 2:30 |
| A module breaks `main` near freeze | `controller.py` fallback + checkpoint merges only + feature freeze at 4:15 |
| Overfitting to the test tracks | Always evaluate on random seeds; never branch on position/station |
| Step-time spikes from the rollout/solver | Hard per-stage budget in `controller.py`; skip the verifier if over budget |
| Fusion eats the main controller's time | Saisi starts fusion only after S1 is merged |
