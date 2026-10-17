# CLAUDE.md: HackBama_01, EcoCAR Autocross controller

Read this first. Three of us run AI coding agents in parallel on separate branches,
so these rules keep merges painless and `main` always submittable.

**Timeline:** hacking started 9:30 AM · **code freeze 5:00 PM** · demo is 3–5 min.

## Project & track

- **Original spec:** [`docs/ecocar_autocross_context_architecture_spec.md`](docs/ecocar_autocross_context_architecture_spec.md)
  (the organizers' brief; this file supersedes its TODO section with the real API below).
- **Challenge:** University of Alabama EcoCAR Innovation Challenge, autonomous **autocross**.
- **What we build:** one Python `Controller` (path planner + motion controller) for the
  `mil-sim` harness: https://github.com/dszurek/ecocar-hackathon-sim
- **The catch:** there is **no map**. Every 50 ms we get `obs.road`, the course visible up to
  **40 m ahead**, and return steering/throttle/brake plus the path we intend to drive.
- **Scored on unreleased courses**, so never tune to the shape of a test track and never hardcode
  stations or corner sequences. Work only from `obs.road`.
- **One-sentence pitch:** _TBD_
- **Demo path (3–5 min):** _TBD_ (suggested: baseline vs. ours on `practice_technical` with `--view`,
  then `report.png` showing the racing line, speed profile and step-time plot)

## Scoring: what actually earns points

Final = weighted technical points (out of 75) over 4 runs + judged innovation (out of 25).
The 4 runs are 2 unreleased courses × 2 backends. **Technical course 60% / main course 40%**,
**CARLA 50% / bicycle 50%**.

| Category | Weight | How to win it |
|---|---|---|
| Completion time | 30 | Racing line + friction-limited speed profile |
| Path deviation | 15 | Publish the **real** plan in `Command.path`, reaching well ahead, and track it tightly. It is compared with the path published 0.5 s earlier. Zig-zag or fold-back paths get thrown out. |
| Compute | 15 | Low p99 step time, no overruns, low memory |
| Physical realism | 15 | No grip over-demand, no rear sliding, no steering-rate saturation, **no throttle+brake overlap** |
| Innovation | 25 | Judged: algorithmic sophistication + path visualization |

Penalties: cone hit −2 · off course −5 · **missed gate −10** · controller error −1 (max 10).
DNF: off course >5 s, >15 m from course, wrong way, stuck 8 s, rollover, 240 s timeout,
or 20 controller errors in a row.

**Priority order:** finish cleanly on `practice_technical` > zero cones/gates > lap time > polish.
A slower clean run beats a fast run that clips cones.

## Stack & environment

- **Python 3.10–3.12 only.** CARLA wheels don't support 3.13+.
- The controller runs in the harness venv, so **the only packages are numpy, PyYAML, psutil,
  matplotlib and pygame.** No scipy, numba or cvxpy. Write numpy-only solvers.
- Develop on the **bicycle backend** (no CARLA needed). CARLA is optional and counts for half the score.

### Setup (one time per teammate)

The simulator is a **git submodule** at `simulator/`, pinned to a known commit. Do NOT edit files
inside it. Our code lives in `hackuab/` at the repo root.

```powershell
git clone --recurse-submodules https://github.com/rmaytr/HackUAB.git
cd HackUAB
git config core.longpaths true                    # Windows: avoids "Filename too long"
# already cloned without the submodule?  git submodule update --init --recursive
cd simulator
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1     # needs Python 3.10–3.12
.venv\Scripts\Activate.ps1          # or call .venv\Scripts\python.exe directly
```

### Everyday commands (run from inside `simulator/`)

```powershell
# watch it drive
python -m autocross run -c ..\hackuab\controller.py -t tracks\practice_technical.yaml --backend bicycle --view
# compare with the baseline
python -m autocross run -c controllers\baseline_pure_pursuit.py -t tracks\practice_technical.yaml --backend bicycle
# random unseen course (catches overfitting); try several seeds
python -m autocross gen-track --seed 42 --obstacles 3 --out tracks\my_random.yaml --preview
# robustness sweep: noise, low grip, latency, start offsets
python -m autocross batch -c ..\hackuab\controller.py
# REQUIRED before merging to main and before submitting
python -m autocross selftest -c ..\hackuab\controller.py
```

Each run writes `runs/<stamp>_<team>_<course>/` with `report.png` (open it first),
`telemetry.csv` (includes our `dbg_*` columns), `events.json` and `summary.json`.

## Simulator API (from `autocross/api.py`)

Conventions: world frame is ENU (x east, y north, yaw CCW). The **vehicle frame is x forward,
y left**. Positive steer, curvature and yaw rate all mean **left**.

```python
from autocross.api import Command, Controller, Observation, TrackInfo, VehicleParams

class HackUABController(Controller):
    def on_start(self, track: TrackInfo, vehicle: VehicleParams) -> None:   # ≤ 60 s, untimed work OK
        super().on_start(track, vehicle)    # track is EMPTY in event mode

    def step(self, obs: Observation) -> Command:                            # ≤ 50 ms, every 50 ms
        ...
```

- **`VehicleParams`:** `mass` 1845 kg, `wheelbase` 3.0 m, `track_width`, `length`, `width`,
  `cg_to_front/rear`, `max_steer` (rad, 35°), `max_steer_rate` (normalised units/s),
  `mu` ≈ 0.95, `max_accel` ≈ 5.2 m/s², `max_decel` ≈ 9 m/s², `max_speed`, `control_dt` (0.05 s),
  `actuation_delay_steps`.
- **`Observation`:** `t, step, x, y, yaw, vx, vy, yaw_rate, ax, ay, speed`, the applied
  `steer, throttle, brake`, `progress, lap, timing_started`, `cones`, `road`.
- **`obs.road` (`RoadModel`, vehicle frame, referenced to the centerline):**
  - `valid`, `range_m` (40), `lateral_offset` (+ = car left of centerline), `heading_error`
    (+ = pointing left), `curvature`, `width_left/right`, `dist_to_left/right_boundary`
    (negative = outside)
  - `preview_s` (K,), `preview_curvature` (K,), `preview_centerline` (K,2),
    `preview_width_left/right` (K,), `preview_left/right_boundary` (K,2)
  - `corners: list[Corner]` with `direction` (±1), `entry/apex/exit_distance` (apex and exit are NaN
    until visible), `min_radius`, `turn_angle`, `fully_visible`
  - `obstacles: list[ObstacleInfo]` with `distance`, `lateral_min/max` (vs. centerline, + left),
    `center` (2,), `points` (K,2)
  - `finish_distance` (NaN until in range)
- **`obs.cones` (`ConeDetections`, optional):** `positions` (K,2), `colors`
  (0 left/blue, 1 right/yellow, 2 orange), within 30 m and a 140° field of view.
- **`Command`:** `steer` [-1, 1] (maps linearly to ±`max_steer`), `throttle` [0, 1], `brake` [0, 1],
  `path` (N,2) in `path_frame="vehicle"` (relative to the pose in the obs being answered),
  `target_speed`, `debug: dict[str, float]` (logged as `dbg_*`).
- If `road is None or not road.valid`, return `Command(brake=1.0)`.

### Harness rules (violations count as controller errors)

- 1 CPU core, no GPU, 1 GB memory. The 50 ms deadline is measured as a round trip and **includes
  background-thread CPU time**. On an overrun the previous command is held.
- Steering is slew-rate limited, and actuation delay may be added in robustness runs.
- An exception, NaN or wrong return type is an error. **Sanitise every output** (`np.nan_to_num`, clip).
- The sandbox forbids writing files, subprocesses, network, `ctypes`, importing `carla`, gc
  inspection, and reading `tracks/` or `runs/`. Log with `Command.debug`, never with files.
- **Helper modules:** the harness puts the controller's folder on `sys.path`, so import siblings
  as plain modules (`from planner import ...`). Use names that can't clash with stdlib or
  installed packages.

## Architecture (target < 35 ms average `step()`)

```text
obs ─▶ horizon.py (~5 ms) ─▶ planner.py (~20 ms) ─▶ tracker.py (~10 ms) ─▶ Command
       corridor + buffer     racing line + v(s)      steer + pedals
```

1. **`horizon.py`: corridor.** Data is already in the vehicle frame. Build the centerline, left/right
   widths and obstacles into a safe corridor: shrink each side by `vehicle.width/2 + 0.3 m`, and cut
   the corridor at each obstacle's `lateral_min/max` plus the buffer, keeping the wider free side.
2. **`planner.py`: path + speed.**
   - Path: start with a smoothed centerline. Target a **minimum-curvature line** inside the corridor
     (shift lateral offsets toward the inside of turns, solved in closed form or with a few
     iterations of a banded linear solve in numpy).
   - Curvature: κ = |x′y″ − y′x″| / (x′² + y′²)^{3/2}, from `np.gradient` on arc length.
   - Speed: v_max(s) = √(a_lat,max / max(|κ|, ε)), then a **backward pass** (braking with `max_decel`)
     and a **forward pass** (accel with `max_accel`).
   - **Visibility guard:** constrain v at s = 40 m as if a hairpin (min radius ~ course minimum)
     starts right after it.
3. **`tracker.py`: control.**
   - Lateral: pure pursuit with speed-scaled lookahead `Ld = clip(Lmin + kv·v, Lmin, Lmax)`
     (or Stanley), plus curvature feedforward `δ_ff = atan(L·κ)`. Respect `max_steer_rate`.
   - Longitudinal: feedforward + PI on target speed. **Throttle and brake are mutually exclusive.**
   - **Friction ellipse:** cap |a_lon| so (a_lat/a_lat,max)² + (a_lon/a_lon,max)² ≤ 1.
4. **`controller.py`** wires 1→2→3, times each stage with `time.perf_counter()`, publishes
   `Command.path` and `debug` timings, and on any exception or budget overrun **falls back to the
   centerline pure-pursuit path**.

### Innovation features (document each one for the judges)

1. Real-time minimum-curvature racing line in the 40 m corridor.
2. Friction-ellipse limiter coupling throttle/brake to lateral demand.
3. Horizon-end visibility braking guard.
4. Online understeer/slip adaptation: compare expected yaw rate `v/L·tan δ` with `obs.yaw_rate`,
   and lower `a_lat,max` when slip exceeds a threshold. Recover it slowly.
5. Obstacle-aware corridor reshaping (obstacles are not on any map; they only appear in perception).
6. **Physics-based prediction** (`predict.py`): a vectorised dynamic bicycle rollout that verifies every plan
   (friction use, corridor containment) before we commit to it, and compensates actuation delay and steering lag.
7. **Sensor-fusion tool** (`fusion/`, outside the scored controller): fast lidar + radar + camera fusion with
   a composite visualisation, whose output feeds our planner as cone detections.

## Folder ownership

Edit only inside the files you own. Shared files need a heads-up in the team chat first.

```
hackuab/                ← submission folder (judges copy it into teams/)
  controller.py         integration, fallback, per-stage timing (keep it thin)
  params.py             tunables + bicycle-model constants copied from simulator/config/sim.yaml
  horizon.py            safe corridor + obstacle avoidance
  planner.py            min-curvature racing line + speed profile + visibility guard
  predict.py            vectorised dynamic bicycle model: rollouts for prediction / verification
  tracker.py            delay compensation, steering, throttle/brake, friction ellipse, slip adaptation
tools/                  dev-only scripts (eval tables, plots, sweeps); NOT submitted
fusion/                 standalone lidar + radar + camera fusion tool; NOT submitted, may use CARLA
docs/                   ROADMAP.md, pitch, innovation write-up, demo media
```

| Teammate | Owns | Branch prefix |
| -------- | ---- | ------------- |
| Rodion   | `planner.py`, `predict.py`, CARLA setup + validation runs | `rodion/*` |
| Saisi    | `horizon.py`, `fusion/` | `saisi/*` |
| Viraj    | `controller.py`, `params.py`, `tracker.py`, `tools/`, demo slides | `viraj/*` |

`docs/` is shared: announce in the chat before editing a file there. **The plan, the timeline and each
task's "how" are in [docs/ROADMAP.md](docs/ROADMAP.md). Read it before starting work.**

## Data contracts between modules

All arrays are numpy and in the **vehicle frame** (x forward, y left). Units are metres, seconds and radians.

```python
# horizon.py
@dataclass
class Corridor:
    s: np.ndarray            # (K,) station along centerline, 0..range
    center: np.ndarray       # (K,2) centerline points
    normal: np.ndarray       # (K,2) unit left-normal at each point
    left: np.ndarray         # (K,) usable offset to the left  (+, buffer already removed)
    right: np.ndarray        # (K,) usable offset to the right (+, buffer already removed)
    valid: bool
def build_corridor(road: RoadModel, vehicle: VehicleParams, p: Params) -> Corridor: ...

# planner.py
@dataclass
class Plan:
    path: np.ndarray         # (N,2) driven line = center + offset*normal; this goes in Command.path
    s: np.ndarray            # (N,) arc length along the path
    kappa: np.ndarray        # (N,) signed curvature, + left
    v: np.ndarray            # (N,) target speed after the backward/forward passes
def plan(c: Corridor, obs: Observation, vehicle: VehicleParams, p: Params) -> Plan: ...

# tracker.py
def track(plan: Plan, obs: Observation, vehicle: VehicleParams, p: Params, state: dict) -> tuple[float, float, float]:
    """Return (steer, throttle, brake), already clipped, NaN-free, and with throttle and brake never both > 0."""
```

Change a contract only after telling the team, and update this section in the same commit.

## Visualization brief (for the demo and the innovation score)

- `Command.path` is drawn live (cyan in CARLA, and also in `--view`). Make it smooth and long (≥ 20 m).
- Log the story through `debug`: `t_horizon_ms`, `t_plan_ms`, `t_track_ms`, `v_target`, `kappa_max`,
  `a_lat_max_est`, `slip`, `fallback` (0/1). They show up in `telemetry.csv` for plots.
- Demo visuals: baseline vs. ours lap time on the same course, the racing line overlaid on the
  centerline, the speed profile, and a step-time histogram showing p99 ≪ 50 ms.
- Charts look deliberate: one accent colour for "ours", grey for the baseline, labelled axes, no
  default matplotlib rainbow.

## Rules

1. **Small commits** with clear messages. Push often.
2. **Stay in your own files** (see ownership above).
3. **Never commit `.env*`, keys, tokens, `runs/`, or `.venv/`.**
4. **Keep `main` submittable.** Work on your branch and open a PR. Merge only after
   `python -m autocross selftest -c ..\hackuab\controller.py` passes with no sandbox
   violations and the run is no worse than the current `main`.
5. **No overfitting.** Never branch on course name, station or position. Validate on ≥ 3 random
   `gen-track` seeds plus `practice_technical`.
6. **Vectorise with numpy.** No Python loops over spatial arrays. Keep average `step()` < 35 ms and p99 < 45 ms.
7. Do any warm-up work in `on_start` (≤ 60 s), not on step 1.
8. Pull/rebase from `main` before opening a PR, and especially before 5:00 PM.
9. Feature freeze target ~4:15 PM. After that: tuning, bug fixes, demo and README only.

## Agent skills in this repo

Skills are committed as real folders (not symlinks) in two places, so they work right after `git pull`:

- `.claude/skills/` is loaded by Claude Code. Start a new session after pulling.
- `.agents/skills/` is the universal location read by Codex, Cursor, Gemini CLI, Copilot and others.

`skills-lock.json` records each skill's source and hash.

| Skill | Source | Use it for |
|---|---|---|
| `python-performance-optimization` | wshobson/agents | Profiling `step()` (cProfile, `perf_counter`) to stay under 35 ms average |
| `python-testing-patterns` | wshobson/agents | pytest unit tests for `horizon.py` / `planner.py` / `tracker.py` against synthetic `RoadModel`s |
| `data-visualization` | anthropics/knowledge-work-plugins | Demo charts: lap time vs. baseline, speed profile, step-time histogram |
| `doc-coauthoring` | anthropics/skills | README, the innovation write-up for judges, the demo script |
| `frontend-design` | anthropics/skills | Making slides and visuals look intentional |
| `find-skills` | vercel-labs/skills | Searching for more skills (`npx skills find <query>`) |

To add another skill, run `npx skills add <owner/repo@skill> -y` from the repo root. Then replace the
symlink it creates in `.claude/skills/` with a real copy of the `.agents/skills/<name>` folder before
committing, because the symlink points at an absolute path on your machine.
