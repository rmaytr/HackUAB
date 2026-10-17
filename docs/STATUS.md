# STATUS: where the project stands (handoff file for any AI agent)

**Update this file at the end of every task** (results table + what changed + next step) and commit it
with your work, so the next agent, whether Claude, ChatGPT/Codex or a teammate's, picks up exactly where you left off.

_Last updated: ~1:15 PM, after merge commit `62602f2` on `main`. Code freeze 5:00 PM, feature freeze 4:15 PM._

## What's on `main`

| Module | Owner | State |
|---|---|---|
| `hackuab/horizon.py` | Saisi | Obstacle-aware safe corridor (+ tests) |
| `hackuab/planner.py` | Rodion | Min-curvature racing line + forward/backward speed profile + visibility guard + AWD traction cap. **Every plan is now checked by `predict.verify_plan()`** (speed scaled down where the dynamic model predicts too much grip use) |
| `hackuab/predict.py` | Rodion | Dynamic AWD bicycle model (`rollout_dynamic`) + `verify_plan()`. ⚠️ `verify_plan` still mirrors the **old** tracker (reads `ellipse_lat_frac`, heuristic friction ellipse); see next task 1 |
| `hackuab/tracker.py` | Viraj | Latency compensation, curvature feedforward + pure-pursuit feedback, **per-axle AWD friction model** (`grip_frac`, replaces the old ellipse), slip adaptation |
| `hackuab/controller.py`, `params.py` | Viraj | Tiered fallback, budget guard, per-stage timing; merges planner `pred_*` diagnostics into `Command.debug` |
| `tools/plot_run.py`, `plot_laps.py`, `runs_table.py`, `figures/` | Viraj | Demo figures: course map, speed trace, g-g diagram, step time, lap times (bicycle + CARLA) |
| `docs/SPONSOR_RESEARCH.md` | Rodion | Sponsor/judging research; read it before writing the pitch |

Vehicle: **all-wheel drive** (EIC Chevy Blazer EV): 40 % front drive, 60 % front brakes. See CLAUDE.md "Drivetrain".

## Latest results (bicycle backend, `selftest`, default params)

| Course | Time | Cones | Technical /75 |
|---|---|---|---|
| hackathon_test | 54.1 s | 0 | 67.4 |
| practice_basic | 31.9 s | 0 | 70.5 |
| practice_loop | 88.9 s | 0 | 69.5 |
| practice_obstacles | 57.8 s | 0 | 68.1 |
| practice_technical | 40.5 s (ref 31.6) | 0 | 65.0 |
| **Mean** | | **0** | **68.1** (Phase 0 baseline: 62.3) |

p99 step time ≈ 4.7 ms / 50 ms (the verifier adds about 1.5 ms). 60 tests pass. 0 controller errors.

**Same code with `lat_accel_frac: 0.85`** (not the default yet): mean **69.4/75**, 0 cones, faster everywhere
(hackathon_test 49.5 s, basic 29.5 s, loop 85.3 s, obstacles 53.6 s, technical 37.9 s). But `practice_basic`'s
score *dropped* 70.5 → 69.0, so find out which category lost points before adopting it.

CARLA: Viraj has run earlier versions (see `figures/*_carla_*`). This exact `main` has **not** been compared on CARLA yet.

## Next tasks

**Rodion (planner.py, predict.py):**
1. **Sync `verify_plan` with the current tracker.** Use `grip_frac` and the per-axle AWD friction model from
   `tracker.py` (preferably call tracker's own helpers instead of copying them, so they can't drift again).
   Add a test that fails if `verify_plan`'s control law and `track()` disagree on the pedal command for the same state.
2. **Pick `lat_accel_frac`** in 0.75–0.9: faster on every course with 0 penalties, also on 3 random courses
   (`gen-track --seed 1/2/3 --obstacles 3`). Explain the `practice_basic` drop at 0.85 (compare the score categories in
   `summary.json`). Record the chosen value here. Don't change the default until Viraj confirms it on CARLA.
3. Then raise `decel_frac` (0.4) the same way. Target: `practice_technical` ≤ 36 s with 0 penalties.
4. Write `docs/INNOVATION.md`: racing line, physics-verified speed profile, visibility guard, AWD model.
   For each: what it is, why it matters, and the evidence (a number or a plot from `figures/`).

**Viraj:** run `compare` (bicycle vs CARLA) on this `main`, then test `lat_accel_frac` 0.85 on CARLA.
**Saisi:** fusion track (roadmap S2).

## How to verify any change (from `simulator/`, Windows)

```powershell
.venv\Scripts\python.exe -m pytest ..\tests -q
.venv\Scripts\python.exe -m autocross run -c ..\hackuab\controller.py --controller-config ..\tools\strict.yaml -t tracks\practice_technical.yaml --backend bicycle --no-render
.venv\Scripts\python.exe -m autocross run -c ..\hackuab\controller.py --controller-config ..\tools\strict.yaml -t tracks\practice_obstacles.yaml --backend bicycle --no-render
.venv\Scripts\python.exe -m autocross selftest -c ..\hackuab\controller.py --no-render
```

To try a parameter without changing the default, put it in a YAML file outside `hackuab/` (e.g. `tools/try.yaml` with
`lat_accel_frac: 0.85`) and pass it with `--controller-config`.

Merge to `main` only if tests pass, selftest has 0 errors, and **no course gets new penalties** compared with the table above.
