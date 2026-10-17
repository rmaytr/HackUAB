# STATUS: where the project stands (handoff file for any AI agent)

**Update this file at the end of every task** (results table + what changed + next step) and commit it
with your work, so the next agent, whether Claude, ChatGPT/Codex or a teammate's, picks up exactly where you left off.

_Last updated: 2026-10-17, ~1:45 PM, team consolidation on `rodion/team-sync`, promoted to `main`
after validation. Code freeze 5:00 PM, feature freeze 4:15 PM._

## United code state on `main`

| Module | Owner | State |
|---|---|---|
| `hackuab/horizon.py` | Saisi | Obstacle-aware safe corridor (+ tests); reads `corridor_margin` consistently with Params/config |
| `hackuab/planner.py` | Rodion | Min-curvature racing line + forward/backward speed profile + visibility guard + AWD traction cap. **Every plan is now checked by `predict.verify_plan()`** (speed scaled down where the dynamic model predicts too much grip use) |
| `hackuab/predict.py` | Rodion | Dynamic AWD bicycle model + verifier using tracker's AWD/slip/default/tangent helpers. Matches current-vs-preview braking limits and overspeed integral suppression. Six regression cases check steering/pedals at every rollout step |
| `hackuab/tracker.py` | Viraj | Latency compensation, curvature feedforward + pure-pursuit feedback, per-axle AWD friction model, slip adaptation; current steering load limits braking, preview load limits throttle; suppresses positive integral when overspeeding |
| `hackuab/controller.py`, `params.py` | Viraj | Tiered fallback, budget guard, per-stage timing; obstacle taper 20 m, overspeed band 1 m/s. `lat_accel_frac` remains 0.6 |
| `tools/plot_run.py`, `plot_laps.py`, `runs_table.py`, `figures/` | Viraj | Demo figures: course map, speed trace, g-g diagram, step time, lap times (bicycle + CARLA) |
| `docs/SPONSOR_RESEARCH.md` | Rodion | Sponsor/judging research; read it before writing the pitch |

Vehicle: **all-wheel drive** (EIC Chevy Blazer EV): 40 % front drive, 60 % front brakes. See CLAUDE.md "Drivetrain".

## Latest unified validation (default params, bicycle)

Combined `rodion/regroup` (which already contains verifier-sync, Viraj's tracker/tools/figures and
Saisi's corridor fix) with the remaining `chore/team-setup` architecture history. All fetched remote
branch tips are ancestors of the integration version. Sponsor research, both earlier planner
branches and Saisi's earlier corridor/fusion branch history are included. No remote branch was
deleted or force-pushed, and the simulator stays pinned at `06db3847f429acc107dd95ad6b176f612245169a`.
Uncommitted work in another agent's `rodion/regroup` worktree is preserved separately; it is not
claimed to be part of this published version. Other machines must pull `main` to use this code.

Integration testing caught a real semantic conflict: the existing pedal regression expected the
new tracker's 0.575 brake command, while the verifier still produced 0.300. `predict.py` now matches
Viraj's current-steering braking limit and positive-integral suppression. It only evaluates the
AWD bound that can clip the requested command, and skips both searches when the command is already
inside the mandatory safety floors. This preserves exact tracker commands while reducing compute.
The added accelerate-then-brake case verifies that a positive integral is actually suppressed.
Tracker/horizon/params changes were merged from their owners, not rewritten.

| Course | Previous recorded main time / points | Unified default time / points | Cones / off / gates / errors |
|---|---:|---:|---|
| hackathon_test | 54.114 / 67.41 | 51.949 / 68.40 | 0 / 0 / 0 / 0 |
| practice_basic | 31.893 / 70.55 | 31.616 / 70.06 | 0 / 0 / 0 / 0 |
| practice_loop | 88.869 / 69.46 | 88.975 / 68.90 | 0 / 0 / 0 / 0 |
| practice_obstacles | 57.817 / 68.07 | 57.281 / 67.34 | 0 / 0 / 0 / 0 |
| practice_technical | 40.545 / 65.04 | 38.701 / 65.88 | 0 / 0 / 0 / 0 |
| **Mean technical /75** | **68.106** | **68.116** | **No new penalties** |

All 68 tests pass, including the unchanged performance checks (gentle-circle verifier benchmark
0.704 ms). Required strict technical/obstacle runs finish in 38.701 / 57.281 s, clean. Full selftest
has no errors, sandbox violations or overruns; p99 7.2-10.3 ms / 50 ms. Some course scores remain
below the old baseline due to compute/deviation differences; the aggregate is effectively unchanged.

Three random courses (`gen-track --seed 1/2/3 --obstacles 3`) also pass with strict error handling:

| Seed | Time (s) | Technical /75 | Cones / off / gates / errors |
|---|---:|---:|---|
| 1 | 51.587 | 68.94 | 0 / 0 / 0 / 0 |
| 2 | 62.879 | 68.79 | 0 / 0 / 0 / 0 |
| 3 | 54.359 | 68.05 | 0 / 0 / 0 / 0 |

Reports: `simulator/runs/20261017-134227_selftest/`; strict runs
`20261017-134128_controller_practice_technical` and `20261017-134153_controller_practice_obstacles`;
random runs under `runs/team-sync/` (all ignored). Technical report visually inspected.
This exact integrated version has not been run on CARLA.

**The earlier 0.8875 recommendation below is historical:** its sweep used the older tracker and
8 m obstacle taper. Revalidate it against this integrated 20 m taper and braking logic before
adopting it or making CARLA comparisons. The default remains 0.6.

## Historical `main` baseline before consolidation (default params)

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

CARLA: Viraj has run earlier versions (see `figures/*_carla_*`). This exact `main` and the verifier-sync branch have **not** been compared on CARLA yet.

## Historical verifier-sync and parameter results (`b343f5a`)

**Historical candidate `lat_accel_frac: 0.8875`; revalidation on united code is required.** It was the only tested value in
0.75-0.9 that passed all five public courses and seeds 1/2/3 with three obstacle groups each, with
zero cones, off-course events, missed gates, controller errors or score deductions. It is faster
on every course than the 0.6 baseline. **`params.py` remains at 0.6.** No tracker, horizon, controller,
simulator or planner behavior was changed; per Rodion's direction this remained parameter-only
apart from the verifier fix and its tests.

The verifier now calls the tracker's per-axle AWD envelope (including `grip_frac`, load transfer,
drive/brake split and traction limits), slip adaptation, defaults and tangent helper. It uses the
same plan stations, nearest-node selection, control-period PI/jerk updates and stopped-car brake
hold. The scalar control calculations retain the existing <2 ms verifier test budget; calling the
whole numpy tracker 30 times exceeded it. Five new parameterized cases compare steering and both
pedals to `track()` on every one of 30 identical predicted states, including nonuniform stations,
tracking error, reduced grip, delay, acceleration, braking, saturation and standstill. Running the
new regression against the old verifier fails: first-case throttle 0.884 versus tracker's 1.0.
The verifier still initializes PI/jerk history afresh and infers initial grip scale from `p.a_lat`;
it does not have access to the live tracker's complete state.

### Results versus the recorded STATUS baseline

All times below are seconds, points are technical /75. All three public-course columns have zero
cones/off-course/missed gates/controller errors. The middle column isolates the verifier change
from the parameter change. Baseline values use the original summary files rather than rounded
numbers from the table above.

| Course | Recorded main 0.6 time / points | Synced verifier 0.6 time / points | Synced verifier 0.8875 time / points | Time saved vs recorded main |
|---|---:|---:|---:|---:|
| hackathon_test | 54.114 / 67.41 | 54.114 / 66.83 | 49.534 / 68.79 | 4.580 |
| practice_basic | 31.893 / 70.55 | 31.910 / 69.26 | 29.441 / 69.31 | 2.452 |
| practice_loop | 88.869 / 69.46 | 88.917 / 68.81 | 84.870 / 69.45 | 3.999 |
| practice_obstacles | 57.817 / 68.07 | 57.930 / 66.78 | 52.996 / 68.87 | 4.821 |
| practice_technical | 40.545 / 65.04 | 40.687 / 64.30 | 37.725 / 66.02 | 2.820 |
| **Mean points** | **68.11** | **67.20** | **68.49** | |

Reusing the AWD helper costs compute points: default selftest p99 is 7.9-11.3 ms versus about
4.7 ms before this branch; recommended-value p99 is 11.6-12.4 ms across all eight courses, mean
step time 7.1-7.6 ms, versus the 50 ms deadline. Recommended runs have no overruns and use about
33 MB. The default-parameter mean score regression is primarily compute, not new penalties.
The recommendation improves mean public score, but **does not improve every course's score**:
basic loses 1.24 points and loop loses 0.01 points versus recorded main despite faster laps.

| Generated course (`--obstacles 3`) | Synced 0.6 time / points | 0.8875 time / points | Time saved | Cones / off / gates / errors at 0.8875 |
|---|---:|---:|---:|---|
| seed 1 | 52.279 / 67.58 | 48.271 / 68.53 | 4.008 | 0 / 0 / 0 / 0 |
| seed 2 | 64.190 / 67.16 | 58.429 / 69.51 | 5.761 | 0 / 0 / 0 / 0 |
| seed 3 | 54.960 / 67.48 | 50.526 / 68.37 | 4.434 | 0 / 0 / 0 / 0 |

Seeds 2 and 3 were repeated independently at 0.8875: identical times (58.429 / 50.526 s), again
zero penalties/errors/overruns. This is bicycle evidence only, not a robustness or CARLA guarantee.
Nearby values fail, and rear-slip dynamics violations remain: recommended runs reach 11.74 deg
on basic, 15.96 deg on seed 1 and 18.94 deg on seed 3. Zero penalty deductions do not mean full
physical-realism points. Do not raise the default without Viraj's CARLA confirmation.

### Why old 0.85 lost points on practice_basic

Compared `simulator/runs/20261017-125806_selftest/practice_basic_bicycle/summary.json` (0.6) with
`simulator/runs/20261017-125550_selftest/practice_basic_bicycle/summary.json` (old 0.85).

| Category points | 0.6 | Old 0.85 | Change |
|---|---:|---:|---:|
| Completion time | 26.07 | 28.39 | +2.32 |
| Path deviation | 14.48 | 13.08 | -1.40 |
| Compute | 15.00 | 14.99 | -0.01 |
| Dynamics | 15.00 | 12.52 | -2.48 |
| **Technical total** | **70.55** | **68.97** | **-1.58** |

The faster lap (31.893 -> 29.510 s) gained time points, but rear slip exceeded the scorer's
6-degree threshold for 8.25% of moving samples (previously 0%; peak 4.58 -> 12.19 degrees).
Grip-demand, steering-rate and pedal-overlap violation fractions stayed zero, so the dynamics
loss is rear sliding. Mean/max committed-path error worsened 0.152/0.404 -> 0.182/0.962 m,
accounting for the deviation loss. Rounded category points need not sum exactly to the total.

### Parameter sweep and exclusions

Public selftests covered the 0.025 grid from 0.75 to 0.9, then 0.8875. All public-clean grid values
were tested on all three random courses. After seed 2 rejected them, the intervening 0.0125
midpoints and 0.875 were checked on seed 2. A dash means not run after an existing exclusion.
All courses finished; there were no controller errors. One excluded 0.8 seed-3 run had a deadline
overrun; no selected/default verification run did.

| lat_accel_frac | Public mean points | Public cones | Seed 1 / 2 / 3 cones | Decision |
|---|---:|---:|---|---|
| 0.7500 | 68.03 | 0 | 0 / 1 / 0 | Exclude |
| 0.7625 | - | - | - / 1 / - | Exclude |
| 0.7750 | 67.81 | 0 | 0 / 1 / 0 | Exclude |
| 0.7875 | - | - | - / 1 / - | Exclude |
| 0.8000 | 68.18 | 0 | 0 / 1 / 0 | Exclude |
| 0.8125 | - | - | - / 1 / - | Exclude |
| 0.8250 | 68.15 | 0 | 0 / 1 / 0 | Exclude |
| 0.8375 | - | - | - / 1 / - | Exclude |
| 0.8500 | 68.00 | 0 | 0 / 1 / 0 | Exclude |
| 0.8625 | - | - | - / 1 / - | Exclude |
| 0.8750 | 68.11 | 1 | - / 1 / - | Exclude |
| **0.8875** | **68.49** | **0** | **0 / 0 / 0** | **Best tested qualifying value** |
| 0.9000 | 68.34 | 0 | 0 / 1 / 2 | Exclude |

The recurring seed-2 collision is unmapped cone 474 near progress 510.4-510.8 m. At the collision,
the rejected runs lack `pred_*` diagnostics, consistent with the planner's existing centerline
fallback when the corridor is invalid; the default run has a verified plan there. This warrants
a separate corridor/fallback investigation, not course-specific tuning. No safeguard was added
because Rodion requested parameter-only scope.

### Verification and reproducibility

- **65 tests pass** (60 previous + five pedal-regression cases); existing predict timing tests pass.
- Required strict default technical run: 40.687 s, zero penalties/errors, p99 10.74 ms.
- Required strict default obstacles run: 57.930 s, zero penalties/errors, p99 11.23 ms.
- Required default selftest: all five finish, zero penalties/controller errors/sandbox violations.
- Recommendation: strict selftest plus three generated courses, zero deductions/errors/overruns;
  sensitive seeds 2/3 repeated cleanly.

Artifacts (ignored, local only): required checks under `simulator/runs/20261017-131251_controller_practice_technical`,
`20261017-131426_controller_practice_obstacles`, `20261017-131508_selftest`;
sweeps/configs/generated tracks under `runs/verifier-sync/`; selected public suite
`runs/verifier-sync/lat08875/20261017-133131_selftest/`, random runs under its `random/` directory,
repeat checks under `runs/verifier-sync/confirm/`. Reports were visually inspected.

To reproduce without editing defaults, create a YAML outside `hackuab/` containing
`strict: true` and `lat_accel_frac: 0.8875`, then pass it with `--controller-config` to `selftest`
and `run`. Generate courses outside the submodule with `gen-track --seed 1 --obstacles 3 --out PATH`
(repeat seeds 2 and 3). Keep every other parameter at its default, including `decel_frac: 0.4`.

## Next tasks

**Rodion (planner.py, predict.py):**
1. Revalidate the historical 0.8875 lateral-fraction candidate on the unified tracker and 20 m taper:
   public courses plus random seeds 1/2/3, then Viraj's CARLA confirmation before changing defaults.
2. After CARLA confirmation, raise `decel_frac` (0.4) the same way. Target: `practice_technical` ≤ 36 s with 0 penalties.
3. Write `docs/INNOVATION.md`: racing line, physics-verified speed profile, visibility guard, AWD model.
   For each: what it is, why it matters, and the evidence (a number or a plot from `figures/`).

**Viraj:** pull the united `main`, run `compare` (bicycle vs CARLA), then test the revalidated lateral fraction on CARLA.
Check rear slip and obstacle clearance as well as lap time; keep the default at 0.6 until confirmed.
**Saisi + Rodion:** follow up on the seed-2 invalid-corridor/centerline-fallback clearance issue separately.
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
