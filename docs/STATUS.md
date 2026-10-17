# STATUS: where the project stands (handoff file for any AI agent)

**Update this file at the end of every task** (results table + what changed + next step) and commit it
with your work, so the next agent, whether Claude, ChatGPT/Codex or a teammate's, picks up exactly where you left off.

_Last updated: 2026-10-17, corridor-aware verifier draft on `rodion/corridor-verifier`.
Baseline: published main `d35176d`. This draft is NOT merged; defaults are unchanged.
Clean timing and CARLA validation remain gates before promotion or any new default.
Code freeze 5:00 PM, feature freeze 4:15 PM._
_~3:40 PM: Eco mode merged into `main` together with PERFORMANCE / ECO mode selection (below)._

## Corridor-aware verifier draft (Rodion, `42918cd`)

**Decision: do not merge.** Rodion chose to finish the draft with load-affected measurements
rather than interrupt another session's export. Functional implementation and regression checks
are complete, but the observed score/runtime/rear-slip tradeoff does NOT qualify for promotion.
Higher lateral-fraction candidates were not started: first obtain a clean default validation.
CARLA is still Viraj's gate; no new default is selected.

Only `hackuab/planner.py`, `hackuab/predict.py`, `tests/test_corridor_verifier.py` and this shared
status file differ from baseline. No tracker, horizon/squeeze, controller, Params/default or
simulator changes. Earlier fusion/diagnosis history was deliberately removed from the PR by
isolating these commits onto `d35176d`; it remains on the separate local work branch. The baseline
controller is preserved at `d35176d` and in `runs/corridor-verifier/baseline/` (detached worktree).

### Behavior and contracts

- Pass the original `Corridor` through `plan -> _verify -> predict.verify_plan`. Its vehicle-width
  and clearance buffers are ALREADY applied: do not shrink the left/right bands again. Negative
  one-sided bounds and interpolated widths are supported.
- Project through interpolated center/normal cross-sections with three fixed vectorised Newton
  steps. Nearest-node widths are inaccurate on curves. Prefer genuine local inverses rather than
  false vertex wedges or remote normal rays across a hairpin. The initial half-cell covers the
  current offset vehicle pose without relaxing lateral limits; the far preview is not extrapolated.
- Return per-pose nonnegative `corridor_violation`, signed offset, centerline station, validity
  and coverage; report first breach step/time/centerline station/plan station. Existing
  `corridor_violation_max` remains signed for compatibility (negative = inside everywhere).
  Invalid/folded geometry or unobserved space fails closed.
- Clearance failures now affect BOTH `v_scale` and `demand_max`. A bounded 0.65-0.9 local speed
  factor starts before the breach, with a response lead for delay + speed preview + three steering
  time constants. Re-run braking AND acceleration reachability, then re-verify. This lead is a
  response heuristic, not a bound on uncertain dynamics.
- Prefer plans passing BOTH grip/slip and clearance. Normal correction retries are bounded by
  `verify_iters` and a 12 ms SOFT verifier budget, also reserving 5 ms for tracking within the
  controller budget. A call cannot be preempted; OS stalls can exceed that budget and are logged.
- If no safe plan is found, keep the obstacle-avoiding lateral path. A clearance fallback brakes
  to a stop before the breach; grip-only fallback uses local demand caps, not a blanket derate.
  Reverify fallback if budget allows. Otherwise `pred_verified=0`, `pred_ok=0`, `pred_unresolved=1`
  explicitly marks it UNVERIFIED. Other telemetry: fallback/clearance-fallback, budget exhaustion,
  initial/final violation and first breach. Do not retain zero stop caps forever; preserve useful
  nonzero grip-cap memory. An unresolved fallback is not called safe.

Limits: reference-point containment using the existing buffers, NOT a new swept-rectangle or
uncertainty guarantee. The predictor still starts PI/jerk/slip history afresh and lacks the live
tracker snapshot/delay FIFO. Slowing cannot repair every lateral/perception failure. Existing
blocked-corridor creep and Viraj's controller-level exception/previous-over-budget centerline
fallbacks bypass this verification and remain separate; this PR does not certify them.

### Default Performance validation versus STATUS and frozen main

All runs use bicycle, one core, enforce mode, no rendering, default PERFORMANCE (lateral fraction
0.75, v_max 30), with three obstacle groups on generated seeds 1-5. Each of the ten baseline and
draft runs finishes with **0 cones, obstacle hits, off-course events, missed gates and controller
errors**. This does NOT mean zero overruns or rear slip.

`STATUS time` is the historical 96a781e promotion table below (0.6 / 22 and older tracking), NOT a
matched A/B for this feature. The freshly frozen `d35176d` baseline has the current settings.

| Course | STATUS time | Fresh baseline time /75 | Draft time /75 | p99 ms baseline -> draft |
|---|---:|---:|---:|---:|
| hackathon_test | 51.357 | 49.766 / 69.38 | 50.308 / 60.04 | 11.73 -> 43.38 |
| practice_basic | 31.272 | 30.694 / 67.66 | 32.245 / 59.09 | 24.12 -> 50.10 |
| practice_loop | 88.646 | 86.139 / 63.25 | 90.904 / 56.98 | 32.89 -> 53.68 |
| practice_obstacles | 56.541 | 54.165 / 67.67 | 54.640 / 66.76 | 14.46 -> 25.38 |
| practice_technical | 38.549 | 36.840 / 67.32 | 36.917 / 63.65 | 12.58 -> 30.54 |
| seed 1 | 51.167 | 48.243 / 70.01 | 49.098 / 63.93 | 14.12 -> 37.98 |
| seed 2 | 62.398 | 61.843 / 64.30 | 63.205 / 62.85 | 27.22 -> 29.41 |
| seed 3 | 53.876 | 53.367 / 60.51 | 52.814 / 69.21 | 44.62 -> 12.08 |
| seed 4 | 58.419* | 57.390 / 67.30 | 57.444 / 68.17 | 15.90 -> 11.42 |
| seed 5 | 52.464* | 49.402 / 69.76 | 50.898 / 68.35 | 10.78 -> 13.09 |

*Historical 96a781e holdout seeds 4/5 each hit one cone. Fresh baseline and draft have none.

Public total: **257.604 -> 265.014 s** (+7.410 s / 2.88%); ten-course total **527.850 -> 538.473 s**
(+10.624 s / 2.01%). Public mean technical **67.056 -> 61.304 /75**; ten-course mean
**66.716 -> 63.903 /75**. Public category means (baseline -> draft): completion 25.744 -> 24.960,
deviation 14.456 -> 13.980, compute 12.132 -> 7.484, dynamics 14.718 -> 14.878. Not all of the
score loss is compute: timing/budget fallback also changes driving.

| Course | Rear-slip peak deg baseline -> draft | Rear-slip-over % baseline -> draft | Path mean/max m baseline -> draft |
|---|---:|---:|---|
| hackathon_test | 5.56 -> 7.00 | 0 -> 0.584 | .064/.228 -> .079/1.323 |
| practice_basic | 6.90 -> 5.02 | 2.52 -> 0 | .068/.429 -> .104/1.194 |
| practice_loop | 10.38 -> 7.99 | 2.19 -> 1.351 | .067/1.021 -> .086/1.097 |
| practice_obstacles | 5.03 -> 5.04 | 0 -> 0 | .073/1.112 -> .071/.224 |
| practice_technical | 5.26 -> 4.96 | 0 -> 0 | .061/.241 -> .061/.402 |
| seed 1 | 5.11 -> 5.00 | 0 -> 0 | .068/.224 -> .072/.786 |
| seed 2 | 7.91 -> 14.06 | 1.51 -> 1.477 | .080/1.127 -> .079/1.237 |
| seed 3 | 5.12 -> 5.03 | 0 -> 0 | .070/1.172 -> .059/.225 |
| seed 4 | 5.05 -> 4.91 | 0 -> 0 | .072/.214 -> .071/.221 |
| seed 5 | 4.87 -> 4.89 | 0 -> 0 | .070/.264 -> .070/.254 |

Frozen baseline has six deadline overruns across the ten comparison runs; draft has 41. A separate
Claude worktree's `export_viz.py --jobs 4` was observed overlapping the final draft batch. It was
not stopped or modified. These measurements cannot establish the isolated feature cost or prove
all failures are caused by load. In particular the draft seed-2 peak rear slip is unacceptable as
release evidence. Rodion explicitly chose to finish with these load-affected results.

Planner-stage wall time over the ten comparison runs: baseline mean/p99 **6.47/21.04 ms**, draft
**9.61/32.31 ms**; nested verifier **5.50/19.93 -> 8.49/30.12 ms**. No under-5-ms claim is justified.
Draft has 865 fallback-labelled calls, 858 unresolved, and 2,552 budget-exhaustion flags out of
10,959 planner calls. Failed/UNVERIFIED plans remain explicitly flagged, not silently certified.

Small hot-cache diagnostic (NOT release timing): gentle-circle verifier median without/with
corridor is 0.833/1.207 ms on logical CPU 0 and 1.523/2.246 ms on CPU 15, 200 samples each.
Only that diagnostic process's affinity changed and was restored; no harness settings changed.

### Checks, provenance and next gates

- Prescribed full pytest command: **131 passed, 4 failed (37.54 s)**. The same four inherited
  PERFORMANCE/ECO-zero golden assertions still expect pre-tuning 96a781e commands/38.548896 s.
  Neither expected outputs nor tolerances were weakened. They require an owner-coordinated
  frozen-config baseline update, not blind regeneration. Current full laps finish; exact old
  lap equality is intentionally no longer true. This is NOT an all-tests-green result.
- Focused planner/predictor/corridor suite: **67 passed (0.80 s)**, including all existing
  `test_predict.py` tests. Existing budgets pass unchanged: batch 2.94 ms (<3), verifier without
  corridor 0.839 ms (<2). 29 new tests cover curved/rotated/nonuniform projection, variable and
  negative bounds, preview edges/hairpins, malformed geometry, first breach, local caps, real
  narrow-vs-wide production response, dynamically verified fallback, budget exhaustion, grip
  memory and finite legacy diagnostics. Pytest's existing cache-permission warning is nonfatal.
- Required strict technical/obstacle checks ran: **36.899 / 56.421 s**, zero penalties/errors,
  p99 **29.29 / 39.19 ms**, **1 / 6 overruns**. Full five-course selftest and seeds 1-5 ran.
  Fresh technical and seed-2 reports were visually inspected.
- Driving validation snapshot `864d256` was transplanted byte-for-byte as `2b648fc` onto main;
  `42918cd` only fixes NaN diagnostics for optional no-corridor callers and is covered by the
  final tests. Normal passed-corridor driving is unchanged by that diagnostics-only follow-up.
- Earlier experiments are retained: curved-end/vertex false stops and a seed-2 remote-normal
  alias were found and fixed with regression tests. Earlier `4c050fc` (same source as `eb74126`)
  finished all ten with zero penalties/overruns, public mean 68.722, but still added rear slip
  on some courses. It is not the final refinement's clean validation and must not substitute for it.

Ignored artifacts: `runs/corridor-verifier/{baseline,localized-final}/validation/` hold summaries,
telemetry, reports, command logs and results.json; `projection-fix/`, `candidate/`, `candidate-final/`
retain failed/earlier experiments. `validate.py` runs serially; `compare.py` reads the summaries;
`benchmark.py` is the diagnostic. Original local CLAUDE_REVIEW_HANDOFF.md remains unmodified/untracked.

Before promotion: coordinate an idle-machine matched A/B on the exact current defaults; require
clean deadlines and no unacceptable score/path/slip regression, then Viraj's CARLA confirmation.
Only then run higher lat_accel_frac candidates separately (never change defaults from bicycle
alone). Integrate Saisi's squeeze fix in a separate comparison/PR; coordinate live tracker-state
and controller-level fallback contracts separately. This draft leaves main untouched;
`d35176d` is the preserved known-good baseline.

## Fixed: blocked corridor (~3:45 PM, `rodion/blocked-corridor`)

When an obstacle leaves no gap the car fits through (`build_corridor` returns `valid=False`), the planner
used to return a normal, accelerating centerline plan: full throttle at the obstacle, target 18 m/s,
fallback 0, in both modes (review finding 1 in docs/CLAUDE_REVIEW_HANDOFF.md, reproduced on `1d93fac`).
Now `planner._blocked_plan` brakes to `blocked_creep_speed` (2 m/s, so a transient blockage or a cone wall
can't cause the 8 s "stuck" DNF; set 0 for a true stop) at a stop point `blocked_stop_margin` (1 m) before
the car's nose reaches the first blocked station, and the speed target never exceeds the current speed.
Logged as `dbg_blocked` / `dbg_blocked_s`. Tests: `tests/test_blocked.py` (final Command in both modes,
closed-loop slow-down, gap control case).
The blocked plan never triggers on any public course (0 of ~11k steps, both modes), so lap times are
unchanged (PERFORMANCE identical; ECO practice_obstacles 59.30 s = before, with 0 overruns).
Finding 2 (corridor verification) is addressed by the unmerged draft above. Still open: the jerk limit can undo
the grip clamp (3), the stale understeer estimate on straights (4), unknown YAML keys silently ignored (5),
and narrow-road clamping (6).

## Historical upstream driving modes (before tracking/speed/verifier updates)

The following tables and golden-test claims describe the older upstream snapshots, not a green
test or release-validation claim for the current defaults or this verifier draft.

One controller, two selectable modes. The mode is picked **only** by `mode:` in a `--controller-config` YAML
(`params.MODES` holds each mode's values), never by course or position. No config means PERFORMANCE.

| Mode | Config file | What it does | Run it (from `simulator/`) | Its tests |
|---|---|---|---|---|
| **PERFORMANCE** | `hackuab/performance.yaml` (or no config) | Fastest legal lap: `eco_weight 0`, the scored submission | `.venv\Scripts\python.exe ..\tools\run_mode.py performance selftest` | `.venv\Scripts\python.exe -m pytest ..\tests -m performance` |
| **ECO** | `hackuab/eco.yaml` | Energy-optimal lap: `eco_weight 0.8` (lift-and-coast, gentler accel, lower top speed) | `.venv\Scripts\python.exe ..\tools\run_mode.py eco selftest` | `.venv\Scripts\python.exe -m pytest ..\tests -m eco` |

- One lap: `..\tools\run_mode.py eco run practice_technical` (add `--view` to watch, `--strict` to re-raise errors,
  `--backend carla` for CARLA). Any extra autocross flags pass through.
- Fast unit tests only: `-m "eco and not sim"`. Full simulator laps are marked `sim`.
- An unknown mode (e.g. a typo `mode: ecco`) raises in `on_start` instead of silently driving the wrong profile.
- Keys set explicitly in the YAML override the mode's values (e.g. `mode: eco` + `eco_weight: 0.6`).

Selftest at merge (bicycle, 0 cones / 0 off-course / 0 gates / 0 errors on every course in every row):

| Course | No config | PERFORMANCE | ECO | `main` before the merge |
|---|---|---|---|---|
| hackathon_test | 51.4 s · 68.6 | 51.4 s · 68.6 | 55.5 s · 66.3 | 51.4 s · 68.6 |
| practice_basic | 31.3 s · 70.3 | 31.3 s · 70.3 | 33.0 s · 69.0 | 31.3 s · 70.3 |
| practice_loop | 88.6 s · 68.9 | 88.6 s · 68.9 | 96.0 s · 66.9 | 88.6 s · 68.8 |
| practice_obstacles | 56.5 s · 68.1 | 56.5 s · 68.1 | 59.3 s · 66.8 | 56.5 s · 68.0 |
| practice_technical | 38.5 s · 65.9 | 38.5 s · 65.8 | 41.7 s · 63.3 | 38.5 s · 65.9 |
| **Mean technical** | **68.3** | **68.3** | **66.5** | **68.3** |

Lap times are identical between no config, PERFORMANCE and pre-merge `main`. The ±0.1 differences are the compute score
reacting to step-time noise (p99 ≈ 8–9 ms). ECO trades technical points (completion time) for energy; its
energy numbers are in the next section. 100 tests pass (`-m performance`: 7, `-m eco`: 10, the rest are mode-independent).

**For Viraj:** `params.py` gained `MODES`, `resolve_mode()` and the `mode` handling in `from_vehicle()`;
`tools/run_mode.py` is new; `pytest.ini` (markers) is new at the repo root.

## Eco mode (merged into `main`; innovation add-on for the judges)

**Pitch:** same controller, one knob. `eco_weight` 0 gives the fastest legal lap and 1 the
lowest-energy lap, and the trade-off is measured on a Pareto front. Selected only through
`--controller-config ..\hackuab\eco.yaml` (eco_weight 0.8). The default (`eco_weight: 0.0`) is
bit-identical to main 96a781e. `tests/test_eco.py` checks this two ways: a golden 140-step
closed-loop command sequence recorded from main, and a full bicycle run that must reproduce main's
38.549 s on practice_technical.

**Headline (measured, bicycle):** Eco (eco_weight 0.8) uses **36 % less energy (Wh/km) for +6.2 %
lap time** on average over practice_technical, hackathon_test and gen-track seeds 1/2/3
(--obstacles 3), with **0 cones, 0 off-course, 0 missed gates and 0 controller errors** on all five
courses. Per course: energy −24 % to −41 %, lap time +2.8 % to +8.2 %.

What Eco does (planner.py, gated by `eco_weight > 0`; all scaled by eco_weight):
1. **Lift-and-coast.** A coast-then-brake reachability pass (vectorised like the backward pass).
   Before each braking zone the profile follows the car's own coast-down deceleration (drag +
   rolling, ~0.2 m/s²) and brakes only where coasting can no longer make the corner. The coast
   length is limited to what fits in the 40 m view, so the car never brakes just to get onto a coast line.
2. **Gentler acceleration** (forward pass ×(1 − 0.5·w)) and a **lower top speed** (cruise ×(1 − 0.35·w)
   of the horizon-guard top speed; drag power grows with v³). The forward-pass seed is shorter
   (×(1 − 0.5·w)), which cuts speed-PI wind-up and the overshoot braking that follows it.
3. Every eco plan still goes through `predict.verify_plan()`. Debug values: `dbg_eco_weight`, `dbg_coast`,
   `dbg_regen_limited` (always 0 because regen-first braking was not built), `dbg_eco_v_cruise`, `dbg_eco_coast_m`.

| eco_weight | Mean lap vs 0 | Worst lap vs 0 | Mean energy vs 0 | Clean on all 5 and within 10 % |
|---:|---:|---:|---:|---|
| 0 (Performance) | 0 % | 0 % | 0 % | yes |
| 0.2 | −0.0 % | +0.6 % | −3.5 % | yes |
| 0.4 | +0.9 % | +1.4 % | −11.8 % | yes |
| 0.6 | +2.8 % | +4.0 % | −23.0 % | yes |
| **0.8 (recommended)** | **+6.2 %** | **+8.2 %** | **−35.8 %** | **yes** |
| 1.0 | +12.1 % | +15.8 % | −49.4 % | no: 1 cone on practice_technical (cone 231, t = 34 s), +15.8 % |

| Course | Performance: lap / Wh/km / points | Eco 0.8: lap / Wh/km / points | Baseline PP: lap / Wh/km / penalties |
|---|---|---|---|
| practice_technical | 38.55 s / 675 / 65.92 | 41.71 s / 413 / 63.19 | 39.47 s / 492 / 8 |
| hackathon_test | 51.36 s / 630 / 68.53 | 55.45 s / 374 / 66.24 | 55.98 s / 433 / 4 |
| seed 1 | 51.19 s / 706 / 68.79 | 54.94 s / 419 / 66.77 | 56.27 s / 519 / 8 |
| seed 2 | 62.40 s / 651 / 68.58 | 64.14 s / 494 / 67.76 | 67.96 s / 400 / 6 |
| seed 3 | 53.88 s / 692 / 68.15 | 56.47 s / 454 / 67.32 | 58.64 s / 469 / 6 |

Eco 0.8 is faster than the baseline pure pursuit on 4 of 5 courses and uses less energy on 4 of 5
(not on seed 2). Unlike the baseline, it has no penalties. Eco costs technical points (the score
rewards time), so **it is a demo and innovation feature, not the submission default.** Full
per-weight table: `tools/figures/eco_pareto.md`; raw data: `tools/figures/eco_pareto_results.json`.

**Energy meter (`tools/energy.py`).** It rebuilds the wheel forces from telemetry.csv exactly like
`bicycle.substep()`: power-limited drive force, brake force, AWD 40/60 drive split, 60/40 brake split,
and the traction-control clip `|Fx_axle| ≤ traction_limit·mu·Fz_axle` with load transfer from the
previous tick's ax. It integrates the clipped axle forces with dt taken from column t. The plain
longitudinal balance (E_trac − E_brake − E_drag − E_roll − ΔKE) leaves 7–13 % unexplained. That
remainder is cornering tyre scrub, the work of the lateral tyre forces, which the meter rebuilds with
`bicycle._lat()` from vy, yaw rate and steer angle. With it, **the balance closes to within ±0.05 % on
every run** (tests: `tests/test_energy.py`, synthetic accelerate/cruise/brake with a known answer).

**Assumptions (not Blazer EV specs; CLI flags):** eta_drive = 0.90 (battery to wheel),
eta_regen = 0.65 (wheel to battery), P_regen_max = 60 kW. E_batt = E_trac/eta_drive − eta_regen·∫min(P_brake, P_regen_max)dt.
The simulator has no electric drivetrain and no regen, so battery Wh are an estimate. The wheel-level
terms (traction, drag, rolling, brake, tyre) are exact to the bicycle model. Note that ΔKE at the
finish line counts as energy used. Performance crosses the finish of practice_technical at about 20 m/s
(108 Wh of 335 Wh). Without that term Eco still saves 40 % there (227 → 137 Wh).

**CARLA:** not measured (Viraj owns CARLA). CARLA telemetry has no pedal-to-force model we can mirror,
so there the tractive force would have to come from m·ax + drag + rolling. That is a proxy that cannot
separate brake heat from tyre losses.

Demo figures (`tools/figures/`): `eco_pareto.png` (lap time vs energy, one line per course, the
recommended point marked, baseline in grey), `eco_energy_breakdown.png` (where the energy goes for
baseline / Performance / Eco on practice_technical), `eco_speed_trace.png` (Performance vs Eco speed
with Eco's coasting stretches shaded).
Reproduce from simulator/: `..\tools\eco_pareto.py --out <tmp>`, then
`..\tools\eco_figures.py --results <tmp>\results.json --eco 0.8`.

Verification on this branch (bicycle):
- 91 tests pass: 84 inherited from main, 3 for the energy meter (`tests/test_energy.py`) and 4 Eco tests,
  including the eco_weight 0 golden sequence and the lap-time regression.
- Default selftest (`runs/20261017-150854_selftest`): times identical to main 96a781e (51.357 / 31.272 /
  88.646 / 56.541 / 38.549 s), 0 penalties, 0 controller errors, 0 overruns, p99 8.3–9.5 ms.
- Eco selftest with `hackuab\eco.yaml` (`runs/20261017-150952_selftest`): hackathon 55.45, basic 32.95,
  loop 96.01, obstacles 59.30, technical 41.71 s; 0 penalties, 0 controller errors, 0 overruns.
  Its p99 of 7.7–25 ms is machine jitter: the same eco technical run gave 14.5 ms and then 8.4 ms back to back,
  and the Pareto sweep's eco runs measured p99 7.9–10.2 ms. The eco planner adds about 1 ms mean.

Limits: lift-and-coast contributes little on its own (Eco coasts about 2 s per lap on
practice_technical), because the 40 m view leaves little room to coast before a corner. Most of the
saving comes from the lower top speed and gentler acceleration, which leave less kinetic energy to
brake away. Not built: regen-first braking (b) and lowering lat_accel_frac. The eco_weight 1.0 cone
was not investigated.

## Latest validated code: lap-time optimization (Rodion)

Goal: lower clean completion time, not necessarily maximize every scoring category. Only Rodion's
planner/predictor and their tests changed; tracker, horizon, controller, Params and the pinned
simulator are untouched. The united-main description and historical tables below remain reference baselines.

Changes:

- Exact O(K) braking/acceleration reachability replaces O(K squared) all-pairs matrices. Cached,
  read-only derivative operators and fewer temporary arrays reduce planner overhead.
- Horizon-end racing-line fade is 4 m / weight 0.5 instead of 8 m / 2.0. All other corridor,
  curvature, grip, braking and verification constraints remain in place.
- A finite finish inside the visible corridor releases only the speculative unseen-hairpin cap;
  it does not release limits for visible corners. No course names or hidden geometry are used.
- Remembered verifier caps now use centerline station for both storage and lookup, matching
  `obs.progress`, rather than mixing progress with offset-path length.
- Predictor sanitization uses tracker's scalar helper. Bounded AWD memoization calls the actual
  tracker helper at cell extremes and reuses only exactly equal plateau answers; boundary cells
  call the original helper at the actual input. No approximate interpolation or copied physics.

### Default-parameter results versus STATUS's united main

Times are seconds; points are technical /75. Every row finishes with zero cones, off-course
events, missed gates, obstacle hits, controller errors and overruns.

| Course | United main time / points | Optimized time / points | Time saved |
|---|---:|---:|---:|
| hackathon_test | 51.949 / 68.40 | 51.357 / 68.47 | 0.592 |
| practice_basic | 31.616 / 70.06 | 31.272 / 70.21 | 0.344 |
| practice_loop | 88.975 / 68.90 | 88.646 / 68.77 | 0.329 |
| practice_obstacles | 57.281 / 67.34 | 56.544 / 67.88 | 0.737 |
| practice_technical | 38.701 / 65.88 | 38.549 / 65.82 | 0.152 |
| Random seed 1 | 51.587 / 68.94 | 51.167 / 68.95 | 0.420 |
| Random seed 2 | 62.879 / 68.79 | 62.398 / 68.51 | 0.481 |
| Random seed 3 | 54.359 / 68.05 | 53.876 / 68.09 | 0.483 |

Total time across these eight courses: 437.347 -> 433.809 s (3.538 s / 0.81% faster).
Mean public technical score: 68.116 -> 68.230. Some individual scores fall despite faster laps;
do not describe this as a uniform scoring improvement. Default selected p99 is 8.3-11.0 ms,
well below 50 ms, with about 34 MB memory. CPU results do not establish a universal speedup:
memoization has cold misses and the shared AWD helper remains the predictor's dominant cost.

Planner-stage wall times from 8,950 controller calls in the five public courses and seeds 1/2/3:
including verification, mean **4.23 ms**, p99 **7.61 ms**, maximum **37.00 ms**; 23.43% exceed
5 ms. Excluding the nested verification timer, mean is 0.88 ms and p99 1.30 ms. Thus the
line/speed planner is comfortably under 5 ms, but the complete verified planner is only under
5 ms on average, not at p99. These are measured bicycle timings on Rodion's machine, not a
hard runtime guarantee or a fresh CARLA measurement.

### Extra holdouts: existing obstacle failures, not a clean-course claim

Generated seeds 4/5 only after selecting the code. Both hit an unmapped obstacle cone, so neither
is a clean pass. An isolated untouched `8acab13` worktree reproduces the same cone IDs at nearly
the same progress. The branch is faster with **no new penalties**, but does not fix these failures.

| Extra seed | Untouched main time / points | Optimized time / points | Cones main / branch |
|---|---:|---:|---:|
| 4 | 58.797 / 65.12 | 58.419 / 65.11 | 1 / 1 (unmapped cone 437, progress ~78 m) |
| 5 | 52.619 / 65.87 | 52.464 / 65.86 | 1 / 1 (unmapped cone 401, progress ~401 m) |

All four runs finish without off-course/missed gates/controller errors/overruns. This is a
pre-existing obstacle-handling limitation requiring the Saisi/Rodion follow-up, not evidence
of general unseen-course safety. No course-specific workaround was added. Reports are under
`runs/optimization/selected-default/holdouts/` and `runs/optimization/baseline-holdouts/`;
the isolated baseline is `runs/optimization/baseline-main/` (all ignored).

### Verification and review

Promotion recheck (2026-10-17, ~2:50 PM): fetched `origin/main` was still `8acab13`, so the
reviewed code can be fast-forwarded without a controller-code merge. Re-ran every required
check on `70cb357`; only this handoff documentation changed afterward.

| Check | Previously recorded optimized result | Pre-merge recheck |
|---|---:|---:|
| Unit tests | 84 passing | 84 passing |
| Strict technical time | 38.549 s, clean | 38.549 s, clean |
| Strict obstacles time | 56.541 s, clean | 56.541 s, clean |
| Full selftest | 5 clean finishes, mean 68.230 /75 | 5 clean finishes, mean 68.370 /75 |

The recheck has zero cones/off-course/missed gates/controller errors/overruns, with selftest
step p99 8.2-9.1 ms. Differences in compute scores do not imply changed controller behavior.
Fresh reports: `runs/merge-validation/20261017-144906_selftest/`; strict runs are siblings
`20261017-144846_controller_practice_technical` and `20261017-144855_controller_practice_obstacles`.
The previously documented random-course evidence applies to the identical controller code.

Required strict technical/obstacle runs finish cleanly in 38.549 / 56.541 s (p99 8.16 / 8.96 ms).
Default selftest and strict random seeds 1/2/3 (`--obstacles 3`) all finish cleanly. Artifacts:
`runs/optimization/selected-default/20261017-141708_selftest/`, its sibling `random/`, and
`runs/optimization/final-strict/` (ignored local files). Technical report visually inspected.
All **84 tests pass**, including unchanged predictor timing budgets. Pytest emits only an
existing cache-directory permission warning; collection and assertions complete normally.

Independent read-only review found no blocking implementation issues. Its documentation and
next-step cap-lookup test suggestions were addressed and re-reviewed. Tests cover exact speed
passes on nonuniform grids, curvature invariance, finish visibility, both remembered-cap storage
and lookup, randomized AWD parity/quantization edges, parameter mutation and existing per-step
tracker pedal parity. This is evidence, not a guarantee of global optimality or unseen-course safety.

Rejected experiments: a path-length surrogate cut some laps but introduced mapped-cone hits;
it was removed completely. Stronger warm-start weights were slower on basic and not adopted.
No generic obstacle safeguard was added, consistent with Rodion's earlier scope decision.

Parameter-only experiments on this exact code: `lat_accel_frac: 0.8`, `decel_frac: 0.6`
cut technical to 34.775 s; `0.85 / 0.7` cut it to 33.998 s. Both pass all five public courses
and seeds 1/2, but **both hit one cone on seed 3** (49.306 / 48.119 s). Neither qualifies as
a clean all-course recommendation. Rear sliding also increases; `0.85 / 0.7` reaches 13.6 degrees
on basic. Default fractions remain unchanged. Artifacts: `runs/optimization/race08-06/` and
`race085-07/` (full selftests in `full/`, random seed 2 in the root, other seeds in `random/`).
Braking-only `decel_frac: 0.6` at default lateral fraction also fails: technical is faster at
36.856 s but hits one cone; seed 3 also hits one cone. It is not recommended either
(`runs/optimization/brake06-final/`). It retains the seed-4/5 obstacle hits as well.

Review follow-ups requiring other owners:

- **Viraj + Rodion:** expose a read-only live tracker-state snapshot to verification (PI, jerk,
  steering-command and filtered-slip histories). Current prediction starts that history afresh.
- **Saisi + Rodion:** coordinate corridor-aware verification and a safe invalid/blocked-corridor
  policy. Planner currently does not pass the corridor to `verify_plan`; `pred_ok` therefore does
  not certify containment. Existing invalid-corridor centerline fallback remains a known risk.
- **Viraj:** confirm the exact branch and any opt-in parameter preset on CARLA before adopting
  new grip/braking defaults. Default lateral/deceleration fractions remain 0.6 / 0.4.

## United code state at consolidation (`8acab13`, before optimization)

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

## Historical consolidation validation (default params, bicycle, `8acab13`)

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
1. Keep this verifier draft unmerged. Coordinate an idle-machine matched default A/B, reconcile
   the inherited golden/config expectations with their owner, and inspect score/slip/runtime
   tradeoffs before any promotion. Preserve `d35176d` as the comparison controller.
2. Obtain Viraj's exact-revision CARLA check. Only after clean default validation, evaluate
   higher lateral fractions separately; do not select a new default without CARLA. Coordinate
   live tracker-state/FIFO and controller-level fallback contracts separately from this PR.
3. Write `docs/INNOVATION.md`: racing line, physics-verified speed profile, visibility guard, AWD model.
   For each: what it is, why it matters, and the evidence (a number or a plot from `figures/`).

**Viraj:** compare the exact default baseline/draft on CARLA, including slip, clearance and runtime.
Defaults inherited from main remain lateral/deceleration 0.75 / 0.4, v_max 30; this PR does not
change them. Coordinate jerk/grip priority, stale-slip recovery and baseline tests with their owners.
**Saisi + Rodion:** keep squeeze-fix integration separate initially, then run an explicit combined
comparison. Buffered reference-point checking does not by itself fix every narrow/blocked corridor.
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
