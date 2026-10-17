# STATUS: where the project stands (handoff file for any AI agent)

**Update this file at the end of every task** (results table + what changed + next step) and commit it
with your work, so the next agent, whether Claude, ChatGPT/Codex or a teammate's, picks up exactly where you left off.

_Last updated: 12:47 PM, after commit `9dfa878` on `main`. Code freeze 5:00 PM, feature freeze 4:15 PM._

## What's on `main`

| Module | Owner | State |
|---|---|---|
| `hackuab/horizon.py` | Saisi | Obstacle-aware safe corridor (+ tests) |
| `hackuab/planner.py` | Rodion | **Min-curvature racing line** + forward/backward speed profile + visibility guard; AWD traction cap |
| `hackuab/predict.py` | Rodion | Dynamic AWD bicycle model (`rollout_dynamic`) + `verify_plan()` (+ 15 tests). **Not wired into the planner yet** |
| `hackuab/tracker.py` | Viraj | Latency compensation, curvature feedforward + pure-pursuit feedback, friction ellipse, slip adaptation |
| `hackuab/controller.py`, `params.py` | Viraj | Tiered fallback, budget guard, per-stage timing in `debug` |
| `docs/SPONSOR_RESEARCH.md` | Rodion | Sponsor/judging research; read it before writing the pitch |

## Latest results (bicycle backend, `selftest`)

| Course | Time | Cones | Technical /75 |
|---|---|---|---|
| hackathon_test | 54.0 s | 0 | 67.5 |
| practice_basic | 31.8 s | 0 | 70.6 |
| practice_loop | 89.1 s | 0 | 69.3 |
| practice_obstacles | 58.3 s | 0 | 67.7 |
| practice_technical | 40.6 s (ref 31.6) | 0 | 65.0 |
| **Mean** | | **0** | **68.0** (Phase 0 baseline: 62.3) |

p99 step time ≈ 3 ms / 50 ms, so there is plenty of compute budget for the verifier. All 55 tests pass. CARLA: not yet run on this version (Viraj, roadmap V5).

## Rodion's next tasks (in order)

1. **Wire `predict.verify_plan()` into `planner.plan()`** (roadmap R3 step 4). After the speed profile is built, call
   `verify_plan(plan, obs, vehicle, p, corridor=c)`. If `ok` is False, multiply `plan.v` by the returned `v_scale`
   (per node), re-run the backward pass so the braking stays consistent, and re-verify (at most 2 iterations). Hard budget:
   skip verification if the planner has already used > 8 ms this step. Log `pred_friction_max`, `pred_lat_dev_max`,
   `v_scale_min` and `verify_iters` to the debug dict (pass them via the tracker `state["debug"]` or ask Viraj for a hook).
2. **Use the verifier to go faster:** raise `lat_accel_frac` (currently 0.6) and `decel_frac` (0.4) step by step while
   the verifier keeps predicted friction use < 0.95. Target `practice_technical` ≤ 36 s with 0 cones, 0 off-course,
   and no grip-demand > 110 %. Test on 3+ random seeds too (`gen-track --seed N --obstacles 3`).
3. Stretch: sampling MPC (roadmap R3 step 5).
4. Write each feature up for judges in `docs/INNOVATION.md` (what it is, why, the evidence plot/number).

## How to verify any change (from `simulator/`, Windows)

```powershell
.venv\Scripts\python.exe -m pytest ..\tests -q
.venv\Scripts\python.exe -m autocross run -c ..\hackuab\controller.py --controller-config ..\tools\strict.yaml -t tracks\practice_technical.yaml --backend bicycle --no-render
.venv\Scripts\python.exe -m autocross run -c ..\hackuab\controller.py --controller-config ..\tools\strict.yaml -t tracks\practice_obstacles.yaml --backend bicycle --no-render
.venv\Scripts\python.exe -m autocross selftest -c ..\hackuab\controller.py --no-render
```

Merge to `main` only if tests pass, selftest has 0 errors, and **no course gets new penalties** compared with the table above.
