# Faster clean laps: review response and implementation plan

2026-10-17. Audience: Rodion, Viraj and Saisi. Status: **proposal, not implemented**.

## Decision and scope

Minimize total completion time subject to clean finishes, acceptable physical behavior and the
50 ms controller deadline. A lower path error or higher score alone is not proof of a faster lap.
Start with correctness and safe degradation, then exploit the resulting tracking headroom.

Reviewed integration code: `2621d15`, combining main `96a781e` (Rodion's planner/predictor optimization
`70cb357`) and Viraj's `viraj/corner-tracking` tip `3f0f6b2`. Viraj changes feedforward preview
0.2 -> 0.1 s and feedback gain 1.0 -> 1.5 in Params, and adds `tools/par_sweep.py`.
Tool-only follow-up `55fe531` adds `par_robust.py` and stacked/edge YAMLs, merged as `afe5523`
after driving tests; it does not change the tested controller code.
Rodion's `planner.py` and `predict.py` are identical to `96a781e`. Eco-mode work is explicitly
excluded at Rodion's request. Rodion chose to keep main at `96a781e`: the clean integration
remains on `rodion/viraj-optimization-sync` until lap time is recovered. Full measured results
and publication details belong in STATUS.md; the proposal's techniques remain unimplemented.

The locally supplied `CLAUDE_REVIEW_HANDOFF.md` reviewed historical `8acab13`; its source is
preserved unchanged. Its timing measurements came from another environment and are not our
current baseline. The review's formerly pending `70cb357` is already on main: do not redo its
O(K) passes, bounded grip/operator caches, finish release or cap-coordinate fixes.

No fix below has been implemented by this review/integration task. No grip/braking default is
being raised. Controller runtime remains numpy + Python stdlib, one core, no I/O or subprocesses,
and only the current 40 m perception horizon. No simulator edits or course-specific conditions.

## Which review findings still apply?

Ran the handoff's six diagnostic assertions against the integrated code. They deliberately assert
the faulty historical behavior; passing them confirms a defect, not successful safety validation.

| Finding | Reproduced behavior on integrated code | Owner and action |
|---|---|---|
| Invalid/blocked corridor silently becomes centerline | At 8 m/s with blockage 12 m ahead: throttle 1, brake 0, target 18 m/s, fallback 0 | Saisi + Rodion + Viraj: explicit infeasibility/stop policy |
| Production verification omits corridor | Same direct rollout passes unchecked; supplied corridor reports 2.458 m violation | Rodion + Saisi: containment plus a meaningful corrective response |
| Jerk limiting can undo grip limiting | Final requested acceleration 3.2 m/s^2, allowed bound including floor 0.78 | Viraj + Rodion: prioritize final grip constraint; mirror prediction |
| Stale slip remains on straight | After 50 straight updates: slip 0.4, scale 0.75 | Viraj: gate on fresh excitation or decay, preserve rear-slip protection |
| Planner YAML keys silently ignored | Requested verify_iters=-1 / rl_margin=1.2 still yield 1 / 0.5 | Viraj + Rodion: supported schema and unknown-key rejection |
| Physically too-narrow road remains valid | Road 1.8 m, vehicle 2.16 m: valid=True with +/-0.2 m band | Saisi: distinguish optional margin loss from physical infeasibility |

Reproduction is synthetic evidence. It does not establish the cause of any particular cone hit.
Previously main and Rodion's optimized code both hit unmapped obstacles on generated seeds 4/5;
use telemetry around those events to identify the causal path before claiming a fix.

## Delivery order

### 1. Make the proposed command and fallback physically meaningful

Owners: Viraj for tracker/controller, Rodion for predictor/planner, Saisi for corridor status.

- Apply jerk preference before the final allowed acceleration interval, or intersect intervals
  explicitly. If they conflict, grip/safety wins; store the actual requested acceleration.
  Existing nonzero safety floors remain explicit overrides, not proof of friction feasibility.
- Decay/gate the understeer estimate on unexcited straights. Do not remove the independent
  rear-slip check; use hysteresis to avoid rapid recovery/cutback oscillation.
- Represent corridor failure reason and first blocked station. Empty perception, physical
  blockage, optional-margin pinch and solver failure need different outcomes and telemetry.
- For a blockage, plan a stop before its near edge including vehicle extent, delay and a
  conservative attainable braking distance. If already outside the stoppable set, report that
  fact and use a least-risk emergency response; a late brake command cannot guarantee avoidance.
- Retain a previous obstacle-aware path only after transforming it to the current vehicle frame
  and revalidating it against fresh observations. A structurally valid path is not a safety check.

Acceptance: when a stop is required to remain inside the validated stopping set, the final
command may not accelerate out of that set toward the blockage. Physical
narrowness is rejected; a straight-to-corner command respects its final bound despite old jerk
history; corner-to-straight slip recovers while active rear sliding still reduces grip. Cover
planner-internal failure, controller exception and compute-budget degradation separately.

### 2. Make prediction faithful, then enforce footprint clearance

Owners: Rodion + Viraj jointly approve the state/FIFO contract; Rodion + Saisi approve geometry.

Pass a copied, read-only tracker snapshot into verification: PI integral, previous acceleration,
previous steering command, filtered slip and grip scale. Model the known command-delay FIFO, not
only a shifted tracking pose. Observations expose applied actuation, not the whole queued sequence;
Viraj must maintain the pending commands and define reset/overrun semantics, delay units in control
steps, and model-uncertainty assumptions; Rodion validates the matching predictor interpretation.
Do not claim knowledge of unseen harness state.

Use swept vehicle-footprint checks against the observed corridor and obstacle points, including
between prediction samples. The corridor is already shrunk by half-width and margin: avoid double
inflation. Either test corners against raw boundaries or add only heading-dependent excess width
to the shrunk-center constraints. Log clearance and violation separately from grip/slip demand.

Supplying `corridor=` alone is insufficient: today's speed factor reacts to grip/slip, so a
clearance-only failure can leave it at 1. Add a bounded response: regenerate a path candidate,
reduce speed where braking can affect the violation, or select the validated stopping policy.
Do not return the lowest-grip-demand candidate if all candidates violate hard clearance limits.

Acceptance: per-step tracker parity with nonempty history and 0/1/2-step delays, explicit FIFO
comparison to harness behavior, no mutation of live tracker state, and a full controller test
that rejects a grip-safe but corridor-unsafe rollout. Include heading, new-obstacle and
already-infeasible cases; use recorded model residuals rather than declaring the model exact.

### 3. Make configuration testable, then reduce verification cost

Owners: Rodion for predictor; Viraj + Rodion jointly approve the config schema and shared envelope API.

Before configurable experiments, expose intended planner controls through Params or another
explicit supported schema. Reject unknown keys, nonfinite values and invalid ranges at startup.
Acceptance: an actual controller-constructor/YAML test observes each supported override in the
effective planner settings; typoed keys fail visibly rather than silently selecting defaults.

Profile serial official-harness runs first, on cold starts and corner transitions as well as
steady straights. Report mean/p95/p99/max, cache hit rates, verifier iterations and fraction above
5 ms. Current planner timing includes verification; keep the separate residual timer visible.
Never compare parallel sweep compute scores with serial timing or raise deadlines to make a pass.

Candidate A: move exact AWD envelope preparation into one shared tracker helper. For each of the
41 acceleration candidates, prepare axle loads, traction eligibility and maximum allowed lateral
demand. Prefix minima can preserve the current helper's **first infeasible candidate** semantics;
a search can then replace repeated allocation of 41-element force arrays. Handle load clipping,
zero demand and equality edges exactly. Snapshot every dependency and bound caches. Benchmark
against the old helper over randomized geometry/params and nextafter boundary cases before use.
Analytic roots are a later option only with exact-grid parity and robust boundary fallback.

Candidate B: an anytime verifier with a mandatory first safety check and elapsed-time gates
before optional replans/candidate batches. On budget exhaustion select only an already validated
policy or the explicit degraded response from step 1. A time check cannot preempt an in-flight
Python/numpy operation, and skipping verification is not a safety mechanism.

Candidate C: reuse a validated trajectory only with an expiring certificate: fresh pose, state,
grip, obstacle geometry and clearance margins must remain within explicitly tested bounds.
Any new obstacle, reset, delayed-command discrepancy or margin loss invalidates it immediately.
Until those bounds exist, keep this in shadow mode; do not silently skip dynamic verification.

Acceptance: unchanged physical/parity assertions, bounded memory and no new collisions/overruns.
The engineering target is verified-planner p99 <5 ms on the target machine; this is **not an
achieved result or portable guarantee**. If the envelope is no longer dominant, stop optimizing it.

### 4. Optimize speed using reachable tire capacity, not a higher global fraction

Owners: Rodion with Viraj's shared tire-envelope API. Depends on steps 1-3.

Use squared speed x=v^2 and station progress to propagate attainable speed intervals. The existing
constant-deceleration pass cannot exploit different braking capacity on a straight versus a loaded
axle in a turn. Build conservative drive/brake limits from curvature, speed, load transfer, power,
grip estimate and delay, then backward-propagate controllable sets and forward-select the fastest
feasible profile. Start with a fixed path and a small speed grid; conservatively handle nonlinear
boundaries and validate the resulting rollout. This need not retain the current constant-limit
pass's exact O(K) formula or linear-program structure.

This adapts the reachability idea from [Pham & Pham's TOPP-RA paper](https://arxiv.org/abs/1707.07239)
to this vehicle; the paper does not validate our tire model, map-free horizon or implementation.
Implement only the needed numpy/stdlib mechanics, not an extra solver dependency.

Acceptance: independently checked per-axle feasibility including drag and power, attained stopping
distance, zero new penalties, and lower aggregate time on locked validation tracks. Test weak-grip
and delay perturbations. Keep today's fixed-fraction profile as a reference and rollback path.

### 5. Search a small set of dynamically trackable racing lines

Owners: Rodion; Viraj coordinates any changed steering policy. Depends on safe candidate rejection.

Minimum curvature is not minimum lap time. Keep the current warm-started QP as the seed, then
compare a small fixed batch (initially 4-8) of smooth corridor-constrained offset/speed candidates:
current line, slightly later apex, less lateral displacement, and a conservative clearance line.
Derive all variations from current visible geometry, never course names or absolute stations.

Rank only feasible candidates by predicted progress/travel time plus committed-path stability;
clearance is a hard rejection, not a soft penalty traded for speed. Include first-control and
terminal feasibility under observed geometry and explicit model assumptions; unknown obstacles
beyond the 40 m horizon are not covered by that claim. Reuse the batched dynamic model, but compare **closed-loop** candidates with the actual
tracker rather than only open-loop constant pedals. Measure end-to-end cost: the existing 32x30
rollout microbenchmark excludes feedback generation, geometry checks and controller overhead.

The longer-term direction is contouring/progress control or one sequential-convex update of line
and speed, inspired by [Liniger et al.'s racing formulation](https://arxiv.org/abs/1711.07300).
Its specialized-solver timings are not evidence that a numpy-only rewrite will meet our budget.
Do not start with a large dense nonlinear MPC solver or assume the rejected path-length surrogate
is safe just because it reduced some lap times.

Acceptance: lower held-out aggregate time, no per-course new penalties, bounded path-commit error,
physical metrics and deadline compliance. Reject the feature if candidate preparation costs more
than the measured benefit. No guaranteed percentage improvement is claimed.

## Promotion gates and experiment design

1. Freeze revision, config, backend, simulator pin and hardware. Change one mechanism per commit;
   ablate correctness, compute and behavior/tuning changes separately.
2. Run all STATUS checks and strict generated seeds 1-5 with three obstacle groups. Seeds 4/5 are
   now known regressions/diagnostics, not fresh evidence. Reserve new seeds (e.g. 6-10) before tuning.
   Their results must not guide candidate selection; once inspected for tuning, retire that set
   as holdout evidence and preselect a new untouched set before the next evaluation.
   A safety-policy fix must remove documented failures, not merely preserve their counts.
   A mechanical parity optimization may preserve a known failing baseline, but must label every
   failure and cannot be described as a clean pass. Mixed safety/time tradeoffs require an explicit
   team decision before promotion rather than declaring the result uniformly better.
3. Add representative low-grip, delay, noise, road-range and initial-offset perturbations after
   deterministic tests. Hard zeros: controller errors, new cones/off-course/missed gates and
   overruns on the locked nominal suite. A new safety failure blocks promotion.
4. For pure speed changes, primary metric is summed raw completion time over the same course set,
   with per-course times, full scorer penalties, DNF, rear slip and boundary margin shown. Reject
   aggregate gains that hide a new collision. Track score categories separately.
5. Repeat serial timing after warmup with a single run at a time and unchanged one-core/enforce
   settings. Record outliers and initial timing-test failures, not just the best rerun. Parallel
   `par_sweep.py` and `par_robust.py` are for search, not release evidence; independently verify
   completion, errors, overruns and all penalties from summary.json. Their measure mode does not
   disable the controller's own elapsed-time/next-step planner-skipping guard, so serial-equivalent
   driving under parallel load is not guaranteed even when the harness holds no commands.
6. Viraj runs CARLA compare on the exact revision/config before grip/braking default increases.
   No approved bicycle result is a guarantee for CARLA or unseen courses.

### Concrete validation manifest for the next implementation

Run from `simulator/` with `.venv/Scripts/python.exe`:

```powershell
.venv/Scripts/python.exe -m pytest ../tests -q
.venv/Scripts/python.exe -m autocross run -c ../hackuab/controller.py --controller-config ../tools/strict.yaml -t tracks/practice_technical.yaml --backend bicycle --no-render
.venv/Scripts/python.exe -m autocross run -c ../hackuab/controller.py --controller-config ../tools/strict.yaml -t tracks/practice_obstacles.yaml --backend bicycle --no-render
.venv/Scripts/python.exe -m autocross selftest -c ../hackuab/controller.py --no-render
```

Selftest covers hackathon_test, practice_basic, practice_loop, practice_obstacles and
practice_technical. Generate each selected seed outside the submodule, replacing N and PATH:

```powershell
.venv/Scripts/python.exe -m autocross gen-track --seed N --obstacles 3 --out PATH
.venv/Scripts/python.exe -m autocross run -c ../hackuab/controller.py -t PATH --controller-config ../tools/strict.yaml --backend bicycle --no-render
```

For timing-sensitive changes repeat serial selftest three times, retaining initialization and all
control calls, not only warmed medians. Report per-course sample counts and pooled timings.

Initial proposed robustness matrix: strict technical, obstacles and each reserved seed under
friction_scale=0.8, actuation_delay_steps=2, road_range_m=30 and start_lateral_offset=0.5,
plus road_offset_noise=0.05 m standard deviation, **one perturbation at a time** via
`--perturb KEY=VALUE`. These are future test settings, not
claims that they have passed. Compare identical settings against the locked baseline, then add
combined perturbations only after identifying individual failure mechanisms. Viraj must validate
CARLA-supported settings separately; do not assume backend equivalence.

## Suggested first implementation slice

Land focused jerk/grip and stale-slip tests/fixes with Viraj while Saisi/Rodion agree on corridor
status and safe fallback. Then wire state/FIFO and clearance into the verifier. Only after those
gates pass should Rodion invest in shared-envelope preparation, reachability speed planning and
small-batch line search. Those offer a credible path to faster clean laps without blind grip tuning.

The safety-filter architecture is inspired by [Wabersich & Zeilinger](https://arxiv.org/abs/1812.05506):
test a proposed input and substitute a feasible policy when necessary. Formal guarantees require
model uncertainty and terminal/backup assumptions that this repository has not yet established.
This proposal does not confer them. Do not claim global optimality or zero future improvements.

## Reader check

A fresh independent reader answered six questions about scope, reproduced defects, ownership,
advanced techniques, validation and guarantees. Its suggestions were incorporated: configuration
tests now have a delivery slot; stopping-set conditions and contract approvals are explicit;
terminal claims are horizon-limited; known-failure versus clean gates, holdout retirement and a
concrete validation manifest are stated. Recheck found no blocking ambiguity or unsupported
guarantee. This tests document clarity, not controller correctness or physical safety.
