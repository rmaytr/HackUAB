# Innovations in the final HackUAB controller

The final selected controller is verifier-enabled `e3b8a28`. It uses the visible road rather than a course map and returns a command every 50 ms. [STATUS](STATUS.md) records the full tests and measured results; [README](../README.md) explains how to run it.

## 1. A racing line solved from current perception

**What:** `planner.py` solves lateral offsets within the corridor using a regularized minimum-curvature objective and projected bound handling. Start-position/heading terms connect the line to the car; shifted warm starts keep consecutive published paths consistent.

**Why:** a smoother line permits more speed for the same lateral acceleration and makes the committed path easier to track.

**Evidence:** all ten final PERFORMANCE public/generated laps scored 100 in the deviation category, with committed-path error at most 0.308 m. The technical course finished in 36.840 s versus the original pure-pursuit 39.47 s; that difference reflects the whole controller, not this feature alone.

## 2. Physics-verified speed planning

**What:** curvature sets a cornering cap. Exact O(K) reachability passes impose braking and power-limited acceleration. A dynamic AWD bicycle rollout predicts 1.5 s ahead at 50 ms resolution, including tyre saturation, longitudinal load transfer, drag, rolling resistance and steering dynamics. Predicted tyre utilisation, grip demand or rear slip above the limits returns local speed caps and may trigger one replanning pass (`verify_iters=1`). Caps relax across subsequent observations.

**Why:** curvature alone ignores transient vehicle dynamics and the response of the controller that will execute the plan.

**Evidence:** verification diagnostics appear on 24,357 final standalone control samples. All 129 tests pass, including exact tracker/pedal and multi-step slip-state parity tests. The planner does not supply the corridor to verification, and the rollout starts its PI/jerk history afresh; this is a bounded prediction check rather than a certificate of safety or corridor containment.

## 3. A guard for limited visibility

**What:** a hypothetical tight corner just beyond the visible horizon limits horizon-end speed; backward braking carries the constraint toward the car. A visible finish releases only this speculative constraint.

**Why:** with only 40 m of preview, the controller must allow for geometry that has not appeared yet.

**Evidence:** all five generated seeds with three obstacle groups finished without driving penalties in both modes. This finite validation does not prove safety on all hidden corners or road conditions.

## 4. AWD-aware tracking and online slip adaptation

**What:** the tracker evaluates front/rear friction separately with 40/60 drive and 60/40 brake splits and load transfer. Curvature feedforward and pure-pursuit feedback steer the car; PI control, friction constraints and slew/jerk controls shape pedals. Measured yaw response and rear slip adjust the grip scale used by planning.

**Why:** the driven front axle shares grip between acceleration and turning, while braking unloads the rear. A single longitudinal limit misses these effects.

**Evidence:** no commanded-grip-over-110% or pedal-overlap violations were measured in the 22 standalone laps. PERFORMANCE technical dynamics scored 100. Basic and seed 2 retained rear-slip peaks of 6.898/7.906 degrees and dynamics scores 96.17/98.24; slip adaptation does not eliminate every slide.

## 5. Perception-based obstacle handling and degraded control

**What:** `horizon.py` subtracts vehicle width and clearance margins, cuts a usable corridor around observed obstacles, and favors a consistent passing side. Invalid/blocked corridors select a plan that brakes toward 2 m/s creep rather than accelerating through a blockage. Integration failures and compute-budget trips select cheaper fallback control.

**Why:** obstacles are discovered through perception, and failure behavior must be defined for the same observation stream as normal driving.

**Evidence:** final public courses and seeds 1-5 in both modes all had zero cone/obstacle hits, off-course events, missed gates and controller errors. Blocked-road tests cover slow-down and passable-gap controls. Creep and fallback are mitigations, not a guaranteed stop or collision-free route.

## 6. PERFORMANCE and ECO with an auditable energy estimate

**What:** PERFORMANCE (`eco_weight=0`) is the default. ECO (`eco_weight=0.8`) adds lift-and-coast, gentler acceleration and lower cruise speed. Both retain prediction and the same safety thresholds. `tools/energy.py` reconstructs wheel forces and energy, including cornering tyre scrub, then applies assumed drive/regen efficiencies.

**Why:** the same planning system can expose a measured trade between completion time and energy instead of requiring a separate controller.

**Evidence:** across the final five public courses, estimated battery energy fell from 2203.49 to 1479.66 Wh (**32.85%**) for **8.08%** more total lap time. Across the twenty exported final preset runs, maximum absolute energy-balance residual was 0.030% (rounded). Assumptions: 90% drive efficiency, 65% regeneration efficiency, 60 kW regen cap. The simulator has no battery or regen; these are generic model assumptions, not Blazer EV specifications or CARLA measurements. Historical Pareto sweeps remain labeled separately in the visualization data.

## 7. Faster computation while keeping verification enabled

**What:** reuse AWD limits only on intervals whose endpoint answers from the shared tracker helper are exactly equal, at the exact grip scale. Cache dependency-complete acceleration tables and node tangents. Share interpolation station searches and use scalar slip updates instead of NumPy scalar dispatch. Caches are bounded and callers receive independent mutable planner tables.

**Why:** redundant numerical work costs compute points and reduces the time available for prediction. Removing that work preserves the model, horizon and safety limits.

**Evidence:** paired diagnostic cold/warm verifier medians improved from 2.930/0.866 ms on original main to 1.692/0.613 ms, approximately 42%/29%. Final PERFORMANCE full-run p99 was 6.957-10.597 ms with zero overruns. ECO retained a 52.060 ms overrun on seed 2 and a separate 38.128 ms soft-budget excursion on seed 4; the faster kernels do not guarantee bounded tail latency.

## 8. Offline camera, LiDAR and radar fusion

**What:** `fusion/` filters/clusters LiDAR, associates camera colour and radar returns, and bridges fused objects into the controller's observation types. A full-controller demo exercises the bridge.

**Why:** it demonstrates a perception front end for a controller that ordinarily receives an already-processed road model.

**Evidence:** the existing six-frame full-controller smoke check measured 1.42 ms fusion median, 8.44 ms p95 and 10.71 ms maximum, with 66/70 exact-colour matches within 0.5 m. A fused obstacle changed steering from 0.000 to -0.049 with fallback 0. This is synthetic/offline evidence. Raw sensors are not provided by the scored Observation API, and fusion is not imported by the scored controller. Real CARLA perception accuracy is not certified.

## Evidence and remaining limits

The final single-pass bicycle validation comprises 129 passing tests and 22 penalty-free standalone finishes. Public means were 70.066/75 PERFORMANCE and 67.618/75 ECO. The score is an average on known public courses, not the official weighted unreleased-course result. Exact-code CARLA validation, ECO tail-latency investigation and full live tracker-state/corridor verification remain outstanding. Rodion explicitly selected this version for final main with those limits recorded.
