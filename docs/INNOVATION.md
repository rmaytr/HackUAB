# INNOVATION: what is new in our controller, why it matters, and the evidence

The EcoCAR Innovation Challenge RFP lists **"Predictive vehicle control for an Autocross course"** as an innovation
opportunity. That is what we built: a controller that predicts what the car will do before it commits to a plan,
with no map, using only the 40 m of road it can see.

Each section below has three parts:

- **What:** the idea in plain English, then the maths.
- **Why it matters:** in the language of the EcoCAR RFP, as quoted in [SPONSOR_RESEARCH.md](SPONSOR_RESEARCH.md).
- **Evidence:** a figure and a measured number.

**How to read the numbers.** Values in `{{FINAL: …}}` come from the latest run and will be replaced by the numbers from
the locked submission code. "Pre-PR #7" marks a figure or number from before the corner-tracking tracker and the
`lat_accel_frac` 0.6 → 0.75 change. Unless stated otherwise, measurements are on the bicycle backend, on one CPU core.

| # | Innovation | Where it lives | Headline evidence |
|---|---|---|---|
| 1 | Minimum-curvature racing line, re-solved every 50 ms | `planner.py` | Path-deviation score {{FINAL: 100 / 100}} on every course vs 86.7–94.1 for the baseline |
| 2 | Physics-verified speed profile | `planner.py` + `predict.py` | Model matches the harness to 0.13 m after 1.5 s (kinematic model: 0.54 m) |
| 3 | Horizon-end visibility guard | `planner.py` | 0 off-course events on 3 unseen random courses |
| 4 | AWD per-axle friction model | `tracker.py` | Dynamics score {{FINAL: 100 / 100}} on practice_technical |
| 5 | Online understeer/slip adaptation | `tracker.py` | {{TODO: lowest a_lat_scale reached in a lap}} |
| 6 | Obstacle-aware corridor + blocked-road safety stop | `horizon.py` + `planner.py` | practice_obstacles: {{FINAL: 0}} cones vs 3 for the baseline |
| 7 | PERFORMANCE vs ECO on one knob, with an energy meter and a Pareto front | `planner.py`, `params.py`, `tools/energy.py` | {{FINAL: −27.1 %}} energy for {{FINAL: +7.1 %}} lap time, 0 penalties |
| 8 | Sensor-fusion tool (not part of the scored controller) | `fusion/` | 1.68 ms median per frame on synthetic data |

---

## 1. Minimum-curvature racing line, re-solved every 50 ms

**What.** Instead of following the centerline, the car drives the smoothest line that fits inside the safe corridor:
it swings wide before a corner, clips the inside at the apex and exits wide, which lowers the curvature and so raises
the speed the tyres allow. Because there is no map, the line is solved again from scratch every 50 ms on the 40 m that
is visible.

The unknowns are lateral offsets $n_i$ along the corridor normals, one per metre (41 of them), bounded by
$-\text{right}_i \le n_i \le \text{left}_i$. For small offsets the path curvature is linear in $n$:

$$\kappa \approx \kappa_c + \left(D_2/\Delta s^2 + \operatorname{diag}(\kappa_c^2)\right) n$$

so minimising $\lVert\kappa\rVert^2$ plus a few regularising terms is one small least-squares solve (41 × 41). Bounds are
handled by a cheap projected active-set method: re-solve 2–3 times with the violated bounds pinned. The extra terms:

- tie $n_0$ and its slope to the car's actual offset and heading, so the plan starts where the car is;
- pull $n$ toward the previous step's solution (shifted by the distance driven), so the published path does not jump
  between steps (the harness compares each path with the one published 0.5 s earlier);
- fade $n$ back to 0 over the last 4 m, so the car never commits to an apex it cannot yet see;
- keep the line 0.5 m inside the corridor, on top of the corridor's own safety margin.

**Why it matters.** It is the "racing line" half of predictive control, and it runs in real time on a single core,
which is the embedded budget the challenge sets (1 CPU core, 50 ms per step). Because it is re-solved from perception
every step, nothing is tuned to a known course, so it carries over to the unreleased evaluation courses.

**Evidence.**
- Figure: [`figures/practice_technical_bicycle_map.png`](../figures/practice_technical_bicycle_map.png): our line vs the
  centerline, coloured by speed (pre-PR #7).
- The solve itself takes about 50 µs. Measured over 8,950 control steps, the line and speed planner without
  verification takes 0.88 ms on average and 1.30 ms at p99 (pre-PR #7; the planner code has not changed since).
- Path-deviation category score: {{FINAL: 100 / 100}} on all 7 bicycle courses in the viz dataset, against 86.7–94.1
  for the baseline pure pursuit (`viz/data/index.json`).
- Keeping committed apexes (the warm-start weight 0.002 → 0.05) cut 1–3 s per course during development.

## 2. Physics-verified speed profile

**What.** A speed limit taken from curvature alone is optimistic: it ignores load transfer, the drivetrain and the
controller's own lag. So we check every plan with a physics model before we drive it, and slow down where the model
predicts the tyres would be overloaded.

First the planner builds the speed profile $v(s)$:

1. **Cornering cap:** $v_\text{cap}(s) = \sqrt{a_\text{lat}/|\kappa(s)|}$.
2. **Backward pass (braking):** $v(s)^2 \le v(s')^2 + 2a_\text{dec}(s' - s)$ for every later station $s'$.
3. **Forward pass (acceleration):** power-limited drive $F = \min(F_\text{drive}, P/v)$ minus drag and rolling resistance.

Both passes are exact O(K) prefix/suffix minimisations, not O(K²) all-pairs comparisons.

Then `predict.verify_plan()` rolls the car forward 1.5 s along the plan, with a **dynamic AWD single-track model** driven
by a mirror of the tracker's control law. The model has the same equations as the harness's bicycle backend:

- tyres that saturate at $\mu F_z$;
- a friction circle per axle;
- longitudinal load transfer;
- the 40/60 drive and 60/40 brake splits with traction control;
- power-limited drive force, drag and rolling resistance;
- the steering slew-rate limit and first-order steering lag.

The stiff tyre modes use a linearly implicit (Rosenbrock–Euler) update, so the model stays stable at the 50 ms control
step, where the harness needs 2.5 ms sub-steps.

Wherever the rollout predicts tyre use $|F|/(\mu F_z) > 0.95$, commanded grip demand $> 1.0$ or rear slip $> 5°$, it
returns a per-node speed factor. The planner caps $v$ there and redoes both passes. The caps are remembered by course
position, so a corner is usually found once and later steps verify in a single pass.

**Why it matters.** This is Model-Based Design inside the control loop. The RFP asks for "verified models and
simulations as the primary tools for design and validation", and for test plans that "identify operational limits and
describe how those limits are enforced". Our plant model checks every plan against the limits before the car acts on it.

**Evidence.**
- Figure: [`figures/practice_technical_bicycle_gg.png`](../figures/practice_technical_bicycle_gg.png): measured
  accelerations stay inside the friction circle (pre-PR #7).
- Model accuracy, replayed against 160 windows of 1.5 s of harness telemetry: yaw-rate RMS error 0.0103 rad/s, lateral
  velocity RMS 0.0125 m/s and 0.13 m position error at 1.5 s. A kinematic model gives 0.042 rad/s, 0.37 m/s and 0.54 m.
- In a unit test, one verification pass brought the predicted grip demand from 1.06 down to 0.93.
- Cost: one `verify_plan` call takes about 0.3 ms. The complete verified planner takes 4.23 ms on average and 7.61 ms at
  p99 (pre-PR #7, 8,950 steps).
- Dynamics (physical-realism) category score: {{FINAL: 100 / 100}} on 5 of 7 courses, {{FINAL: 96.2}} on
  practice_basic and {{FINAL: 98.2}} on seed 2 (`viz/data/index.json`).

## 3. Horizon-end visibility guard

**What.** The car sees only 40 m ahead, so the last visible metre could be the entry of a hairpin. The guard
assumes the worst: a corner of radius $R_\text{vis}$ = 8 m may start right after the end of the view. That caps the speed at the
horizon end at $v \le \sqrt{a_\text{lat} R_\text{vis}}$, and the backward pass carries that cap back to the car, so it
can always slow down in time for whatever appears next. Once the finish line is visible, only this speculative cap is
released; limits from corners the car can actually see stay in place.

**Why it matters.** This is a perception limitation handled by design. The RFP asks teams to address "perception
system limitations" and "fallback strategies". (Calling it a SOTIF-style performance limitation is our own framing; the
RFP cites ISO 26262, not SOTIF.)

**Evidence.**
- With the current defaults, the cap at the horizon end is $\sqrt{0.75 \cdot 0.95 \cdot 9.81 \cdot 8}$ ≈ {{FINAL: 7.5 m/s}}.
- {{FINAL: 0}} off-course events and {{FINAL: 0}} cone hits on three unseen `gen-track` courses (seeds 1–3, 3 obstacle
  groups each), whose corners the controller had never seen (`viz/data/index.json`).
- {{TODO: share of control steps where the visibility cap is the active speed limit}}

## 4. AWD per-axle friction model

**What.** The car is all-wheel drive, so both axles push and both brake, and each has its own grip budget.
The tracker computes the largest acceleration and braking the car can use *while cornering* without either axle leaving
its friction circle. Per axle, in steady state:

$$F_{z,f} = \frac{m(g\,l_r - a_x h)}{L},\qquad F_{z,r} = \frac{m(g\,l_f + a_x h)}{L}$$

$$F_{y,f} = \frac{m\,a_\text{lat}\,l_r}{L},\qquad F_{y,r} = \frac{m\,a_\text{lat}\,l_f}{L}$$

Drive force is split 40 % front / 60 % rear and brake force 60 % front / 40 % rear. Each candidate $a_x$ must satisfy
$F_x^2 + F_y^2 \le (\mu F_z)^2$ and the traction-control limit $|F_x| \le 0.97\,\mu F_z$ on both axles, with
$\mu$ = 0.85 × the vehicle's $\mu$ as a safety margin. All candidates are evaluated in one vectorised pass.
This captures effects a single friction ellipse misses: braking unloads the rear, which still carries 40 % of the brake
force, so the rear circle usually limits trail braking. Throttle mid-corner also loads the front tyres, so the car
gets power-on understeer.

**Why it matters.** It is the plant model of the real target vehicle (the GM-track 2026 Chevy Blazer EV), and it
enforces an operational limit in every step. The same split and load transfer are used in the verifier (`predict.py`)
and in the energy meter, so one consistent vehicle model runs through planning, control and evaluation.

**Evidence.**
- Figure: [`figures/practice_technical_bicycle_gg.png`](../figures/practice_technical_bicycle_gg.png) and the CARLA
  version [`figures/practice_technical_carla_gg.png`](../figures/practice_technical_carla_gg.png) (pre-PR #7).
- Dynamics category score {{FINAL: 100 / 100}} on practice_technical: no grip over-demand, no rear sliding past the
  scorer's 6° threshold, no steering-rate saturation and no throttle/brake overlap (`viz/data/index.json`).

## 5. Online understeer/slip adaptation

**What.** If the road is more slippery than the model assumes, the car should notice and slow down. Each step the
tracker compares the yaw rate the steering angle should produce, $r_\text{exp} = \frac{v}{L}\tan\delta$, with the
measured yaw rate, and low-pass filters the understeer indicator $1 - r_\text{meas}/r_\text{exp}$. It also computes the
rear-axle slip angle $\operatorname{atan2}(v_y - l_r r, v_x)$.

If filtered understeer exceeds 0.15 or rear slip exceeds 4°, the usable lateral grip scale drops by 0.01 per step (to
at least 0.75). Otherwise it recovers by 0.002 per step. The planner multiplies its cornering limit $a_\text{lat}$ by
this scale on the next step, so the whole speed plan backs off when the tyres start to slide.

**Why it matters.** It is online correction of the plant model: the controller re-identifies grip from its own sensors
instead of trusting a fixed $\mu$. In the RFP's terms it is a fallback strategy for when the model's assumptions stop
holding, and it matters for robustness on low-grip runs.

**Evidence.**
- Logged every step as `dbg_a_lat_max_est` / `dbg_slip` in `telemetry.csv`.
- {{TODO: lowest a_lat_scale reached in a lap, and lap result in the low-grip `batch` robustness run}}

## 6. Obstacle-aware corridor reshaping and the blocked-road safety stop

**What.** Obstacles are not on any map; they appear only in perception. `horizon.py` shrinks the road on each side by
half the car's width plus a 0.3 m margin. It then cuts the corridor around each obstacle's lateral extent and keeps the
wider free side. A hysteresis of 0.3 m means that when the two gaps are nearly equal, the car stays on the side it is
already on. The cut ramps in and out over 20 m, so the racing line shifts smoothly instead of jinking.

If no gap is wide enough for the car, the corridor is marked invalid. The planner then switches to a
**blocked plan**: the centerline, with a speed profile that brakes at 0.8 × max deceleration to a 2 m/s creep at a point
1 m before the car's nose reaches the blockage, and never exceeds the current speed. Creeping instead of stopping
means a transient false detection cannot trigger the harness's 8 s "stuck" DNF. Before this fix, an invalid corridor
produced a normal, accelerating plan: full throttle at the obstacle.

**Why it matters.** This is safety by design, with a defined degraded mode. The RFP asks for "fallback strategies" and
for "operational limits" that are enforced. The fallback is tiered: a blocked road gives a controlled stop, and a
module error or a budget overrun drops the car to a conservative centerline plan (see `controller.py`).

**Evidence.**
- practice_obstacles: {{FINAL: 0}} cone hits vs 3 for the baseline; unseen seeds 1–3 with 3 obstacle groups each:
  {{FINAL: 0}} cone hits vs 3–4 for the baseline (`viz/data/index.json`).
- `tests/test_blocked.py`: the final command slows the car in both modes; a closed-loop run slows down before the
  blockage; a control case with a gap drives through.
- The blocked plan never triggered on any public course (0 of about 11,000 steps, both modes), so it costs no lap time.
- Known limit: on two further unseen seeds (4 and 5), the pre-PR #7 controller hit one obstacle cone each.
  {{TODO: re-run seeds 4 and 5 on the locked main}}

## 7. Two driving modes on one knob: PERFORMANCE vs ECO

**What.** The same controller drives either the fastest legal lap (PERFORMANCE, the scored default) or a lower-energy
lap (ECO). One parameter, `eco_weight` ∈ [0, 1], selects the trade-off, chosen only by a config file, never by course
or position. ECO changes only the speed profile, and every ECO plan still goes through the physics verifier:

- **Lift-and-coast:** before each braking zone the profile follows the car's own coast-down (drag + rolling, about
  0.2 m/s²) and brakes only where coasting can no longer make the corner. The coast length is limited to what fits in
  the visible 40 m.
- **Gentler acceleration** (forward pass × (1 − 0.5w)) and a **lower top speed** (× (1 − 0.35w)), because drag
  power grows with $v^3$.

To measure the result we wrote an **energy meter** (`tools/energy.py`) whose force model mirrors the simulator's bicycle
backend: power-limited drive force, brake force, the 40/60 and 60/40 splits, and the per-axle traction clip with load
transfer. A plain longitudinal balance (traction − brake − drag − rolling − ΔKE) leaves 7–13 % unexplained. That
remainder is cornering tyre scrub, which the meter rebuilds from the lateral tyre forces; with it, **the energy balance
closes**. Battery energy is

$$E_\text{batt} = \frac{E_\text{trac}}{\eta_\text{drive}} - \eta_\text{regen}\int \min(P_\text{brake}, P_\text{regen,max})\,dt$$

with **assumed** $\eta_\text{drive}$ = 0.90, $\eta_\text{regen}$ = 0.65 and $P_\text{regen,max}$ = 60 kW (generic EV
values, **not** Blazer EV specifications; the simulator has no battery or regen).

**Why it matters.** The GM track lists "energy efficiency and safety as top priorities", and the RFP mentions
"energy-aware" systems and "drive quality". ECO shows the controller can make that trade explicitly and measure it,
without changing its safety limits.

**Evidence.**
- Figures: [`tools/figures/eco_pareto.png`](../tools/figures/eco_pareto.png) (lap time vs energy per course, the
  recommended point marked, the baseline in grey), [`tools/figures/eco_energy_breakdown.png`](../tools/figures/eco_energy_breakdown.png)
  (where the energy goes) and [`tools/figures/eco_speed_trace.png`](../tools/figures/eco_speed_trace.png) (coasting
  stretches shaded). All three are pre-PR #7.
- Current controller, `eco_weight` 0.8 vs PERFORMANCE, mean over 7 courses (4 public + 3 unseen):
  **{{FINAL: −27.1 %}} energy (Wh/km) for {{FINAL: +7.1 %}} lap time**, with 0 cones, 0 off-course, 0 missed gates and 0
  controller errors on every lap. Per course: energy {{FINAL: −12.9 % to −37.1 %}}, lap time {{FINAL: +3.8 % to +10.5 %}}.
  practice_technical: {{FINAL: 36.73 s / 717 Wh/km}} → {{FINAL: 40.59 s / 489 Wh/km}} (`viz/data/index.json`).
- Pre-PR #7 Pareto sweep (`tools/figures/eco_pareto.md`, 5 courses): `eco_weight` 0.2 / 0.4 / 0.6 / 0.8 gave −3.5 % /
  −11.8 % / −23.0 % / −35.8 % energy for −0.0 % / +0.9 % / +2.8 % / +6.2 % lap time. `eco_weight` 1.0 hit one cone, so
  0.8 is the recommended point.
- Energy balance: the largest residual across all 63 laps in the viz dataset is 0.08 %.
- Limit: lift-and-coast alone saves little, because the 40 m view leaves little room to coast (about 2 s per lap on
  practice_technical). Most of the saving comes from the lower top speed and gentler acceleration. ECO has not been
  run on CARLA.

## 8. Sensor-fusion tool (not part of the scored controller)

**What.** A standalone, CPU-only lidar + radar + camera fusion pipeline in pure numpy (`fusion/`), built as the
perception front end such a controller would need on a real car. Per frame it:

1. filters lidar points by height;
2. groups them into objects with connected components on a 2-D grid;
3. reads each object's colour from the camera image patch it projects to;
4. attaches radar returns by nearest neighbour.

A cluster without a reliable colour stays UNKNOWN rather than being reported as an obstacle. `fusion/bridge.py`
converts the result into the harness's own `ConeDetections` and obstacle list, so the fused output can feed our planner
in an offline demo. `fusion/animate.py` renders a side-by-side GIF. `fusion/carla_capture.py` records synchronised
sensor frames from a running CARLA world without ticking it.

**This tool is not loaded by the scored controller.** The sandboxed controller receives a processed `RoadModel`, and
raw sensors are not available to it.

**Why it matters.** The RFP expects teams to build their own "perception (object detection and classification), sensor
fusion, path planning, and vehicle controls", and mentions "energy-aware perception stacks". UA's own published
precedent is object-level camera + radar fusion with Kalman filters and a GNN tracker. For context, ETH's AMZ team
dropped their camera module in 2021 because its latency and compute outweighed the gain. Ours is cheap enough to keep.

**Evidence** (`python -m fusion.demo --frames 120`, synthetic frames, fusion CPU time only, measured 17 Oct 2026 on a
team laptop):
- Latency: **1.68 ms median**, 2.62 ms p95 and 13.30 ms max per frame. The max is the first frame, where clustering
  takes 12.09 ms during warm-up.
- 1,331 of 1,453 detections (92 %) have the correct colour within 0.5 m; median position error 0.015 m.
- Synthetic results do not establish real CARLA precision or recall, and the CARLA capture path has not been run yet.
- {{TODO: fusion composite GIF path (fusion/animate.py output)}}

---

## Built the way EcoCAR builds: model, verify, deploy

| Our feature | Sponsor term (from SPONSOR_RESEARCH.md) |
|---|---|
| Friction model, steering-rate guard, grip cap | "operational limits" that are enforced |
| Visibility guard | "perception system limitations" handled by design |
| Tiered fallback (blocked stop, centerline plan, pure pursuit) | "fallback strategies" |
| `predict.py` rollout on the harness's own vehicle equations | "verified models and simulations"; "predictive vehicle control" |
| Bicycle backend, then CARLA | model-in-the-loop on a fitted plant, cross-checked in higher-fidelity simulation |
| `selftest`, robustness `batch`, pytest on synthetic road models | "requirement-based testing" |
| Separate modules with fixed data contracts | "decoupled subsystem development" |
| ECO mode + energy meter | "energy efficiency and safety as top priorities" |
