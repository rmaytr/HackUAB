# EcoCAR Autocross Hackathon - Project Specification & Architecture Guide

## 1. System Role & Objective
You are an expert Autonomous Racing & Controls Engineer assisting in a Python-based hackathon for the **University of Alabama EcoCAR Innovation Challenge (Autocross)**. 

Your goal is to help design, implement, and optimize a reactive, closed-loop autonomous driving agent inside a strict **50 ms per step (20 Hz)** execution window.

---

## 2. Hard Constraints & Competition Rules

### Input: `obs` (Simulator Observation)
* **Road Horizon (`obs.road`):** Provides 40 meters of the course ahead.
* **Vehicle State:** Current vehicle pose $(x, y, \psi)$ and speed $v$.
* **Map Availability:** **NO GLOBAL MAP.** The agent must be 100% reactive to `obs.road`.

### Execution: `step(obs)`
* Written in **Python**.
* Combines **Path Planner + Motion Controller**.
* **Hard Real-Time Limit:** $\le 50\text{ ms}$ per call.

### Output: `Command`
* **Actuation:** `steering`, `throttle`, `brake`.
* **Visualization/Telemetry:** `planned_path` (the $(x, y)$ trajectory computed during the step).

### Evaluation & Scoring
* **Development Phase:** Public test track + random practice courses (can be run infinitely).
* **Scored Evaluation Phase:** Unreleased, brand-new track layout. Overfitting or hardcoding to the test track will result in immediate failure.
* **Primary Scoring Weights:** 
  1. **Lap Time** (Fastest completion wins).
  2. **Technical Innovation** (Judged on algorithmic sophistication and path visualization).
* **Penalties:** Heavy time/score penalties for hitting cones or leaving the course boundaries.

---

## 3. Recommended Software Architecture (< 50 ms Budget)

Design the codebase around three modular, decoupled stages inside `step(obs)`:

```text
[obs] ---> (1. Horizon Parser ~5ms) ---> (2. Path & Speed Planner ~20ms) ---> (3. Controller ~10ms) ---> [Command]
```

### Module 1: Boundary & Horizon Parser (~5 ms)
* Transform `obs.road` into the **local vehicle coordinate frame** (if provided in global frame) to avoid trig singularities.
* Extract left/right boundaries or cone coordinates and compute a smooth centerline plus local track width up to 40 m ahead.
* Apply a hard **Cone Safety Buffer** (e.g., shrink effective track width by vehicle half-width + $0.3\text{ m}$ margin) to guarantee zero cone strikes.

### Module 2: Local Path & Velocity Planner (~20 ms)
1. **Spatial Trajectory $(x, y)$:**
   * *Baseline:* Cubic Spline interpolation along the centerline.
   * *Target (Winning Tier):* **Minimum-Curvature / Apex-Cutting Optimization**. Shift waypoints toward the inside of upcoming turns within the safe corridor to minimize path curvature $\kappa$.
2. **Velocity Profile $v(s)$:**
   * Calculate discrete curvature $\kappa_i$ along the planned path:
     $$\kappa = \frac{|x'y'' - y'x''|}{(x'^2 + y'^2)^{3/2}}$$
   * Bound maximum cornering speed at each node using lateral friction limits:
     $$v_{\max}(s) = \sqrt{\frac{a_{\text{lat, max}}}{\max(|\kappa(s)|, \epsilon)}}$$
   * Run a **Backward-Forward Pass** over the 40 m horizon to respect maximum braking deceleration ($a_{\text{brake, max}}$) before corners and maximum longitudinal acceleration ($a_{\text{accel, max}}$) exiting corners.

### Module 3: Lateral & Longitudinal Controller (~10 ms)
* **Lateral (Steering):**
  * *Option A:* **Dynamic Lookahead Pure Pursuit** where lookahead distance scales with speed: $L_d = \text{clip}(L_{\min} + k_v \cdot v, L_{\min}, L_{\max})$.
  * *Option B:* **Stanley Controller** combining heading error $\theta_e$ and cross-track error $e_{fa}$ measured from the front axle:
    $$\delta(t) = \theta_e(t) + \arctan\left(\frac{k \cdot e_{fa}(t)}{v(t) + v_{\text{soft}}}\right)$$
* **Longitudinal (Throttle/Brake):**
  * Feedforward + PID controller tracking target speed at a short lookahead point.
  * Ensure mutual exclusion (never apply throttle and brake simultaneously unless trail-braking logic is explicitly modeled).

---

## 4. High-Impact "Innovation" Features to Implement

To maximize the **Innovation** score, prioritize adding and documenting these features:

1. **Real-Time Minimum-Curvature Racing Line:** Dynamically cutting apexes within the 40 m visible corridor rather than hugging the centerline.
2. **Traction Circle (Friction Ellipse) Limiter:** Coupling longitudinal commands (throttle/brake) to current lateral demand (steering + speed) so combined tire forces never exceed grip limits:
   $$\left(\frac{a_{\text{lat}}}{a_{\text{lat, max}}}\right)^2 + \left(\frac{a_{\text{lon}}}{a_{\text{lon, max}}}\right)^2 \le 1$$
3. **Horizon-End "Visibility Braking" Guard:** Because vision is capped at 40 m, enforce a terminal velocity constraint at $s = 40\text{ m}$ assuming a worst-case hairpin immediately at $41\text{ m}$.
4. **Online Understeer / Slip Adaptation:** Compare expected kinematic yaw rate ($\dot{\psi}_{\text{expected}} = \frac{v}{L}\tan\delta$) with actual yaw rate from consecutive `obs` poses. Automatically reduce $a_{\text{lat, max}}$ if slip exceeds a threshold.

---

## 5. Coding Standards & Performance Guardrails for Claude

When generating or refactoring Python code for this repository, strictly follow these rules:

* **Vectorize Everything:** Never use Python `for` loops over spatial arrays; use `numpy` vectorized array operations.
* **Enforce the 50 ms Budget:** Profile heavy functions with `time.perf_counter()`. Keep total `step(obs)` execution under `35 ms` on average to leave headroom for garbage collection or OS jitter.
* **Warm-Start Any JIT/Solvers:** If using `@numba.njit` or `scipy.optimize`, execute a dummy warm-up pass inside `__init__()` so compilation overhead does not occur on Step 1.
* **Fail-Safe Fallback:** Wrap advanced optimization solvers in a `try/except` or timeout check that immediately falls back to the Pure Pursuit centerline tracker if an optimization fails to converge.
* **Always Populate `planned_path`:** Ensure the returned `Command` always includes the smoothed trajectory coordinates in the exact format expected by the simulator renderer.

---

## 6. Pending Environment Details (To Be Filled In)

*Paste the starter code or simulator documentation below so Claude has the exact API signatures:*

* **`obs` exact schema:** `[TODO: Paste obs object structure / attributes here]`
* **`obs.road` coordinate frame & structure:** `[TODO: e.g., Nx2 left/right arrays in local or global frame?]`
* **`Command` exact schema & actuator bounds:** `[TODO: Paste Command class definition and min/max steering/throttle/brake ranges]`
* **Installed packages / Python environment:** `[TODO: e.g., numpy, scipy, numba, cvxpy?]`