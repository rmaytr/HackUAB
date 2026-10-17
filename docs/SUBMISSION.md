# SUBMISSION: what we hand in, who makes it, and by when

Written 4:07 PM. **Code freeze 5:00 PM.** Everything below is ordered by deadline.
Source of the rules: `simulator/README.md` ("Submitting", "Scoring", "Official evaluation") and
`simulator/config/scoring.yaml`. Upstream has no newer instructions (checked: pinned `06db384` = upstream).

## How we're scored (what each deliverable is for)

| Points | What earns them | Our deliverable that wins them |
|---|---|---|
| **75**: technical, computed by the judges' script | `hackuab/` on 2 **unreleased** courses × 2 backends (CARLA 50 %, bicycle 50 %), technical course 60 % | **D1: the code submission** |
| **25**: innovation, judged ("live pitch / Spirit of Innovation", algorithmic sophistication + **path visualization**) | What the judges see and understand | **D2: video**, **D3: README + INNOVATION.md**, **D4: figures**, **D5: visualization app** |

The technical points are decided by the code alone. The video and docs can only win the 25 innovation points,
so they must make each innovation **visible and backed by a measured number**.

## ⚠️ Ask the organisers NOW (Rodion, 2 minutes)

1. **Where and how do we submit?** A form, Devpost, an email, or the GitHub link? Is there a separate deadline for the video?
2. **Team folder name:** the README says `teams/<team>/controller.py`. Is `hackuab` OK as the team name?
3. **Video format:** max length (we're planning 4:00), and is an unlisted YouTube or Google Drive link fine?
4. Is the **live pitch** separate from the video, or is the video the pitch?

---

## D1: Code submission (scored) · owner **Viraj** · locked by **4:40**, tagged **4:55**

**What:** the folder `hackuab/` (controller.py + horizon.py, planner.py, predict.py, tracker.py, params.py,
performance.yaml, eco.yaml). The judges copy it to `teams/hackuab/` and run it with **no config**, which means PERFORMANCE mode.

**Steps:**
1. **4:15–4:35:** merge Viraj's best branch (the 71+/75 work) into `main`, **only** if it is clean (0 cones/off/gates/errors)
   on all public courses AND on `gen-track` seeds 1–5 with obstacles (unseen-course check), on **both** backends.
   If it isn't clean on CARLA, keep current `main`. A clean 68 beats a 71 that clips cones on an unseen course.
2. **4:35:** final checks on the locked `main` (from `simulator/`):
   ```powershell
   .venv\Scripts\python.exe -m pytest ..\tests -q
   .venv\Scripts\python.exe -m autocross selftest -c ..\hackuab\controller.py
   .venv\Scripts\python.exe -m autocross selftest -c ..\hackuab\controller.py --carla
   ```
   Required: every course finished, 0 sandbox violations, 0 controller errors. Save both selftest tables (screenshot + the
   `runs/<stamp>_selftest` folder). They go into the README and the video.
3. **4:55:** `git tag submission && git push origin submission`. Zip `hackuab/` as `hackuab.zip` if the organisers want a file.
4. After the tag: **no more commits to `hackuab/`**, only docs and media.

## D2: Video, 3–5 min (target 4:00) · owner **Rodion** (narration + edit), footage from everyone

**Record everything by 4:45, edit 4:45–5:15** (or by the organisers' video deadline).
Tools: **OBS Studio** or the Windows Game Bar (`Win + Alt + R`) for screen capture with mic; **Clipchamp** (built into
Windows) to cut. Export MP4 at 1080p. Record each clip separately. Never demo live without a recorded backup.

| Time | Segment | What's on screen | Footage owner |
|---|---|---|---|
| 0:00–0:20 | **Hook** | CARLA chase-cam of our car through the esses; title "HackUAB: physics-verified autonomous autocross" | Viraj |
| 0:20–0:45 | **Problem** | No map, 40 m of sight, 50 ms per decision, unseen courses. The baseline car on the same course (clips cones) | Viraj (baseline CARLA clip) |
| 0:45–1:45 | **How it works** | The 4-stage pipeline (Horizon → Planner → Predict → Tracker) with the visualization app's Pipeline tab stepping through one control cycle | Rodion (Base44 app) |
| 1:45–2:30 | **Innovation 1–3** | Racing line overlaid on the centerline (map figure), the physics verifier predicting grip use, the g-g diagram inside the friction circle, obstacle avoidance on practice_obstacles | Rodion + figures |
| 2:30–3:10 | **Two modes** | PERFORMANCE vs ECO side by side in the app; the Pareto plot; the energy-breakdown bar; the headline "−X % energy for +Y % lap time, 0 cones" | Rodion |
| 3:10–3:35 | **Sensor fusion** | The fusion composite animation (lidar + radar + camera → cones that feed our planner), plus ms per frame | Saisi |
| 3:35–3:55 | **Results** | Table: baseline vs ours (lap, cones, p99 ms, technical points) on both backends + robustness sweep. The selftest terminal screenshot | Viraj |
| 3:55–4:00 | **Close** | "It generalises because it's physics, not tuning." Team names + repo link | Rodion |

**CARLA footage (Viraj, 4:15–4:40):** restart CARLA windowed with `-Quality Epic`, then record:
`..\tools\run_mode.py performance run practice_technical --backend carla` (our car, cyan planned path visible),
the baseline on the same course, and one ECO run. Also a `--view` bicycle run on practice_obstacles (obstacle avoidance).

**Rules for every number in the video:** it comes from a run on the **final locked `main`**. No pre-PR #7 numbers.

## D3: README.md + docs/INNOVATION.md · owner **Rodion** (draft now, numbers at 4:45)

`README.md` is still the setup stub, and judges open it first. Replace it with:
1. One-sentence pitch + a hero image (CARLA screenshot or the racing-line map).
2. **Results table:** baseline vs PERFORMANCE vs ECO, per public course, both backends (lap time, cones, technical /75, p99 ms).
3. **Architecture:** the 4-stage pipeline diagram + one line per module.
4. **Innovation list** (link to INNOVATION.md): min-curvature racing line re-solved every 50 ms; physics-verified speed
   profile (dynamic AWD bicycle model rollout); horizon-end visibility guard; AWD per-axle friction model; online slip
   adaptation; obstacle-aware corridor + blocked-road safety stop; ECO mode with an energy meter and a Pareto front; sensor fusion.
5. **How to run:** setup, `run_mode.py performance|eco`, `pytest -m performance|eco`, selftest.
6. **Honest notes:** the energy assumptions (eta_drive 0.9, eta_regen 0.65, P_regen 60 kW; *not* Blazer EV specs), what's untested.
7. Team: Rodion, Saisi, Viraj.

`docs/INNOVATION.md`: one section per innovation: **what** it is, **why** it matters (sponsor language from
docs/SPONSOR_RESEARCH.md: safety, energy, real-time embedded, model-based design), and **evidence** (a figure + a number).

## D4: Figures · owner **Viraj** (regenerate on the locked `main`, 4:40–4:50)

All current PNGs predate PR #7. Regenerate from final runs (from `simulator/`):

| Figure | Command | Used in |
|---|---|---|
| Course map: racing line vs centerline, speed-coloured, ours vs baseline | `..\tools\plot_run.py OURS_RUN BASELINE_RUN --out ..\figures` | README, video 1:45 |
| Speed trace, g-g diagram with friction circle, step-time histogram | (same `plot_run.py` call) | README, video 1:45 / 3:35 |
| Lap times per course, ours vs baseline | `..\tools\plot_laps.py OURS_SELFTEST BASELINE_SELFTEST --out ..\figures` | README, video 3:35 |
| ECO Pareto + energy breakdown + speed traces | `..\tools\eco_pareto.py` then `..\tools\eco_figures.py --results …` | README, video 2:30 |
| Fusion composite GIF + ms per stage | `fusion/demo.py` (Saisi) | video 3:10 |

Do the bicycle and CARLA versions of the map and g-g plots (`figures/practice_technical_carla_*` already shows the format).

## D5: Visualization app (Base44) · owner **Rodion**

- It reads `viz/data/` from GitHub. That data is on branch `rodion/viz-data` and was generated **before PR #7**, so it's out of date.
  At 4:40, once `main` is locked: run `tools/export_viz.py` on `main`, commit `viz/data/`, merge into `main`, and point the app's
  base URL at `.../HackUAB/main/viz/data/`.
- Record the app walkthrough for video segments 0:45 and 2:30. Put the app link in the README.

## D6: Sensor fusion (not scored, innovation story) · owner **Saisi**

- PR #8 (`saisi/fusion-starter`) is open. Merge it **only** if it doesn't touch `hackuab/` (it shouldn't) and its demo runs.
- Deliver a 20–30 s clip or GIF + one number (ms per frame) for the video, plus a paragraph for INNOVATION.md.

---

## Timeline (all three of you)

| Time | Rodion | Viraj | Saisi |
|---|---|---|---|
| 4:07–4:15 | Ask the organisers (above); start the README/INNOVATION draft | Pick the final branch; start the clean checks on both backends | Finish the fusion demo; review PR #8 |
| 4:15–4:40 | README + INNOVATION draft; record the Base44 app clips | Lock `main`; record CARLA footage | Record the fusion clip; merge PR #8 |
| 4:40–4:55 | Re-export viz data; fill the real numbers into README | Final selftests (both backends), regenerate figures | Help Rodion with INNOVATION.md |
| **4:55** | | **Tag `submission`** | |
| 4:55–5:15 | Edit + export the video, upload, submit the link | Screenshot the final results | Proof-watch the video |

## Final checklist

- [ ] Organisers' answers on where/how/deadline
- [ ] `main` locked: tests pass, selftest bicycle + CARLA clean, 0 sandbox violations
- [ ] `submission` tag pushed (and `hackuab.zip` if needed)
- [ ] README.md replaced (pitch, results, architecture, innovations, how to run, team)
- [ ] docs/INNOVATION.md with evidence per innovation
- [ ] Figures regenerated on the locked `main`
- [ ] viz/data refreshed on `main`; app link in the README
- [ ] Video 3–5 min recorded, edited, uploaded; link submitted
- [ ] Every number in the video/README comes from the locked `main`
