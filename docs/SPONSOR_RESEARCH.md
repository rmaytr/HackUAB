# Sponsor research: EcoCAR autocross hackathon

Researched 2026-10-17. This is a synthesis of desk research. Items marked **(unverified)** come from search snippets, unreachable pages or inference. The proposals in the last sections are proposals only: `docs/ROADMAP.md` has not been edited.

> **Naming warning.** The **EcoCAR Innovation Challenge (EIC)** is the new competition (AVTC15, 2026–2030). Its GM track uses a **2026 Chevy Blazer EV**. The **EcoCAR EV Challenge** (AVTC14, 2022–2026, **Cadillac LYRIQ**) is the earlier competition, and it finished in May 2026. The EIC has not held any competition events yet. Every EIC "event" below comes from its request for proposals (RFP), and the RFP marks them illustrative.

## Executive summary

- **The person really "buying" here is the UA EcoCAR CAV team.** The harness was written by Daniel Szurek, UA EcoCAR's connected and automated vehicles (CAV) lead, with help from a second contributor. UA is on the EIC GM track, which has a planned year-4 autonomous autocross. In effect our hackathon is a prototype of a real event they will have to compete in.
- **Quote the RFP back to them.** The EIC RFP lists **"Predictive vehicle control for an Autocross course"** as an innovation opportunity, and it names **"Autono-cross"** as a CAV evaluation. Our physics-verified planner fits that line almost word for word. Make it the title of the pitch.
- **Speak Model-Based Design and safety.** The RFP's language is "verified models and simulations", "requirement-based testing", MiL/HiL/ViL, ISO 26262/FMEA/HARA, and test plans that "identify operational limits and describe how those limits are enforced" with "fallback strategies". Our friction ellipse, visibility guard, predictive verifier and centerline fallback are exactly those things, so we should name them that way.
- **Energy and drive quality count, not just lap time.** The GM track lists "energy efficiency and safety as top priorities". Most CAV features are "evaluated for energy consumption impacts", and the 2026 winner was praised for "smooth, intuitive operation". A cheap energy proxy and a jerk/smoothness metric would let us use their vocabulary honestly.
- **Fusion should be object-level, fast and visibly calibrated.** UA's own published precedent is camera+radar Kalman/GNN fusion in MATLAB. One strong hook is that the ETH AMZ team dropped their camera module because of its latency. A numpy pipeline that runs in a few ms, with lidar painted on the image and a latency waterfall, is the convincing demo.

## Q1. Organisers, headline sponsors and the UA team

| Role | Who | Source |
|---|---|---|
| Launch and management | US DOE + **Argonne National Laboratory** (AVTC series since 1988, DOE's "flagship workforce development program") | [EIC fact sheet](https://avtcseries.org/wp-content/uploads/2026/04/EcoCAR-Innovation-Challenge-Fact-Sheet.pdf) |
| Track sponsors | **GM** (Blazer EV: propulsion, automated driving, V2X) and **Stellantis** (2026 Jeep Cherokee hybrid) | [AVTC launch release](https://avtcseries.org/news-media/energy-department-and-argonne-join-gm-stellantis-and-mathworks-to-launch-ecocar-innovation-challenge/) |
| Technology partner | **MathWorks** (Model-Based Design, MATLAB/Simulink/Simscape licences) | same |
| Additional sponsors | Caterpillar, Siemens Digital Industries Software, dSPACE, Bosch, Altec, Hesse, Volta Foundation. No sponsor tiers are published. | same |

The EIC was unveiled with its RFP on 30 Sep 2025 ([AVTC](https://avtcseries.org/news-media/ecocar-innovation-challenge-unveiled/)) and launched on 13 Apr 2026 with 20 universities, 10 per track. **The University of Alabama is on the GM track** ([launch release](https://avtcseries.org/news-media/energy-department-and-argonne-join-gm-stellantis-and-mathworks-to-launch-ecocar-innovation-challenge/)). The stated mission is workforce development: "develop the next generation of industry-ready leaders through hands-on, interdisciplinary experiences". The 2026 messaging leans on AI, ML and quantum computing, plus "concept to potential commercialization" ([fact sheet](https://avtcseries.org/wp-content/uploads/2026/04/EcoCAR-Innovation-Challenge-Fact-Sheet.pdf)). In the launch release, GM says EcoCAR builds engineers who "integrate software, controls, advanced powertrains", and MathWorks stresses "the same Model-Based Design" used in industry.

**UA EcoCAR** results in the EV Challenge:

| Year | Overall placing |
|---|---|
| Y1 | 3rd |
| Y2 | 4th |
| Y3 | 3rd, with 683/1000 ([Y3 recap](https://avtcseries.org/news-media/ecocar-year-3-competition-recap-engineering-excellence-in-action/)) |
| Y4 (final) | **4th overall**, with **1st in communications, CAV, and system design & integration** ([UA Engineering](https://news.eng.ua.edu/?p=11538)) |

The Y4 podium was Georgia Tech (867/1000), McMaster and Virginia Tech ([2026 winners release](https://avtcseries.org/wp-content/uploads/2026/05/EcoCAR-Challenge-Winners-Press-Release-2026.pdf)).

UA has also won several sponsor awards ([EV Challenge page](https://avtcseries.org/about-the-ecocar-ev-challenge/year-two)):

| Award | UA result |
|---|---|
| MathWorks Model-Based Design Award | 1st in Y3, 2nd in Y4 |
| Argonne AI Innovation Award | 1st in Y3 |
| CAV Systems Presentation | 1st in Y2 |

Faculty advisors are Dr. Paul Puzinauskas (lead) and Dr. Hwan-Sik Yoon (CAV lead advisor, NSF Outstanding Faculty Advisor) ([AVTC](https://avtcseries.org/news-media/nsf-outstanding-faculty-advisor-award/)).

*Conflicts in the sources:*

- The UA article calls the competition "2023–2026", while AVTC dates it 2022–2026.
- The AVTC UA team page lists a Y4 award titled "CAV Vehicle Technical Specifications" ([AVTC UA page](https://avtcseries.org/about-the-ecocar-ev-challenge/competing-teams/university-of-alabama/)). The UA article reports a broader "1st in CAV". These may be the same award described differently **(unverified)**.
- One research note marked UA's EIC track as unverified. The AVTC launch release lists UA on the GM track, and the UA article confirms the Blazer EV, so the release settles it.

## Q2. The vehicle and the autonomy stack

**Vehicle.** AVTC calls the EIC GM-track vehicle the **2026 Chevy Blazer EV**. Some universities call it the "Blazer SS" ([WMU](https://wmich.edu/engineer/news/2026/06/ecocar-innovation-challenge-gm)) or "Blazer Super Sport" ([Penn State](https://www.psu.edu/news/engineering/story/penn-state-advanced-vehicle-team-accepted-ecocar-innovation-challenge)). Szurek's GitHub says "Blazer EV SS" ([GitHub](https://github.com/dszurek)). This is a naming variant, not a real conflict.

**Required autonomy.** The RFP targets **SAE Level 4** "with a safety driver present in limited closed-course conditions". Teams "will develop their own perception stack" and their own "perception (object detection and classification), sensor fusion, path planning, and vehicle controls". The RFP also says teams must "heavily utilize simulation environments to develop and validate code prior to on-vehicle deployment" ([RFP Rev B](https://avtcseries.org/wp-content/uploads/2025/10/EcoCAR-Innovation-Challenge_RFP_Rev-B-Oct-22.pdf)).

The planned CAV progression by year:

| Year | Planned CAV milestone |
|---|---|
| Y1 | "Simulation established for motion planning & lateral/longitudinal control" |
| Y2 | Basic-ODD motion control in the vehicle |
| Y3 | Expanded ODD |
| Y4 | Full ODD |

Our hackathon is Y1 work.

**Sensors and compute.** **No public source names the EIC sensor suite or compute.** For the LYRIQ era, these suppliers are confirmed:

| Supplier | What they supplied | Source |
|---|---|---|
| dSPACE | AUTERA compute, RTMaps/ASM/AURELION/VEOS | [engineering.com](https://www.engineering.com/ecocar-ev-the-cadillac-of-design-competitions/) |
| NXP | NavQPlus edge board | [AVTC](https://avtcseries.org/news-media/nxp-sponsors-ecocar-ev-challenge-empowering-future-automotive-innovators/) |
| Infineon | AURIX microcontrollers, ToF and radar for in-cabin sensing | [AVTC](https://avtcseries.org/news-media/ecocar-ev-challenge-contributor-sponsors/) |
| Cohda Wireless | C-V2X radios | same |
| Vector | CAN tools | same |

The engineering.com article describes the perception sensors only as "lidar, radar, 3D cameras". It is **(unverified)** whether this hardware carries over to the EIC.

**Public UA perception work.** In a 2019 MathWorks blog post, UA described its Mobility Challenge setup: a Mobileye 6 camera, Bosch mid-range radars (front plus four corners), and **Kalman filters with a Global Nearest Neighbor tracker** in MATLAB/Simulink, logged with Vector CANoe ([MathWorks Student Lounge](https://blogs.mathworks.com/student-lounge/2019/07/24/perception-level-2-automation/); [File Exchange 72003](https://www.mathworks.com/matlabcentral/fileexchange/72003-automated-vehicle-perception-system-and-sensor-fusion)). UA presented at IEEE ITEC in 2026, but the paper's title is not known. No public UA material on LYRIQ-era perception or controls was found.

## Q3. Real CAV and autocross events, scoring and failure modes

**The EIC autocross exists only on paper.** RFP Table 8, which the RFP marks illustrative, lists these sample events ([RFP Rev B](https://avtcseries.org/wp-content/uploads/2025/10/EcoCAR-Innovation-Challenge_RFP_Rev-B-Oct-22.pdf)):

- **"Autono-cross"** under CAV System Evaluation.
- "Autocross, acceleration, and handling" under Vehicle Performance.
- **"Autocross: Culminating year 4 event for a human-driver run versus an autonomous run"**.

Penn State describes the autocross as precision-based, with cars navigating traffic obstacles ([Penn State](https://www.psu.edu/news/engineering/story/penn-state-advanced-vehicle-team-accepted-ecocar-innovation-challenge)). **No rules, point values or scoring formula are public.** The RFP says confirmed features and events will be released to selected teams later.

**What completed EcoCAR events rewarded.** EV Challenge years were scored out of 1,000 points. Its CAV dynamic events were:

- Connected Intersection Navigation
- ACC Feature Evaluation (sponsored by dSPACE)
- the Argonne-built **Connected Driving Evaluation (CoDE)**
- lateral automation: lane centering and automatic parking ([2026 winners release](https://avtcseries.org/wp-content/uploads/2026/05/EcoCAR-Challenge-Winners-Press-Release-2026.pdf))

Argonne validates these features with repeatable test systems. **LARS** replays speed traces, and **FlexCorridor** generates SAE J2735 SPaT/MAP messages in real time ([AVTC](https://avtcseries.org/news-media/ecocar-teams-put-connected-and-automated-vehicle-features-to-the-test/)). ACC was also scored for **energy consumption** on a chassis dynamometer ([AVTC CARB article](https://avtcseries.org/news-media/ecocar-teams-put-their-cav-technologies-to-the-test-at-year-3-carb-dynamometer-event/), from a search summary). A panel of industry judges gave the Consumer Acceptability Award for driving performance alongside CAV features.

**Failure modes.** No public reports of CAV disqualifications or failures were found. The structural gate is that vehicles must "pass rigorous safety inspections" before they can take part in any testing ([RFP Rev B](https://avtcseries.org/wp-content/uploads/2025/10/EcoCAR-Innovation-Challenge_RFP_Rev-B-Oct-22.pdf)). Our hackathon's DNF rules play the same role: if you are unsafe, you score nothing.

**Inference:** real EcoCAR judging rewards three things together: energy impact, repeatable test-driven validation, and smoothness as the consumer perceives it. Raw time is only one of them.

## Q4. What "technical innovation" means to these sponsors

| Theme | Sponsor wording (verbatim) | Source |
|---|---|---|
| Model-Based Design | "modeling, simulation, and requirement-based testing"; "Reliance on verified models and simulations as the primary tools for design and validation"; "Decoupled subsystem development" | [RFP Rev B](https://avtcseries.org/wp-content/uploads/2025/10/EcoCAR-Innovation-Challenge_RFP_Rev-B-Oct-22.pdf) |
| X-in-the-loop | "MiL/HiL/ViL: model/hardware/vehicle in the loop" | same |
| Safety | FMEA, HARA, "portions of ISO 26262", E-Stop; test plans "must identify operational limits and describe how those limits are enforced"; "perception system limitations, fallback strategies" | same |
| Energy + quality | "energy efficiency and safety as top priorities"; "energy-aware perception stacks"; "drive quality" | same |
| AI and innovation list | "**Predictive vehicle control for an Autocross course**"; "AI-driven requirements generation for verification, and testing generation for validation"; ML controllers welcome where "trade-offs against traditional controller algorithms are considerably positive" | same |
| MathWorks award criteria (the most recent published version is from EcoCAR 3) | "plant modeling, control design and tuning, data analysis, development process, and lessons learned" | [Embedded Computing Design](https://embeddedcomputing.com/application/industrial/student-competitions-foster-real-world-engineering-principles) (search summary) |
| Tooling | "Simulate driving scenarios", "verify the functionality of embedded software", Requirements Toolbox, Simulink Test | [MathWorks Automated Driving](https://www.mathworks.com/solutions/automotive/automated-driving.html) |

**SOTIF (ISO 21448) does not appear** in the RFP text we extracted. EcoCAR documents cite ISO 26262/FMEA/HARA instead. We can describe our 40 m visibility limit as a SOTIF-style performance limitation, but we should present that as our own framing, not theirs.

## Q5. Sensor fusion: what "fast and cheap" means, visuals, and 3 numpy ideas

**What production and student teams do.** Research state of the art is BEV feature fusion such as BEVFusion, which needs a GPU ([arXiv 2205.13542](https://arxiv.org/html/2205.13542v3)). Production ADAS mostly uses **object-level (late) fusion with Kalman tracking and assignment** ([arXiv 2103.07825](https://arxiv.org/pdf/2103.07825); [arXiv 2304.10410](https://arxiv.org/pdf/2304.10410)). Student race teams typically use "lidar proposes, camera classifies".

**Numbers we can cite for "fast and cheap":**

- The ETH AMZ lidar pipeline takes about **32 ms** per scan on x86. Their camera CNN ran on a 4 W Edge TPU. The team **dropped the camera module at its 2021 competitions because latency, complexity and compute outweighed the gain** ([ar5iv 2210.10933](https://ar5iv.labs.arxiv.org/html/2210.10933)).
- The PointPainting projection step costs about **0.75 ms** ([arXiv 1911.10150](https://arxiv.org/pdf/1911.10150)).
- Aptiv quoted long-range lidar at about $500–650 ([S&P Global](https://autotechinsight.spglobal.com/news/5277667/future-of-imaging-radar-interview-with-aptiv-)).
- 4D radar is pitched as the cheap lidar complement because it measures velocity directly ([arXiv 2504.03438](https://arxiv.org/html/2504.03438v1)).

No verified industry-wide "perception < X ms" budget was found, so **we should use our own measured ms/frame as the headline**.

**Visuals engineers trust.** The MathWorks track-level fusion example shows radar, lidar and fused tracks labelled R/L/F, plus **GOSPA, missed-target and false-track plots over time** ([MathWorks](https://www.mathworks.com/help/lidar/ug/track-level-fusion-of-radar-and-lidar.html)). Tools such as Foxglove project lidar onto the image specifically to prove calibration ([Foxglove](https://foxglove.dev/blog/introducing-foxglove-studios-new-image-panel)).

**CARLA defaults to watch** ([CARLA sensors](https://carla.readthedocs.io/en/latest/ref_sensors/)):

- The lidar range defaults to 10 m, so raise it.
- Lidar points are in a **y-right** frame.
- Radar returns are (velocity, azimuth, altitude, depth).
- The camera defaults to 800×600 at 90° FOV.

**Three ~3 h numpy demos.** All three use synthetic ground truth so we can compute metrics. Every number here is a target to measure, not a result.

1. **Paint-and-cluster cone detector** (AMZ + PointPainting style). Ground removal, then a BEV grid with connected components, then projecting the centroids into the camera and reading their colour. Show the camera image with depth-coloured lidar beside a BEV view. Report precision/recall by range bin and ms per stage.
2. **Radar-Doppler + lidar object tracker** (MathWorks track-level style). A batched constant-velocity KF, Mahalanobis gating with greedy GNN, and covariance intersection. Show R/L/F tracks with 2σ ellipses and GOSPA over time, plus how many frames it takes for the velocity estimate to converge.
3. **"Fusion in a few ms" latency waterfall.** Compare naive loops with the vectorised version per stage, and plot a p50/p99 histogram against the 50 ms and 100 ms lines. This ties straight back to the AMZ story.

## Q6. Who wrote the simulator, and who the judges likely are

**dszurek is Daniel Szurek.** He is a UA accelerated MS CS student, UA EcoCAR's CAV lead for 2026, and his personal site describes him as an "ADAS Software Engineer" ([GitHub](https://github.com/dszurek); [UA Engineering](https://news.eng.ua.edu/?p=11538); [danielszurek.com](https://www.danielszurek.com)). His commit history ([commits](https://github.com/dszurek/ecocar-hackathon-sim/commits/main)) runs as follows:

- **30 Sep 2026:** built the harness.
- **13 Oct 2026:** added the protected evaluation courses.
- **16 Oct 2026:** fitted the bicycle model to CARLA telemetry, set the official weighting, and hardened the sandbox. The evaluation courses moved to a private repository on the same day.

A second contributor, "wscampbell", committed sandbox and bicycle-fit changes. It is plausible that this is Wes Campbell, UA's propulsion controls & modeling lead **(unverified; the GitHub profile is empty)**. The README refers to an "EIC autonomous autocross event" and to MIL/SIL integration with MATLAB/Simulink/Embedded Coder ([repo](https://github.com/dszurek/ecocar-hackathon-sim)).

**Judges are not named anywhere (unverified).** The most likely panel is UA EcoCAR CAV members and/or advisors, possibly including sponsor engineers. "HackBama" itself could not be confirmed. hackbama.com returned HTTP 522, and a search summary described HackBama as an information-security community ([hackbama.com](https://hackbama.com/); [X @HackBama27](https://x.com/hackbama27)). Our ROADMAP mentions a coordinator, but the research did not identify who she is.

## What this means for us (proposals for docs/ROADMAP.md)

**1. One-sentence pitch** (to fill the TBD in CLAUDE.md):

> *"Predictive vehicle control for an autocross course: a map-free controller that plans a minimum-curvature racing line in the 40 m it can see, verifies every plan against a fitted dynamic-bicycle plant model, and enforces its friction and visibility limits within a 50 ms single-core budget, with a safe fallback when anything fails."*

**2. Priorities.** Keep "clean > fast > clever". Add two cheap items that are worth innovation points, so we can tell the story in sponsor language:

- **(a) A requirements table** in `docs/INNOVATION.md`. Each requirement gets a pass/fail result from `telemetry.csv` (Viraj, `tools/`, about 30 min).
- **(b) An energy and drive-quality proxy** logged in `debug`. Positive tractive energy Σ max(0, m·a·v)·dt, braking energy, and RMS longitudinal and lateral jerk. These are labelled as proxies, not as AVTC metrics.

Example requirements for the table:

- REQ-1: no throttle+brake overlap
- REQ-2: predicted friction use ≤ 0.95
- REQ-3: p99 step time < 45 ms
- REQ-4: 0 cones and 0 gates on 5 seeds
- REQ-5: CARLA vs bicycle lap time within 2.5 %

**3. Map our features to sponsor language** (slides and README):

| Our feature | Sponsor term |
|---|---|
| Friction-ellipse limiter, steering-rate respect | "operational limits enforced" |
| Horizon-end visibility guard | "perception system limitations" handled by design (SOTIF-style, our framing) |
| Centerline pure-pursuit fallback on exception or overrun (`debug.fallback`) | "fallback strategy" / degraded mode |
| `predict.py` rollout using constants fitted to CARLA telemetry | "verified plant model"; "predictive vehicle control" |
| Bicycle backend → CARLA `compare` | MIL on a fitted plant, cross-validated in higher-fidelity simulation. Call this "SIL-style": it is not true SIL of generated code. |
| `selftest` + `batch` sweep + pytest on synthetic `RoadModel`s | "requirement-based testing", scenario-based testing |
| Separate modules with frozen data contracts | "decoupled subsystem development" |
| p99 step time, memory, no overruns | real-time embedded budget |
| Slip adaptation | online plant-model correction (MathWorks "control design and tuning") |
| Fusion tool (object-level, Kalman/GNN) | continues UA's 2019 camera+radar precedent; "energy-aware perception" (ms and W per frame) |

**4. Demo story** (a small reorder of the 4-minute plan):

1. Open with the RFP quote "Predictive vehicle control for an Autocross course" and the year-4 "human-driver run versus an autonomous run".
2. In the physics segment, add a 10 s **"limits enforced"** slide: the g-g diagram inside the ellipse, plus the requirements table with every row PASS.
3. In the fusion segment, lead with "AMZ dropped their camera for latency; ours costs X ms on one core", then show a lidar-on-image overlay and a GOSPA plot in the R/L/F convention.
4. Close with "built the way EcoCAR builds: model → verify → deploy".

**5. Emphasise, don't add.** We should not add new controller features for these sponsors. The predictive verifier is the centrepiece because it is literally on their innovation list. If fusion has to be cut, keep Demo idea 3 (the latency waterfall), because it is the cheapest and the most on-message.

## Glossary of sponsor terms

| Term | Meaning | How we use it |
|---|---|---|
| Model-Based Design (MBD) | Design driven by verified plant and controller models through test and deployment | Fitted bicycle plant → rollout verifier → controller |
| MiL / SiL / HiL / ViL | Model-, software-, hardware- and vehicle-in-the-loop testing | Bicycle = MiL; CARLA = higher-fidelity, SIL-style check |
| Plant model | Model of the vehicle being controlled | `predict.py` dynamic bicycle model |
| Requirement-based testing | Tests traced to explicit requirements | REQ table with pass/fail from telemetry |
| ODD | Operational design domain: conditions the system is designed for | "Map-free autocross, 40 m preview, cones and obstacles" |
| Operational limits | Bounds the system enforces on itself | Friction ellipse, steer rate, visibility speed cap |
| Fallback strategy | Safe behaviour when the nominal function fails | Centerline pure pursuit on error or overrun |
| FMEA / HARA | Failure-mode and hazard analyses | One-slide "failure → detection → response" table |
| ISO 26262 / SOTIF (ISO 21448) | Functional safety / safety of the intended functionality | Cite 26262 as the sponsors do; SOTIF only as our own framing |
| Drive quality | How smooth the vehicle feels (SAE metrics) | Jerk RMS, no pedal overlap, steering-rate margin |
| Energy-aware | Designing for energy consumption impact | Tractive and braking energy proxy |
| CAV / V2X | Connected and automated vehicle / vehicle-to-everything | Context only; we have no V2X |
| GOSPA | Multi-target tracking error metric | Fusion evaluation plot |
| Autono-cross | The EIC's planned autonomous autocross event | Pitch opener |

## Open questions for the coordinator

1. Who is on the judging panel (UA CAV team, advisors, sponsor engineers), and is there a rubric for the 25 innovation points?
2. Is the fusion interest about **radar** specifically, and what output do they want: an object list, a BEV map, or cones?
3. Will the judges run CARLA themselves on the same machine, and does Embedded Coder/MIL integration matter for this round?
4. Does any energy or smoothness metric affect the score, or does "physical realism" cover it all?
5. Are the hackathon results used to inform UA's EIC autocross stack? If so, do they want our code to stay readable or portable to Simulink?
6. Is there a preferred demo format (live vs recorded, slide count), and are the 3–5 minutes strict?
