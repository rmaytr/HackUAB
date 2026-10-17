"""Run a fusion benchmark and produce a visual without CARLA.

From HackUAB repo root:
  simulator/.venv/bin/python -m fusion.demo --frames 120 --png fusion_demo.png
  simulator/.venv/bin/python -m fusion.demo --full-controller --frames 20

This is an offline simulation, not a lap-time or official scoring measurement.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import numpy as np
from .schema import FrameBundle
from .synthetic import make_frame
from .fuse import fuse_frame, project_to_image


COLORS = {0: "#337cf1", 1: "#e0bc28", 2: "#ef8335", -1: "#9ca3af"}


def _score_frame(result, true_xy, true_colors):
    """Greedy one-to-one matching; accuracy requires correct class and <0.5m position."""
    used = set()
    hits = []
    errors = []
    for xy, color in zip(true_xy, true_colors):
        opts = [(float(np.linalg.norm(o.xy-xy)), i, o) for i, o in enumerate(result.objects)
                if i not in used and o.color == int(color)]
        if not opts:
            continue
        d, j, _ = min(opts, key=lambda x: x[0])
        if d < .5:
            used.add(j)
            hits.append(1)
            errors.append(d)
    return len(hits), len(true_xy), len(result.objects), errors


def _figure(frame, result, truth_xy, truth_colors, file_path: str, planned=None, baseline=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={"width_ratios":[1.08,1.]})
    ax = axes[0]
    ax.imshow(frame.image)
    ax.set_title("RGB camera + LiDAR-proposed fused labels")
    for ob in result.objects:
        uv = project_to_image(np.array([[ob.xy[0], ob.xy[1], .43]]), frame)[0]
        if not np.isfinite(uv).all():
            continue
        ax.scatter(*uv, s=110, marker="o", facecolors="none",
                   edgecolors=COLORS[ob.color], linewidth=2)
        ax.annotate(f"{['blue','yellow','orange','?'][ob.color] if ob.color >= 0 else '?'} {ob.xy[0]:.0f}m",
                    uv + [4,-6], color="white", fontsize=7)
    ax.set_xlim(0, frame.image.shape[1]); ax.set_ylim(frame.image.shape[0], 0)
    ax.set_xlabel("pixels")
    ax = axes[1]
    lidar = frame.lidar
    good = (lidar[:, 2] > .08) & (lidar[:, 2] < 1.5)
    ax.scatter(lidar[good, 1], lidar[good, 0], c="gray", s=3, alpha=.42, label="LiDAR returns")
    for ob in result.objects:
        ax.scatter(ob.xy[1], ob.xy[0], s=65, c=COLORS[ob.color], edgecolors="black", linewidths=.5)
        if np.isfinite(ob.radial_velocity):
            ax.annotate(f"radar {ob.radial_velocity:+.1f} m/s", (ob.xy[1],ob.xy[0]),
                        xytext=(3,4), textcoords="offset points", fontsize=6)
    if planned is not None and planned.path is not None:
        p = np.asarray(planned.path)
        ax.plot(p[:, 1], p[:, 0], color="#02b59f", linewidth=2.0, label="fusion-injected plan")
    if baseline is not None and baseline.path is not None:
        p = np.asarray(baseline.path)
        ax.plot(p[:, 1], p[:, 0], "k--", linewidth=1.25, label="no fused obstacles")
    ax.scatter([0], [0], c="black", marker="^", s=100, label="ego")
    ax.axvline(-3.6, color="gray", ls=":", alpha=.5)
    ax.axvline(+3.6, color="gray", ls=":", alpha=.5)
    ax.set_xlim(-5,5); ax.set_ylim(0, 36)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Vehicle-frame bird's-eye view (+y is left)")
    ax.set_xlabel("lateral y (m)"); ax.set_ylabel("forward x (m)")
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    Path(file_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(file_path, dpi=140)
    plt.close(fig)


def _load_controller():
    """Imports the EXISTING repo controller; does not copy or change its files."""
    root = Path.cwd().resolve()
    if not (root / "hackuab/controller.py").is_file() or not (root / "simulator/autocross/api.py").is_file():
        raise RuntimeError("--full-controller must run FROM THE REAL HackUAB repo root with its simulator submodule")
    sys.path.insert(0, str(root / "simulator"))
    sys.path.insert(0, str(root / "hackuab"))
    from autocross.api import RoadModel, VehicleParams, Observation, TrackInfo
    from controller import HackUABController
    return RoadModel, VehicleParams, Observation, TrackInfo, HackUABController


def test_full_controller(frame, fused, taper=None):
    """One static offline observation through both full controller variants.

    NOTE: This is not a closed-loop drive or proof of speed/score gains.
    """
    from .bridge import inject_fusion
    RoadModel, VehicleParams, Observation, TrackInfo, HackUABController = _load_controller()
    s = np.arange(0, 41, dtype=float)
    center = np.column_stack((s, np.zeros_like(s)))
    widths = np.full_like(s, 3.6)
    road = RoadModel(valid=True, range_m=40., lateral_offset=0., heading_error=0.,
                     curvature=0., width_left=3.6, width_right=3.6,
                     dist_to_left_boundary=3.6, dist_to_right_boundary=3.6,
                     preview_s=s, preview_curvature=np.zeros_like(s),
                     preview_centerline=center,
                     preview_width_left=widths, preview_width_right=widths,
                     preview_left_boundary=center+np.array([0.,3.6]),
                     preview_right_boundary=center+np.array([0.,-3.6]),
                     corners=[], obstacles=[], finish_distance=float('nan'))
    vp = VehicleParams(mass=1845.,wheelbase=3.,track_width=1.67,length=4.79,width=2.16,
                       cg_to_front=1.45,cg_to_rear=1.55,max_steer=np.deg2rad(35.),
                       max_steer_rate=2.,mu=.95,max_accel=5.2,max_decel=9.,
                       max_speed=40.,control_dt=.05,actuation_delay_steps=0)
    emptyf = np.empty(0)
    track = TrackInfo(name="fusion_offline_snapshot", closed=False, laps=1,
                      centerline=np.empty((0,2)),width_left=emptyf,width_right=emptyf,
                      cones=np.empty((0,2)),cone_colors=np.empty(0, dtype=int),
                      start_line=np.empty((0,2)),finish_line=np.empty((0,2)),
                      gates=np.empty((0,2,2)),full_knowledge=False)
    obs = Observation(t=frame.t,step=0,x=float(frame.ego_pose[0]),y=0.,yaw=0.,
                      vx=frame.ego_speed_mps,vy=0.,yaw_rate=0.,ax=0.,ay=0.,
                      speed=frame.ego_speed_mps,steer=0.,throttle=0.,brake=0.,
                      progress=float(frame.ego_pose[0]),lap=0,timing_started=True,
                      cones=None,road=road)
    new_obs = inject_fusion(obs, fused)
    cfg = {"strict": True}
    if taper is not None:
        cfg["obstacle_taper_m"] = float(taper)
    ctrl_before, ctrl_after = HackUABController(cfg), HackUABController(cfg)
    ctrl_before.on_start(track,vp)
    ctrl_after.on_start(track,vp)
    cmd_before = ctrl_before.step(obs)
    cmd_after = ctrl_after.step(new_obs)
    return cmd_before, cmd_after, new_obs


def main() -> None:
    ap = argparse.ArgumentParser(description="Synthetic LiDAR+RGB+radar fusion with CPU benchmark")
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--png", default="fusion_demo.png")
    ap.add_argument("--save-npz", metavar="FILE")
    ap.add_argument("--recorded", metavar="FILE", help="an .npz exported by CARLA recorder")
    ap.add_argument("--full-controller", action="store_true", help="inject detections into FULL HackUAB controller once")
    ap.add_argument("--taper", type=float, default=None, help="offline controller taper override; no params.py edits")
    args = ap.parse_args()
    if args.frames < 1:
        ap.error("--frames must be >= 1")
    if args.recorded and args.full_controller:
        ap.error("Recorded CARLA frames need a matching real RoadModel/Observation; "
                 "do not feed them into the demo's synthetic straight-road model. "
                 "Use --full-controller with synthetic frames only.")
    latencies, hits, truth_count, detections, errors = [], 0, 0, 0, []
    for i in range(args.frames if args.recorded is None else 1):
        if args.recorded:
            frame = FrameBundle.load(args.recorded)
            truth_xy = np.empty((0,2)); truth_colors = np.empty(0,dtype=int)
        else:
            frame, truth_xy, truth_colors = make_frame(i)
        result = fuse_frame(frame)
        latencies.append(result.stage_ms['total'])
        if not args.recorded:
            n, t, d, errs = _score_frame(result, truth_xy, truth_colors)
            hits += n;truth_count += t;detections += d; errors += errs
        if i == 0:
            first_frame, first_result = frame, result
            first_truth, first_colors = truth_xy, truth_colors
    lats = np.asarray(latencies)
    print(f"Source: {first_frame.source}; {len(lats)} frame(s); CPU fusion ONLY (plotting/loading excluded)")
    print(f"Fusion latency: median={np.median(lats):.2f} ms; p95={np.percentile(lats,95):.2f} ms; "
          f"max={lats.max():.2f} ms")
    print("First-frame stages (ms): " + str({k:round(v,2) for k,v in first_result.stage_ms.items()}))
    if not args.recorded:
        print(f"Synthetic exact-color matches within 0.5 m: {hits}/{truth_count}; "
              f"all detections={detections}; median position error="
              f"{np.median(errors) if errors else float('nan'):.3f} m")
        print("Synthetic performance does NOT establish real CARLA precision/recall or lap-time gains.")
    else:
        print("No ground-truth labels loaded for this recorded frame; detection accuracy unverified.")
    cmd_b = cmd_f = None
    if args.full_controller:
        cmd_b, cmd_f, fobs = test_full_controller(first_frame, first_result, taper=args.taper)
        print(f"Offline full-controller before vs fusion: obstacles {len(fobs.road.obstacles)}; "
              f"steer {cmd_b.steer:+.3f}->{cmd_f.steer:+.3f}; "
              f"throttle {cmd_b.throttle:.2f}->{cmd_f.throttle:.2f}; "
              f"brake {cmd_b.brake:.2f}->{cmd_f.brake:.2f}; "
              f"fallback {cmd_b.debug.get('fallback')}->{cmd_f.debug.get('fallback')}")
        print("This is one OFFLINE observation, not a scored closed-loop race.")
    if args.png:
        _figure(first_frame,first_result,first_truth,first_colors,args.png,cmd_f,cmd_b)
        print("Visualization:",args.png)
    if args.save_npz:
        first_frame.save(args.save_npz)
        print("Saved compatible replay frame:",args.save_npz)


if __name__ == "__main__":
    main()
