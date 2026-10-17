"""OPTIONAL CARLA sidecar recorder. Run OUTSIDE the scored controller on Windows/Linux.

This module has not been exercised here because CARLA is unavailable on macOS.
It connects to an already-running CARLA 0.9.16 world, attaches sensors to an
existing vehicle and records synchronized sensor-frame triples as .npz files.
It NEVER ticks the world; the autocross harness remains simulation owner.

From HackUAB repo root on a CARLA machine, once the controller is driving:
  simulator\\.venv\\Scripts\\python.exe -m fusion.carla_capture --out fusion_capture --seconds 35
Run the official harness in another terminal WITH --realtime and WITHOUT --no-render.
Camera output needs rendering, even if CARLA is started with -Headless.
"""
from __future__ import annotations
import argparse
from collections import deque
from pathlib import Path
import threading
import time
import numpy as np
from .schema import FrameBundle


def main() -> None:
    try:
        import carla
    except ImportError as exc:
        raise SystemExit("CARLA Python package not available; use this on a Windows/Linux CARLA machine") from exc
    ap = argparse.ArgumentParser(description="Capture RGB+LiDAR+radar from an active CARLA vehicle")
    ap.add_argument("--out", default="fusion_capture")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--ego-id", type=int, default=None, help="CARLA actor ID if multiple vehicles exist")
    ap.add_argument("--seconds", type=float, default=35.)
    ap.add_argument("--max-frames", type=int, default=80)
    ap.add_argument("--every", type=int, default=2, help="save one of every N synchronized triples")
    args = ap.parse_args()
    if args.max_frames < 1 or args.every < 1 or args.seconds <= 0:
        ap.error("--seconds, --max-frames and --every must be positive")
    client = carla.Client(args.host, args.port)
    client.set_timeout(8.0)
    world = client.get_world()
    print("Waiting for a CARLA ego vehicle. Start HackUAB with --realtime in another terminal.")
    deadline = time.monotonic() + args.seconds
    car = None
    while time.monotonic() < deadline:
        actors = world.get_actors().filter("vehicle.*")
        if args.ego_id is not None:
            cars = [v for v in actors if v.id == args.ego_id]
        else:
            cars = list(actors)
        if len(cars) > 1 and args.ego_id is None:
            raise SystemExit("Multiple vehicles detected; re-run with --ego-id ID")
        if cars:
            car = cars[0]; break
        time.sleep(.2)
    if car is None:
        raise SystemExit("No ego vehicle appeared; start an autocross CARLA run before/during capture")
    print(f"Attaching to actor {car.id} {car.type_id}. Do not run with --no-render.")
    bps = world.get_blueprint_library()
    cam_bp = bps.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "320")
    cam_bp.set_attribute("image_size_y", "180")
    cam_bp.set_attribute("fov", "90")
    cam_bp.set_attribute("sensor_tick", "0.1")
    lidar_bp = bps.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("channels", "32")
    lidar_bp.set_attribute("points_per_second", "56000")
    lidar_bp.set_attribute("rotation_frequency", "10")
    lidar_bp.set_attribute("range", "40")
    lidar_bp.set_attribute("sensor_tick", "0.1")
    radar_bp = bps.find("sensor.other.radar")
    radar_bp.set_attribute("horizontal_fov", "70")
    radar_bp.set_attribute("vertical_fov", "25")
    radar_bp.set_attribute("range", "40")
    radar_bp.set_attribute("sensor_tick", "0.1")
    mounts = {"camera": (1.5, 0., 1.2), "lidar": (1.0, 0., 1.35),
              "radar": (1.5, 0., .65)}
    sensors = []
    lock = threading.Lock()
    pending = {}
    ready = deque()
    count = 0
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    def handle(name, measurement):
        try:
            if name == "camera":
                raw = np.frombuffer(measurement.raw_data, dtype=np.uint8)
                value = raw.reshape(measurement.height, measurement.width, 4)[:, :, [2,1,0]].copy()
            elif name == "lidar":
                # CARLA sensor axes: x forward, y RIGHT, z up.
                raw = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                raw[:, 0] += mounts["lidar"][0]
                raw[:, 1] = -(raw[:, 1] + mounts["lidar"][1])  # -> vehicle y LEFT
                raw[:, 2] += mounts["lidar"][2]
                value = raw
            else:
                # Each detection exposes depth/azimuth/altitude/velocity directly.
                # Remap its Cartesian position through the forward-mounted sensor
                # into the common ego origin and left-positive azimuth.
                rows = []
                for d in measurement:
                    d0 = float(d.depth); a = float(d.azimuth); elev = float(d.altitude)
                    x = mounts["radar"][0] + d0*np.cos(elev)*np.cos(a)
                    y_left = -(mounts["radar"][1] + d0*np.cos(elev)*np.sin(a))
                    z = mounts["radar"][2] + d0*np.sin(elev)
                    rows.append([np.sqrt(x*x+y_left*y_left+z*z), np.arctan2(y_left,x),
                                 np.arctan2(z,np.hypot(x,y_left)), float(d.velocity)])
                value = np.asarray(rows, dtype=np.float32).reshape(-1,4)
            with lock:
                frameid = int(measurement.frame)
                packet = pending.setdefault(frameid, {"t": float(measurement.timestamp)})
                packet[name] = value
                if all(k in packet for k in ("camera","lidar","radar")):
                    ready.append((frameid, pending.pop(frameid)))
                # Bound memory if a sensor skips an ID or no-render disables RGB.
                if len(pending) > 120:
                    for old in sorted(pending)[:-90]:
                        pending.pop(old, None)
        except Exception as exc:
            print("Capture callback error:",name,exc)

    try:
        for name, blueprint in (("camera",cam_bp),("lidar",lidar_bp),("radar",radar_bp)):
            pose = carla.Transform(carla.Location(*mounts[name]), carla.Rotation())
            actor = world.spawn_actor(blueprint, pose, attach_to=car)
            sensors.append(actor)
            actor.listen(lambda data, n=name: handle(n,data))
        joined = 0
        while time.monotonic() < deadline and count < args.max_frames:
            with lock:
                item = ready.popleft() if ready else None
            if item is None:
                time.sleep(.02); continue
            frame_id, d = item
            joined += 1
            if joined % args.every:
                continue
            try:
                v = car.get_velocity()
                speed = float(np.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
                loc = car.get_transform()
                # Position/yaw are logged for reference only. Capture is not
                # a score-valid synchronized Observation reconstruction.
                ego = np.array([loc.location.x,-loc.location.y,
                                -np.deg2rad(loc.rotation.yaw)])
            except Exception:
                ego = np.zeros(3); speed=0.
            fx = 320./(2.*np.tan(np.deg2rad(90.)/2.))
            K = np.array([[fx,0.,160.],[0.,fx,90.],[0.,0.,1.]])
            f = FrameBundle(t=d["t"],lidar=d["lidar"], radar=d["radar"],
                            image=d["camera"], K=K,
                            camera_xyz=np.array([1.5,0.,1.2]),
                            ego_pose=ego, ego_speed_mps=speed,source="carla")
            f.save(out / f"frame_{frame_id:08d}.npz")
            count += 1
            if count % 10 == 0: print("Saved",count,"synchronized frames")
    finally:
        for actor in sensors:
            try:
                actor.stop()
                actor.destroy()
            except Exception:
                pass
    print(f"Captured {count} synced frames to {out}.")
    if count == 0:
        print("No matching triples: use --realtime, keep rendering enabled, confirm the vehicle was running.")


if __name__ == "__main__":
    main()
