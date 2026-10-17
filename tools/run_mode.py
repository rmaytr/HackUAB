"""Run the controller in one driving mode: performance (fastest lap) or eco (energy-optimal lap).

Dev tool, NOT part of the submission. Run it from simulator/ with the simulator's venv:

    .venv\\Scripts\\python.exe ..\\tools\\run_mode.py performance selftest
    .venv\\Scripts\\python.exe ..\\tools\\run_mode.py eco selftest
    .venv\\Scripts\\python.exe ..\\tools\\run_mode.py eco run practice_technical            # bicycle, no render
    .venv\\Scripts\\python.exe ..\\tools\\run_mode.py eco run practice_technical --view     # extra args pass through
    .venv\\Scripts\\python.exe ..\\tools\\run_mode.py performance selftest --carla

Each mode is a YAML in hackuab/ (performance.yaml, eco.yaml) whose `mode:` key picks a profile
from params.MODES. Add --strict to re-raise module errors instead of using the fallback.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROLLER = os.path.join(ROOT, "hackuab", "controller.py")
MODES = ("performance", "eco")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=MODES)
    ap.add_argument("action", choices=("run", "selftest"))
    ap.add_argument("track", nargs="?", default="practice_technical", help="track name for `run`")
    ap.add_argument("--strict", action="store_true", help="re-raise module errors (development)")
    args, extra = ap.parse_known_args()

    cfg = os.path.join(ROOT, "hackuab", f"{args.mode}.yaml")
    if args.strict:  # same mode file plus strict: true, in a temp copy outside the submission folder
        tmp = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        with open(cfg) as f:
            tmp.write(f.read() + "\nstrict: true\n")
        tmp.close()
        cfg = tmp.name

    cmd = [sys.executable, "-m", "autocross", args.action, "-c", CONTROLLER, "--controller-config", cfg,
           "--team", f"hackuab_{args.mode}"]
    if args.action == "run":
        track = args.track if args.track.endswith(".yaml") else os.path.join("tracks", f"{args.track}.yaml")
        cmd += ["-t", track]
        if "--backend" not in extra:
            cmd += ["--backend", "bicycle"]
        if "--view" not in extra and "--no-render" not in extra:
            cmd += ["--no-render"]
    elif "--no-render" not in extra:
        cmd += ["--no-render"]
    cmd += extra
    print(">", " ".join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
