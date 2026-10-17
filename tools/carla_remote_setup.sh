#!/usr/bin/env bash
# One-shot CARLA setup for a fresh Ubuntu 22.04/24.04 cloud GPU VM (e.g. AWS g4dn.xlarge, NVIDIA T4).
#
# It installs Docker + the NVIDIA Container Toolkit (if missing), starts the official
# carlasim/carla:0.9.16 server headless in Docker, clones our repo with the simulator submodule,
# runs the simulator's setup, then runs `autocross check` and a CARLA lap of the baseline and of
# our controller on practice_technical.
#
# The WHOLE harness runs on the VM next to the server. Don't point a laptop at a remote CARLA
# server: the loop is synchronous, so internet latency on every tick makes runs crawl.
#
# Usage on the VM (needs an NVIDIA driver; the AWS "Deep Learning Base ... GPU AMI (Ubuntu)" has one):
#   curl -fsSLO https://raw.githubusercontent.com/rmaytr/HackUAB/main/tools/carla_remote_setup.sh
#   bash carla_remote_setup.sh
# Re-running is safe: every step is skipped when already done.
#
# Options (environment variables):
#   REPO_URL=...        repo to clone            (default https://github.com/rmaytr/HackUAB.git)
#   WORKDIR=...         where to clone it        (default ~/HackUAB)
#   QUALITY=Low|Epic    CARLA render quality     (default Low; Epic for demo footage)
#   SKIP_RUNS=1         set up only, no laps
#   RUN_SELFTEST=1      also run the full selftest on both backends (~several minutes)
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/rmaytr/HackUAB.git}"
WORKDIR="${WORKDIR:-$HOME/HackUAB}"
CARLA_IMAGE="${CARLA_IMAGE:-carlasim/carla:0.9.16}"
QUALITY="${QUALITY:-Low}"
PORT=2000
CARLA_DIST="$HOME/carla_dist"          # CARLA_ROOT-style folder holding the client wheel
SIM="$WORKDIR/simulator"
VPY="$SIM/.venv/bin/python"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { printf '\n\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------------------------
step "1/7 Checking the NVIDIA driver"
command -v nvidia-smi >/dev/null 2>&1 || die "nvidia-smi not found. Use a GPU image with drivers \
(AWS: 'Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)'), or run: \
sudo apt-get install -y ubuntu-drivers-common && sudo ubuntu-drivers install && sudo reboot"
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader \
  || die "nvidia-smi failed: the driver is installed but not loaded (try a reboot)"

# ---------------------------------------------------------------------------------------------
step "2/7 System packages (git, python venv)"
sudo apt-get update -qq
sudo apt-get install -y -qq git curl ca-certificates gnupg python3 python3-venv python3-pip >/dev/null

# ---------------------------------------------------------------------------------------------
step "3/7 Docker + NVIDIA Container Toolkit"
if ! command -v docker >/dev/null 2>&1; then
  sudo apt-get install -y -qq docker.io >/dev/null
  sudo systemctl enable --now docker
fi
if ! sudo docker info 2>/dev/null | grep -qi 'nvidia'; then
  # official NVIDIA apt repository (docs.nvidia.com/datacenter/cloud-native/container-toolkit)
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
  sudo apt-get update -qq
  sudo apt-get install -y -qq nvidia-container-toolkit >/dev/null
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
fi
sudo docker info 2>/dev/null | grep -qi 'nvidia' || die "Docker has no nvidia runtime after install"

# ---------------------------------------------------------------------------------------------
step "4/7 CARLA image ($CARLA_IMAGE) and client wheel"
sudo docker pull -q "$CARLA_IMAGE"
if [ ! -d "$CARLA_DIST/PythonAPI/carla/dist" ]; then
  # setup.sh tries `pip install carla==0.9.16` first; this wheel is its fallback via CARLA_ROOT
  mkdir -p "$CARLA_DIST/PythonAPI/carla"
  cid=$(sudo docker create "$CARLA_IMAGE")
  sudo docker cp "$cid:/home/carla/PythonAPI/carla/dist" "$CARLA_DIST/PythonAPI/carla/" 2>/dev/null \
    || echo "    (no wheel found in the image; relying on PyPI)"
  sudo docker rm "$cid" >/dev/null
  sudo chown -R "$USER" "$CARLA_DIST"
fi

# ---------------------------------------------------------------------------------------------
step "5/7 Starting the CARLA server headless (quality $QUALITY, port $PORT)"
sudo docker rm -f carla-server >/dev/null 2>&1 || true
sudo docker run -d --name carla-server --runtime=nvidia --net=host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  "$CARLA_IMAGE" bash CarlaUE4.sh -RenderOffScreen -nosound \
  -quality-level="$QUALITY" -carla-rpc-port="$PORT" >/dev/null

# ---------------------------------------------------------------------------------------------
step "6/7 Our repo + simulator submodule + harness setup"
if [ -d "$WORKDIR/.git" ]; then
  git -C "$WORKDIR" pull --ff-only
  git -C "$WORKDIR" submodule update --init --recursive
else
  git clone --recurse-submodules "$REPO_URL" "$WORKDIR"
fi
CARLA_ROOT="$CARLA_DIST" bash "$SIM/scripts/setup.sh"
"$VPY" -c "import carla" 2>/dev/null || die "CARLA Python client did not install into $SIM/.venv"

echo "    waiting for the CARLA server (first start compiles shaders, up to ~3 min)..."
ok=0
for _ in $(seq 1 36); do
  if "$VPY" -c "import carla; c = carla.Client('localhost', $PORT); c.set_timeout(5.0); print('    server', c.get_server_version())" 2>/dev/null; then
    ok=1; break
  fi
  sudo docker ps -q -f name=carla-server | grep -q . || break      # container died: stop waiting
  sleep 5
done
if [ "$ok" != 1 ]; then
  echo "---- last CARLA server log lines ----"; sudo docker logs --tail 40 carla-server 2>&1 || true
  die "CARLA server not reachable. Vulkan errors above usually mean the container can't see the GPU's \
graphics libraries: check that 'nvidia-smi' works, then re-run this script."
fi

# ---------------------------------------------------------------------------------------------
step "7/7 Check + laps on CARLA"
cd "$SIM"
"$VPY" -m autocross check
if [ "${SKIP_RUNS:-0}" != 1 ]; then
  "$VPY" -m autocross run -c controllers/baseline_pure_pursuit.py -t tracks/practice_technical.yaml \
    --backend carla --no-render --team baseline
  "$VPY" -m autocross run -c ../hackuab/controller.py -t tracks/practice_technical.yaml \
    --backend carla --no-render --team hackuab
fi
if [ "${RUN_SELFTEST:-0}" = 1 ]; then
  "$VPY" -m autocross selftest -c ../hackuab/controller.py --carla --team hackuab --no-render
fi

cat <<EOF

Done. CARLA keeps running in Docker (container 'carla-server').

  Run more laps:     cd $SIM && .venv/bin/python -m autocross run -c ../hackuab/controller.py \\
                       -t tracks/practice_technical.yaml --backend carla --no-render
  Pull new code:     git -C $WORKDIR pull && git -C $WORKDIR submodule update --init
  Copy results home: scp -r <user>@<vm-ip>:$SIM/runs/<run-folder> .      (run this on your laptop)
  Server logs:       sudo docker logs --tail 50 carla-server
  Stop the server:   sudo docker rm -f carla-server

STOP THE VM in your cloud console when you're not using it, because GPU instances bill by the hour.
EOF
