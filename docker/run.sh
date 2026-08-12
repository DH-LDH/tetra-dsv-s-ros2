#!/usr/bin/env bash
# Open a shell in the Humble/Fortress dev container with the workspace mounted.
#
#   ./docker/run.sh              # interactive shell
#   ./docker/run.sh colcon build # run one command and exit
#
# The host user was added to the "docker" group after this login session began,
# so a plain `docker` still fails until you log out and back in. `sg docker -c`
# picks the group up immediately; drop the wrapper once you have re-logged in.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-tetra-humble:dev}"
# Unique per invocation. A fixed name would stop you opening a second shell
# alongside a running simulation - which is exactly what teleop needs.
CONTAINER="${CONTAINER:-tetra-humble-$$}"

DOCKER=(docker)
if ! docker info >/dev/null 2>&1 && sg docker -c "docker info" >/dev/null 2>&1; then
  DOCKER=(sg docker -c)
  WRAP=1
fi

# Let container GUIs draw on the host X server. Gazebo and RViz both need this.
xhost +local:root >/dev/null 2>&1 || true

ARGS=(--rm)
# Allocate a TTY only when there is one to allocate, so the script also works
# from CI, a pipe, or a non-interactive agent shell.
[ -t 0 ] && ARGS+=(-it)

ARGS+=(
  --name "${CONTAINER}"
  --net host
  --ipc host
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -e XDG_RUNTIME_DIR=/tmp/runtime-dev
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${WS_ROOT}:/ws"
  -w /ws
)

# GPU. Fortress renders the VLP-16's rays on the GPU (gpu_lidar), so this is
# not just about a pretty window - without it the sensor itself crawls.
#
# NVIDIA_DRIVER_CAPABILITIES is the part everyone misses: "--gpus all" grants
# only compute+utility, so OpenGL silently falls back to llvmpipe and Gazebo
# gets slow with no error message anywhere. "graphics" is what turns that into
# real hardware GL. Verify with:
#     ./docker/run.sh glxinfo -B | grep "OpenGL renderer"
# It must say NVIDIA, not llvmpipe.
if docker info 2>/dev/null | grep -q nvidia || sg docker -c "docker info" 2>/dev/null | grep -q nvidia; then
  ARGS+=(
    --gpus all
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility
  )
else
  echo "warning: no nvidia container runtime; falling back to software rendering" >&2
  [ -d /dev/dri ] && ARGS+=(--device /dev/dri)
fi

# Software-rendering escape hatch: LIBGL_ALWAYS_SOFTWARE=1 ./docker/run.sh
[ -n "${LIBGL_ALWAYS_SOFTWARE:-}" ] && ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)

if [ $# -eq 0 ]; then
  CMD=(/bin/bash)
else
  # The command travels as base64 in an env var, then gets eval'd.
  #
  # Base64 because sg only accepts a single string and flattening argv through
  # printf %q emits bash-only $'...\n...' quoting that sg's /bin/sh mangles.
  # An env var rather than a pipe because piping the script into bash makes the
  # pipe the shell's stdin, so anything interactive - teleop_twist_keyboard
  # above all - reads EOF instead of the keyboard and exits immediately.
  B64=$(printf '%s' "$*" | base64 -w0)
  ARGS+=(-e "RUN_B64=${B64}")
  CMD=(/bin/bash -lc 'eval "$(printf %s "$RUN_B64" | base64 -d)"')
fi

if [ -n "${WRAP:-}" ]; then
  printf -v QUOTED '%q ' docker run "${ARGS[@]}" "${IMAGE}" "${CMD[@]}"
  exec sg docker -c "${QUOTED}"
else
  exec docker run "${ARGS[@]}" "${IMAGE}" "${CMD[@]}"
fi
