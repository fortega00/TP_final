#!/usr/bin/env bash
# Gestiona el entorno Python del TP dentro del contenedor habitual.
set -euo pipefail

SLAM_SOURCE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_CONTAINER="${SLAM_CONTAINER:-yahboom_rosmaster}"
SLAM_CONTAINER_WS="${SLAM_CONTAINER_WS:-/home/alumno/rosmaster_ws}"

usage() {
  cat <<'EOF'
Usage: ./slam/scripts/container_env.sh {setup|enter|status}

setup   Create/update the isolated Humble environment and enable Numba.
enter   Open a Bash shell with ROS, the workspace and Numba enabled.
status  Show Python, dependency versions and the backend without starting nodes.

Uses the existing running container. Does not start ROS, RViz or Gazebo.
Override SLAM_CONTAINER and SLAM_CONTAINER_WS if needed.
EOF
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  setup|enter|status) SLAM_ACTION="$1" ;;
  *) usage >&2; exit 2 ;;
esac
[[ $# == 1 ]] || { usage >&2; exit 2; }

if [[ "$(docker inspect --format '{{.State.Running}}' "$SLAM_CONTAINER" 2>/dev/null)" != true ]]; then
  echo "Start the existing container first: ~/PRA_00/container.sh start" >&2
  exit 1
fi

if [[ "$SLAM_ACTION" == setup ]]; then
  # Descarga ruedas en el host; el contenedor no necesita pip global ni red.
  SLAM_PLATFORM="$(python3 -c 'import platform, sys; print(sys.version_info[:2], platform.machine())')"
  SLAM_CONTAINER_PLATFORM="$(docker exec "$SLAM_CONTAINER" /usr/bin/python3 -c \
    'import platform, sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 required"; print(sys.version_info[:2], platform.machine())')"
  if [[ "$SLAM_PLATFORM" != "$SLAM_CONTAINER_PLATFORM" ]]; then
    echo "Setup requires host and container to use Python 3.10 and the same architecture." >&2
    exit 1
  fi
  SLAM_WHEELS="$(mktemp -d)"
  trap 'rm -rf -- "$SLAM_WHEELS"' EXIT
  python3 -m pip download --only-binary=:all: --no-deps --dest "$SLAM_WHEELS" \
    pip==24.0 numba==0.57.1 llvmlite==0.40.1
  tar -C "$SLAM_SOURCE" -cf - scripts/tp_env.bash \
    requirements.txt requirements-acceleration-humble.txt \
    | docker exec -i "$SLAM_CONTAINER" bash -c \
      'set -e; mkdir -p "$1/.tp_env"; tar -xf - -C "$1/.tp_env"' _ "$SLAM_CONTAINER_WS"
  docker cp "$SLAM_WHEELS/." "$SLAM_CONTAINER:$SLAM_CONTAINER_WS/.tp_env/"
  docker exec "$SLAM_CONTAINER" bash "$SLAM_CONTAINER_WS/.tp_env/scripts/tp_env.bash" setup
else
  if ! docker exec "$SLAM_CONTAINER" test -f "$SLAM_CONTAINER_WS/.tp_env/scripts/tp_env.bash"; then
    echo "Environment is not installed. Run: ./slam/scripts/container_env.sh setup" >&2
    exit 1
  fi
  if [[ "$SLAM_ACTION" == enter ]]; then
    # El helper habitual conserva su gestión de X11 y las cookies del contenedor.
    if [[ -x "$HOME/PRA_00/container.sh" ]]; then
      CONTAINER_NAME="$SLAM_CONTAINER" "$HOME/PRA_00/container.sh" enter
    else
      docker exec -it -w "$SLAM_CONTAINER_WS" "$SLAM_CONTAINER" bash -i
    fi
  else
    docker exec "$SLAM_CONTAINER" bash -c \
      'source "$1/.tp_env/scripts/tp_env.bash" && tp-env status' _ "$SLAM_CONTAINER_WS"
  fi
fi
