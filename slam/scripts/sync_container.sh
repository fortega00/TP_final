#!/usr/bin/env bash
# Copy just this ROS package into the running container used by PRA_00.
set -euo pipefail

SLAM_SOURCE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_CONTAINER="${SLAM_CONTAINER:-yahboom_rosmaster}"
SLAM_CONTAINER_WS="${SLAM_CONTAINER_WS:-/home/alumno/rosmaster_ws}"
SLAM_BUILD=1

usage() {
  cat <<'EOF'
Usage: ./scripts/sync_container.sh [--sync-only]

Copy slam into the running PRA_00 container and build only that package.
--sync-only copies sources without building.
Override SLAM_CONTAINER and SLAM_CONTAINER_WS if needed.
Afterwards enter with ~/PRA_00/container.sh enter and source install/setup.bash.
EOF
}

while (($#)); do
  case "$1" in
    --sync-only) SLAM_BUILD=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

if [[ "$(docker inspect --format '{{.State.Running}}' "$SLAM_CONTAINER" 2>/dev/null)" != true ]]; then
  echo "Start the existing container first: ~/PRA_00/container.sh start" >&2
  exit 1
fi

# The package was called tp_slam before the reorganization; two copies would both build.
if docker exec "$SLAM_CONTAINER" test -d "$SLAM_CONTAINER_WS/src/tp_slam"; then
  echo "Remove the old package first: rm -rf $SLAM_CONTAINER_WS/src/tp_slam $SLAM_CONTAINER_WS/{build,install}/tp_slam" >&2
  exit 1
fi

echo "Copying slam to ${SLAM_CONTAINER}:${SLAM_CONTAINER_WS}/src/slam"
tar --exclude='__pycache__' --exclude='*.pyc' --exclude='*.pyo' \
  -C "$SLAM_SOURCE" -cf - \
  package.xml setup.py setup.cfg requirements.txt slam launch config resource rviz worlds \
  | docker exec -i "$SLAM_CONTAINER" bash -c \
    'set -e; mkdir -p "$1/src/slam"; tar -xf - -C "$1/src/slam"' \
    _ "$SLAM_CONTAINER_WS"

if ((SLAM_BUILD)); then
  docker exec -w "$SLAM_CONTAINER_WS" "$SLAM_CONTAINER" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select slam
    source install/setup.bash
    ros2 pkg prefix slam
    ros2 pkg executables slam
  '
fi

echo "Ready. Inside the container, source ~/rosmaster_ws/install/setup.bash"
