#!/usr/bin/env bash
# Reuse the Humble image from the earlier practicals without changing it.
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_RESULTS="${TASK_RESULTS:-${TASK_ROOT}/artifacts/ros}"
TASK_IMAGE="${TASK_IMAGE:-yahboom_rosmaster:latest}"
mkdir -p "$TASK_RESULTS"
docker run --rm --network none \
  -e ROS_DOMAIN_ID=71 -e ROS_LOCALHOST_ONLY=1 \
  -v "${TASK_ROOT}:/workspace/src/tp_slam:ro" \
  -v "${TASK_RESULTS}:/results:rw" -w /workspace \
  "$TASK_IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon --log-base /tmp/slam-log build --build-base /tmp/slam-build \
      --install-base /tmp/slam-install --packages-select tp_slam
    source /tmp/slam-install/setup.bash
    python3 -m unittest discover -s /workspace/src/tp_slam/tests -v
    ign sdf -k /workspace/src/tp_slam/worlds/office.sdf
    python3 /workspace/src/tp_slam/tests/ros_smoke.py --mode toy --output /results/toy
    python3 /workspace/src/tp_slam/tests/make_test_bag.py --output /tmp/slam-test-bag
    python3 /workspace/src/tp_slam/tests/ros_smoke.py --mode bag --bag-path /tmp/slam-test-bag --output /results/bag
    python3 /workspace/src/tp_slam/tests/ros_smoke.py --mode bag --odom-source tf --bag-path /tmp/slam-test-bag --output /results/bag_tf
  '
