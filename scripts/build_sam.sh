#!/usr/bin/env bash
set -eo pipefail
# Use an ASCII absolute path: ROSIDL in this environment splits non-ASCII paths.
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -L)
source /opt/ros/jazzy/setup.bash
if [[ "${1:-}" == --local-message-only ]]; then
  cmake -S "$HERE/build_support/livox_ros_driver2" -B "$HERE/build_support/build_ascii" -DCMAKE_INSTALL_PREFIX="$HERE/build_support/install" -DPython3_EXECUTABLE=/usr/bin/python3
  cmake --build "$HERE/build_support/build_ascii" -j2
  cmake --install "$HERE/build_support/build_ascii"
  export CMAKE_PREFIX_PATH="$HERE/build_support/install:${CMAKE_PREFIX_PATH:-}"
  export AMENT_PREFIX_PATH="$HERE/build_support/install:${AMENT_PREFIX_PATH:-}"
else
  source "$HERE/sensors/driver_workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
  export CMAKE_PREFIX_PATH="$HERE/sensors/driver_workspace/install/livox_ros_driver2:${CMAKE_PREFIX_PATH:-}"
fi
cmake -S "$HERE/mapping_ws/src/fast_lio_sam" -B "$HERE/mapping_ws/build" -DCMAKE_INSTALL_PREFIX="$HERE/mapping_ws/install" -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$HERE/mapping_ws/build" -j2
cmake --install "$HERE/mapping_ws/build"
