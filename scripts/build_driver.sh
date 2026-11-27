#!/usr/bin/env bash
# Build SDK and real driver locally. Does not connect to sensors or install system services.
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -L)
python3 -c 'import sys; sys.exit(not (sys.argv[1].isascii() and " " not in sys.argv[1]))' "$HERE" || { echo 'Use an ASCII path without spaces, e.g. /home/pc/chassis-autonomy'; exit 2; }
source /opt/ros/jazzy/setup.bash
cmake -S "$HERE/sensors/Livox-SDK2" -B "$HERE/sensors/build/sdk" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HERE/sensors/sdk-install"
cmake --build "$HERE/sensors/build/sdk" -j2
cmake --install "$HERE/sensors/build/sdk"
python3 "$HERE/tools/check_livox_exports.py"
# Driver source uses ament directly; no colcon requirement, package-specific prefix matches runtime.
cmake -S "$HERE/sensors/driver_workspace/src/livox_ros_driver2" -B "$HERE/sensors/driver_workspace/build" \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DLIVOX_LIDAR_SDK_LIBRARY="$HERE/sensors/sdk-install/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$HERE/sensors/sdk-install/include" \
  -DCMAKE_INSTALL_PREFIX="$HERE/sensors/driver_workspace/install/livox_ros_driver2"
cmake --build "$HERE/sensors/driver_workspace/build" -j2
cmake --install "$HERE/sensors/driver_workspace/build"
