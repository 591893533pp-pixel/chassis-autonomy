#!/usr/bin/env bash
# Source, do not execute. No hardware or services are started.
source /opt/ros/jazzy/setup.bash
CHASSIS_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -L)
if [[ -f "$CHASSIS_ROOT/sensors/driver_workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash" ]]; then
  source "$CHASSIS_ROOT/sensors/driver_workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
else
  export AMENT_PREFIX_PATH="$CHASSIS_ROOT/build_support/install:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$CHASSIS_ROOT/build_support/install/lib:${LD_LIBRARY_PATH:-}"
fi
export AMENT_PREFIX_PATH="$CHASSIS_ROOT/mapping_ws/install:$CHASSIS_ROOT/.runtime/model_prefix:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$CHASSIS_ROOT/mapping_ws/install/lib:$CHASSIS_ROOT/sensors/sdk-install/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=31 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=2
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export FASTRTPS_DEFAULT_PROFILES_FILE="$CHASSIS_ROOT/sensors/config/fastdds_navigation.xml"
