#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source '/home/pc/rk3588 link/navigation/driver_workspace/install/local_setup.bash'
source '/home/pc/rk3588 link/mapping/programs/FAST-LIO-SAM/install/local_setup.bash'
export LD_LIBRARY_PATH="/home/pc/rk3588 link/navigation/programs/sensors/sdk-install/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=31 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export FASTRTPS_DEFAULT_PROFILES_FILE='/home/pc/rk3588 link/navigation/config/fastdds_navigation.xml'
export OPENBLAS_NUM_THREADS=2 PYTHONUNBUFFERED=1
exec python3 "$HERE/run_mapping.py" "$@"
