#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case "${1:-help}" in
 check|capture)
  source /opt/ros/jazzy/setup.bash
  NAV=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["navigation_root"])' "$HERE/config/settings.json")
  [[ "$NAV" = /* ]] || NAV="$HERE/$NAV"
  source "$NAV/driver_workspace/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
  export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
  export FASTRTPS_DEFAULT_PROFILES_FILE="$NAV/config/fastdds_navigation.xml"
  export LD_LIBRARY_PATH="$NAV/sdk-install/lib:${LD_LIBRARY_PATH:-}"
  ;;
esac
export OPENBLAS_NUM_THREADS=2
exec python3 "$HERE/calibrate.py" "$@"
