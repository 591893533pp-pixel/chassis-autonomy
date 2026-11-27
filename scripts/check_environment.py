#!/usr/bin/env python3
"""Read-only readiness report. Exit 2 when dependencies/physical acceptance are missing."""
import json,sys
from pathlib import Path
from navigation_gate import configuration_hash,require_acceptance
ROOT=Path(__file__).resolve().parents[1]
def main():
 missing=[];found={}
 try:
  from ament_index_python.packages import get_package_prefix
  for p in ['nav2_map_server','nav2_amcl','nav2_smac_planner','dwb_core','nav2_bt_navigator','nav2_collision_monitor','nav2_lifecycle_manager','nav2_rviz_plugins','rviz2','robot_state_publisher']:
   try:found[p]=get_package_prefix(p)
   except Exception:missing.append(p)
 except ImportError:missing=['ROS environment not sourced']
 try:require_acceptance();accepted=True;reason=None
 except Exception as exc:accepted=False;reason=str(exc)
 result={'fastlio_compiled':(ROOT/'mapping_ws/install/lib/fast_lio_sam/fastlio_mapping').exists(),'missing_packages':missing,'found_packages':found,'physical_acceptance':accepted,'acceptance_reason':reason,'configuration_sha256':configuration_hash(),'hardware_started':False}
 print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if not missing and accepted else 2
if __name__=='__main__':raise SystemExit(main())
