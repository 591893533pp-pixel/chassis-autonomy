#!/usr/bin/env python3
import time,rclpy
rclpy.init();node=rclpy.create_node('wait_for_slam_subscriptions');deadline=time.monotonic()+20
try:
 while time.monotonic()<deadline:
  if all(any(e.node_name=='laserMapping' or e.node_name=='fastlio_mapping' for e in node.get_subscriptions_info_by_topic(t)) for t in ['/slam/points','/rear_right/imu']):
   print('SLAM subscribers discovered');break
  # Names can be overridden; use both actual subscription types as fallback.
  topics=dict(node.get_topic_names_and_types())
  if all(node.count_subscribers(t)>0 for t in ['/slam/points','/rear_right/imu']):print('Data subscribers discovered');break
  rclpy.spin_once(node,timeout_sec=.1)
 else:raise RuntimeError('No SLAM subscriptions in 20 seconds')
finally:node.destroy_node();rclpy.try_shutdown()
