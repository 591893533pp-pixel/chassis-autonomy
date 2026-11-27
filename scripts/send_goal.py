#!/usr/bin/env python3
"""Send one Nav2 goal. Hardware requires the separate accepted, START-gated bridge."""
import argparse,math
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal

def main():
 p=argparse.ArgumentParser();p.add_argument('x',type=float,nargs='?');p.add_argument('y',type=float,nargs='?');p.add_argument('--yaw-deg',type=float,default=0.);p.add_argument('--cancel',action='store_true');a=p.parse_args();rclpy.init();n=rclpy.create_node('chassis_one_goal')
 try:
  if a.cancel:
   client=n.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
   if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Nav2 cancel service unavailable')
   future=client.call_async(CancelGoal.Request());rclpy.spin_until_future_complete(n,future,timeout_sec=5.)
   if not future.done():raise RuntimeError('Cancel confirmation timed out')
   print(future.result());return
  if a.x is None or a.y is None or not all(math.isfinite(v) for v in [a.x,a.y,a.yaw_deg]):raise ValueError('Provide finite x y and optional yaw')
  client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
  if not client.wait_for_server(timeout_sec=5.):raise RuntimeError('Nav2 action server unavailable')
  goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=a.x;goal.pose.pose.position.y=a.y;goal.pose.pose.orientation.z=math.sin(math.radians(a.yaw_deg)/2);goal.pose.pose.orientation.w=math.cos(math.radians(a.yaw_deg)/2)
  f=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=5.)
  if not f.done() or not f.result().accepted:raise RuntimeError('Goal was not accepted')
  print('Goal accepted. This alone does not enable the motors. Ctrl+C requests cancellation.')
  h=f.result()
  try:
   result=h.get_result_async();rclpy.spin_until_future_complete(n,result);print(result.result())
  except KeyboardInterrupt:
   cancel=h.cancel_goal_async();rclpy.spin_until_future_complete(n,cancel,timeout_sec=3.);print('Cancellation requested; verify stopped status.')
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
