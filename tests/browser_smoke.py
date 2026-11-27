#!/usr/bin/env python3
"""Synthetic ROS map/TF to loopback browser, plus read-only endpoint checks. No motors."""
import os,time,json,subprocess,signal,urllib.request,urllib.error,re
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='75';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import numpy as np,rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
ROOT=Path(__file__).resolve().parents[1]
rclpy.init();n=rclpy.create_node('synthetic_navigation_preview');tf=TransformBroadcaster(n)
pub=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
z=np.load(ROOT/'tests/fixtures/navigation_grid.npz');grid=z['grid'];msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=grid.shape[1];msg.info.height=grid.shape[0];msg.info.resolution=float(z['resolution']);msg.info.origin.position.x=float(z['origin'][0]);msg.info.origin.position.y=float(z['origin'][1]);msg.info.origin.orientation.w=1.;msg.data=grid.ravel().tolist()
log=(ROOT/'reports/browser_server.log').open('w');proc=subprocess.Popen(['python3',str(ROOT/'scripts/map_console.py'),'--navigation','--seconds','45'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
chrome=None
try:
 start=time.monotonic();checked=False
 while time.monotonic()-start<35:
  stamp=n.get_clock().now().to_msg();msg.header.stamp=stamp;pub.publish(msg);t=TransformStamped();t.header.frame_id='map';t.header.stamp=stamp;t.child_frame_id='base_link_trial';t.transform.translation.x=-.8;t.transform.rotation.w=1.;tf.sendTransform(t);rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()-start>4 and not checked:
   data=json.load(urllib.request.urlopen('http://127.0.0.1:18766/data'));assert data['pose'] and data['grid'];assert not data['allow_goals']
   page=urllib.request.urlopen('http://127.0.0.1:18766/').read().decode();token=re.search("'X-Chassis-Token':'([^']+)'",page).group(1)
   try:urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:18766/goal',data=b'{}',headers={'Content-Type':'application/json'}));raise AssertionError('Missing auth accepted')
   except urllib.error.HTTPError as e:assert e.code==403
   result=json.load(urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:18766/goal',data=b'{"x":0,"y":0,"yaw":0}',headers={'Content-Type':'application/json','X-Chassis-Token':token})))
   assert result['ok'] is False and '只读' in result['error']
   chrome=subprocess.Popen(['google-chrome','--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--no-proxy-server','--window-size=1280,900','--screenshot='+str(ROOT/'reports/map_console_smoke.png'),'--timeout=10000','--virtual-time-budget=3000','http://127.0.0.1:18766'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);checked=True
  if checked and chrome.poll() is not None:assert chrome.returncode==0;break
 assert checked and (ROOT/'reports/map_console_smoke.png').exists()
 print(json.dumps({'synthetic_only':True,'map_and_body_visible':True,'readonly_goal_rejected':True,'missing_token_rejected':True,'hardware_started':False}))
finally:
 if chrome and chrome.poll() is None:chrome.terminate();chrome.wait(timeout=5)
 os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);log.close();n.destroy_node();rclpy.try_shutdown()
