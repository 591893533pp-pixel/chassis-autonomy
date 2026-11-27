#!/usr/bin/env python3
"""Synthetic closed translation route through a static 3D room. NO HARDWARE.
Tests the compiled production backend, final exports and actual loop factor wiring.
Does not validate real sensor calibration, timing, or navigation performance.
"""
import os,time,sys,subprocess,signal,json,datetime,math,argparse
from pathlib import Path
import numpy as np,yaml
os.environ['ROS_DOMAIN_ID']='74';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from sensor_msgs.msg import PointCloud2,PointField,Imu
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from verify_map_export import verify

def kinematics(t):
    duration=40.;u=np.clip((np.asarray(t)-2)/duration,0.,1.);s=2*np.pi*u-np.sin(2*np.pi*u)
    speed=2*np.pi/duration*(1-np.cos(2*np.pi*u));acc=(2*np.pi/duration)**2*np.sin(2*np.pi*u)
    pos=np.column_stack([.8*(np.cos(s)-1),.8*np.sin(s),np.zeros_like(u)])
    acceleration=np.column_stack([-.8*(np.cos(s)*speed**2+np.sin(s)*acc),.8*(-np.sin(s)*speed**2+np.cos(s)*acc),np.full_like(u,9.81)])
    return pos,acceleration

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--localization',action='store_true');a=ap.parse_args()
 run=ROOT/'reports'/(('localization_' if a.localization else 'synthetic_')+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));run.mkdir()
 cfg=yaml.safe_load((ROOT/('config/localization_lio.yaml' if a.localization else 'mapping_trial/config/lio.yaml')).read_text());cfg['/**']['ros__parameters']['mapping/extrinsic_T']=[0.,0.,0.];(run/'lio.yaml').write_text(yaml.safe_dump(cfg))
 log=(run/'fastlio.log').open('w');proc=subprocess.Popen([str(ROOT/'mapping_ws/install/lib/fast_lio_sam/fastlio_mapping'),'--ros-args','--params-file',str(run/'lio.yaml'),'-p','result_save/output_dir:='+str(run/'optimized')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();n=rclpy.create_node('synthetic_sam_input');cloud=n.create_publisher(PointCloud2,'/slam/points',20);imu=n.create_publisher(Imu,'/rear_right/imu',1000)
 transforms=[]
 from tf2_msgs.msg import TFMessage
 tfsub=n.create_subscription(TFMessage,'/tf',lambda m:transforms.extend((t.header.frame_id,t.child_frame_id) for t in m.transforms),1000)
 samples=[];n.create_subscription(__import__('nav_msgs.msg',fromlist=['Odometry']).Odometry,'/Odometry',lambda m:samples.append(m),1000)
 rng=np.random.default_rng(19);room=[]
 for axis,value in [(0,-4),(0,4),(1,-3),(1,3),(2,-1),(2,2)]:
  p=rng.uniform([-4,-3,-1],[4,3,2],(2200,3));p[:,axis]=value;room.append(p)
 room=np.concatenate(room)
 dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('tag','u1'),('line','u1'),('timestamp','<f8')]);fields=[PointField(name=k,offset=dt.fields[k][1],datatype=PointField.FLOAT64 if k=='timestamp' else PointField.UINT8 if k in ('tag','line') else PointField.FLOAT32,count=1) for k in dt.names]
 try:
  deadline=time.monotonic()+15
  while cloud.get_subscription_count()==0 or imu.get_subscription_count()==0:
   if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Backend not ready')
   rclpy.spin_once(n,timeout_sec=.05)
  epoch=time.time_ns();clock=time.monotonic()
  for frame in range(100 if a.localization else 450):
   for j in range(20):
    t=frame*.1+j*.005;ns=epoch+int(t*1e9);m=Imu();m.header.frame_id='trial_slam_imu_159';m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9);accel=kinematics([t])[1][0];m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,accel);imu.publish(m)
   arr=np.zeros(6000,dtype=dt);times=frame*.1+np.linspace(0,.095,len(arr));p=room[rng.integers(0,len(room),len(arr))]-kinematics(times)[0]
   for i,k in enumerate(['x','y','z']):arr[k]=p[:,i]
   arr['intensity']=10.;arr['timestamp']=epoch+times*1e9;header=Header(frame_id='livox_192_168_1_159');header.stamp.sec,header.stamp.nanosec=divmod(epoch+int(frame*.1e9),10**9);cloud.publish(point_cloud2.create_cloud(header,fields,arr))
   while time.monotonic()<clock+(frame+1)*.05:rclpy.spin_once(n,timeout_sec=.001)
   if proc.poll() is not None:raise RuntimeError('Premature backend exit')
  deadline=time.monotonic()+90
  while proc.poll() is None and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
  if proc.poll()!=0:raise RuntimeError('Backend export/exit failed')
  if a.localization:
   assert len(samples)>20, 'No local odometry'
   assert not any(c=='trial_odom' for p,c in transforms), 'Frontend conflicts with AMCL map->odom'
   assert ('trial_odom','trial_slam_imu_159') in transforms, 'Local TF missing'
   report={'localization_tf_ownership_passed':True,'tf_messages':len(transforms)}
  else:report=verify(run/'optimized')
  report.update(synthetic_only=True,hardware_started=False,odom_messages=len(samples),production_loop_thresholds=True)
  (run/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print(run)
 finally:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=60)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
