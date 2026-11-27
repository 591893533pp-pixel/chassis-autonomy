#!/usr/bin/env python3
"""Candidate perception bridge. No CAN. Uses raw LiDAR before the 1 m mapping crop.
All extrinsics are the frozen trial values; physical acceptance remains mandatory.
"""
import json,math,time,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer,TransformListener
ROOT=Path(__file__).resolve().parents[1]
def matrix(t):
    m=np.eye(4);q=t.rotation;m[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();v=t.translation;m[:3,3]=[v.x,v.y,v.z];return m
class Bridge(Node):
 def __init__(self):
    super().__init__('chassis_navigation_sensors');self.tf=Buffer();self.listener=TransformListener(self.tf,self)
    self.ex=json.loads((ROOT/'mapping_trial/config/extrinsics_active.json').read_text())['sensors'];self.geometry=json.loads((ROOT/'model/body_geometry.json').read_text())
    self.parts=[(np.array(p['min'])-.005,np.array(p['max'])+.005) for p in self.geometry['parts']]
    self.frames={n:'livox_'+v['ip'].replace('.','_') for n,v in self.ex.items()}
    self.cloudpub={n:self.create_publisher(PointCloud2,f'/chassis_nav/{n}/obstacles',10) for n in self.ex}
    self.scanpub=self.create_publisher(LaserScan,'/chassis_nav/localization_scan',10);self.odompub=self.create_publisher(Odometry,'/chassis_nav/odom',10)
    self.status=self.create_publisher(String,'/chassis_nav/perception_status',10);self.latest={};self.last_stamp={};self.previous=None;self.odom_seen=0.
    self.subs=[self.create_subscription(PointCloud2,f'/{n}/points',lambda m,n=n:self.cloud(m,n),qos_profile_sensor_data) for n in self.ex]
    self.subs.append(self.create_subscription(Odometry,'/Odometry',self.odom,100));self.create_timer(.1,self.report)
 def cloud(self,m,name):
    ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec;now=self.get_clock().now().nanoseconds
    if m.header.frame_id!=self.frames[name] or not -5_000_000<=now-ns<=200_000_000 or ns<=self.last_stamp.get(name,0):return
    xyz=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
    xyz=xyz[np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)>.1)]
    e=self.ex[name];rot=Rotation.from_euler('xyz',e['rpy_deg'],degrees=True).as_matrix();t=np.array(e['xyz_m']);body=xyz@rot.T+t
    mask=np.ones(len(body),dtype=bool)
    for lo,hi in self.parts:mask&=~np.all((body>=lo)&(body<=hi),axis=1)
    lo=self.geometry['min_m'][2]+.02;hi=self.geometry['max_m'][2]+.05
    mask&=(body[:,2]>=lo)&(body[:,2]<=hi)
    self.cloudpub[name].publish(point_cloud2.create_cloud_xyz32(m.header,xyz[mask].astype(np.float32)))
    self.latest[name]=(time.monotonic(),ns,body[mask]);self.last_stamp[name]=ns
    if len(self.latest)!=2:return
    values=list(self.latest.values())
    if max(v[1] for v in values)-min(v[1] for v in values)>30_000_000:return
    points=np.concatenate([v[2] for v in values]);ranges=np.full(720,np.inf,dtype=np.float32)
    distance=np.linalg.norm(points[:,:2],axis=1);bins=np.floor((np.arctan2(points[:,1],points[:,0])+math.pi)*720/(2*math.pi)).astype(int)%720
    valid=(distance>=.1)&(distance<=10.);np.minimum.at(ranges,bins[valid],distance[valid])
    scan=LaserScan();scan.header=m.header;scan.header.frame_id='base_link_trial';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.angle_max=scan.angle_min+719*scan.angle_increment;scan.range_min=.1;scan.range_max=10.;scan.scan_time=.1;scan.ranges=ranges.tolist();self.scanpub.publish(scan)
 def odom(self,m):
    if m.header.frame_id!='trial_odom' or m.child_frame_id!='trial_slam_imu_159':return
    try:tr=self.tf.lookup_transform(m.child_frame_id,'base_link_trial',rclpy.time.Time.from_msg(m.header.stamp)).transform
    except Exception:return
    p=m.pose.pose;q=p.orientation;pose=np.eye(4);pose[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();pose[:3,3]=[p.position.x,p.position.y,p.position.z];pose=pose@matrix(tr)
    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    out=Odometry();out.header=m.header;out.child_frame_id='base_link_trial';out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pose[:3,3]);quat=Rotation.from_matrix(pose[:3,:3]).as_quat();out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
    # Do not claim IMU covariance is the lever-arm transformed chassis covariance.
    for i in (0,7,14,21,28,35):out.pose.covariance[i]=.02
    if self.previous:
        old,ts=self.previous;dt=stamp-ts
        if dt<=0:return
        if dt<.3:
            v=pose[:3,:3].T@(pose[:3,3]-old[:3,3])/dt;w=Rotation.from_matrix(old[:3,:3].T@pose[:3,:3]).as_rotvec()/dt
            out.twist.twist.linear.x=float(v[0]);out.twist.twist.linear.y=float(v[1]);out.twist.twist.angular.z=float(w[2])
    self.previous=(pose,stamp);self.odom_seen=time.monotonic();self.odompub.publish(out)
 def report(self):
    now=time.monotonic();self.status.publish(String(data=json.dumps({'monotonic':now,'sources':{n:{'age':now-v[0],'obstacle_points':len(v[2])} for n,v in self.latest.items()},'odom_age':now-self.odom_seen,'body_anchor_verified':False,'nearfield_crop_m':.1,'scope':'COMMISSIONING_CANDIDATE'})))
def main():
 rclpy.init();n=Bridge()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
