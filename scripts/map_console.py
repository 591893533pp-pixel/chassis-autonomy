#!/usr/bin/env python3
"""Loopback map console. Replaces optimized maps rather than accumulating stale scans.
Default read-only. --navigation --allow-goals exposes explicit goal/cancel buttons.
Still needs the separate accepted, START-gated hardware process to move.
"""
import argparse,base64,io,json,math,queue,secrets,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Path as RosPath,OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer,TransformListener
from footprint_check import inflate,collision
ROOT=Path(__file__).resolve().parents[1]
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--navigation',action='store_true');ap.add_argument('--allow-goals',action='store_true');ap.add_argument('--port',type=int,default=18766);ap.add_argument('--seconds',type=int,default=3600);a=ap.parse_args()
 if a.allow_goals and not a.navigation:raise ValueError('Goals require --navigation')
 if not 1024<=a.port<=65535 or not 10<=a.seconds<=3600:raise ValueError('Invalid port or duration')
 frame='map' if a.navigation else 'trial_map';g=json.loads((ROOT/'model/body_geometry.json').read_text());footprint=g['footprint'];token=secrets.token_urlsafe(32);lock=threading.Lock();jobs=queue.Queue();grid_state={}
 state={'frame':frame,'points':[],'path':[],'plan':[],'pose':None,'pose_seen':0.,'map_seen':0.,'grid':None,'footprint':footprint,'allow_goals':a.allow_goals,'goal_status':'未发送目标','anchor_verified':False}
 rclpy.init();n=rclpy.create_node('chassis_map_console');tf=Buffer();listener=TransformListener(tf,n);client=ActionClient(n,NavigateToPose,'/navigate_to_pose');goal_state={'pending':False,'handle':None,'cancel_pending':False}
 def cloud(m):
  if m.header.frame_id!=frame:return
  pts=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).reshape(-1,3);pts=pts[np.isfinite(pts).all(1)];pts=pts[::max(1,math.ceil(len(pts)/40000))]
  with lock:state['points']=np.round(pts,3).tolist();state['map_seen']=time.monotonic()
 def path(m,key):
  if m.header.frame_id!=frame:return
  pts=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
  with lock:state[key]=pts
 def grid(m):
  q=m.info.origin.orientation
  if m.header.frame_id!=frame or abs(q.x)+abs(q.y)+abs(q.z)>1e-8 or abs(q.w-1)>1e-6:return
  array=np.array(m.data,dtype=np.int8).reshape(m.info.height,m.info.width);pix=np.where(array<0,100,np.where(array>=65,15,225)).astype(np.uint8)[::-1];f=io.BytesIO();Image.fromarray(pix).save(f,format='PNG');origin=[m.info.origin.position.x,m.info.origin.position.y]
  with lock:
   state['grid']={'image':'data:image/png;base64,'+base64.b64encode(f.getvalue()).decode(),'origin':origin,'resolution':m.info.resolution,'width':m.info.width,'height':m.info.height};state['map_seen']=time.monotonic();grid_state.update(grid=array,origin=origin,resolution=m.info.resolution)
 qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 subs=[n.create_subscription(PointCloud2,'/lio_sam/mapping/cloud_global',cloud,1),n.create_subscription(RosPath,'/lio_sam/mapping/path',lambda m:path(m,'path'),1),n.create_subscription(RosPath,'/plan',lambda m:path(m,'plan'),1),n.create_subscription(OccupancyGrid,'/map',grid,qos)]
 def set_status(text):
  with lock:state['goal_status']=text
 def accepted(f):
  goal_state['pending']=False;h=f.result();goal_state['handle']=h if h.accepted else None
  if not h.accepted:set_status('目标被导航拒绝');return
  if goal_state['cancel_pending']:h.cancel_goal_async();set_status('已请求取消');return
  set_status('目标已接受；运动仍受独立手柄使能与保护控制')
  def result(f):goal_state['handle']=None;set_status('导航结束，状态码 '+str(f.result().status))
  h.get_result_async().add_done_callback(result)
 def tick():
  try:
   tr=tf.lookup_transform(frame,'base_link_trial',rclpy.time.Time());q=tr.transform.rotation;t=tr.transform.translation;stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
   if not -.3<=time.time()-stamp<.5:raise RuntimeError('TF stale')
   pose=[t.x,t.y,t.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
   with lock:state['pose']=pose;state['pose_seen']=time.monotonic()
  except Exception:pass
  while not jobs.empty():
   kind,data,reply=jobs.get()
   try:
    if not a.allow_goals:raise ValueError('当前是只读模式')
    if kind=='cancel':
     goal_state['cancel_pending']=True
     if goal_state['handle']:goal_state['handle'].cancel_goal_async()
     set_status('已请求取消；请观察车辆停车');reply.put({'ok':True});continue
    if goal_state['pending'] or goal_state['handle'] is not None:raise ValueError('请先取消或等待当前目标结束')
    values=[data[k] for k in ('x','y','yaw')]
    if not all(type(v) in (int,float) and math.isfinite(v) for v in values):raise ValueError('目标必须为有限坐标')
    with lock:
     if time.monotonic()-state['pose_seen']>.5 or not grid_state:raise ValueError('当前位置或导航地图未就绪')
     reason=collision(grid_state['grid'],grid_state['resolution'],grid_state['origin'],inflate(footprint,.03),values)
    if reason:raise ValueError('目标处完整车体轮廓不可用：'+reason)
    if not client.server_is_ready():raise ValueError('Nav2未就绪')
    msg=NavigateToPose.Goal();msg.pose.header.frame_id=frame;msg.pose.header.stamp=n.get_clock().now().to_msg();msg.pose.pose.position.x=float(values[0]);msg.pose.pose.position.y=float(values[1]);msg.pose.pose.orientation.z=math.sin(values[2]/2);msg.pose.pose.orientation.w=math.cos(values[2]/2)
    goal_state.update(pending=True,cancel_pending=False);client.send_goal_async(msg).add_done_callback(accepted);set_status('目标提交中');reply.put({'ok':True})
   except Exception as exc:reply.put({'ok':False,'error':str(exc)})
 n.create_timer(.1,tick)
 page=(ROOT/'scripts/map_console.html').read_text().replace('__TOKEN__',token)
 class Handler(BaseHTTPRequestHandler):
  def log_message(self,*_):pass
  def send(self,value,typ='application/json',status=200):
   body=value.encode() if isinstance(value,str) else json.dumps(value,allow_nan=False).encode();self.send_response(status);self.send_header('Content-Type',typ);self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Length',str(len(body)));self.end_headers()
   try:self.wfile.write(body)
   except (BrokenPipeError,ConnectionResetError):pass
  def do_GET(self):
   if self.path=='/':self.send(page,'text/html; charset=utf-8')
   elif self.path=='/data':
    with lock:d=dict(state);d['pose_age']=time.monotonic()-state['pose_seen'];d['map_age']=time.monotonic()-state['map_seen']
    self.send(d)
   else:self.send({'error':'Not found'},status=404)
  def do_POST(self):
   if self.headers.get('X-Chassis-Token')!=token or self.headers.get('Content-Type')!='application/json':self.send({'error':'Forbidden'},status=403);return
   try:
    length=int(self.headers.get('Content-Length','0'))
    if not 0<length<2048 or self.path not in ('/goal','/cancel'):raise ValueError('Invalid request')
    data=json.loads(self.rfile.read(length));reply=queue.Queue(maxsize=1);jobs.put((self.path[1:],data,reply));self.send(reply.get(timeout=5))
   except Exception as exc:self.send({'ok':False,'error':str(exc)},status=400)
 server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler);threading.Thread(target=server.serve_forever,daemon=True).start();deadline=time.monotonic()+a.seconds
 print(f'http://127.0.0.1:{a.port} mode='+('goals enabled' if a.allow_goals else 'read only'),flush=True)
 try:
  while rclpy.ok() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
 except KeyboardInterrupt:pass
 finally:
  goal_state['cancel_pending']=True
  stop_deadline=time.monotonic()+2.
  while goal_state['pending'] and time.monotonic()<stop_deadline:rclpy.spin_once(n,timeout_sec=.05)
  if goal_state['handle'] is not None:
   f=goal_state['handle'].cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=2.)
  server.shutdown();server.server_close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
