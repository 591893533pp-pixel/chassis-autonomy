#!/usr/bin/env python3
"""Bounded, read-only ROS map preview over a loopback HTTP server."""
import collections
import json
import math
import pathlib
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ROOT = pathlib.Path(__file__).resolve().parent
PAGE = '''<!doctype html><meta charset="utf-8"><title>双雷达实时建图</title>
<style>body{margin:0;background:#101923;color:#e1eaf4;font:16px sans-serif}header{padding:18px 24px}h2{margin:0 0 10px}button{padding:7px 14px;margin-right:12px}#status{padding:10px 0;color:#ffd48a}canvas{display:block;width:100%;height:72vh}small{color:#a7b9cb}</style>
<header><h2>双雷达实时建图 · 俯视预览</h2><button onclick="follow=!follow">切换跟随 / 全图</button>缩放 <input id="zoom" type="range" min="5" max="80" value="25"><span id="status">连接中</span><br><small>青色：累计点云投影　橙色：轨迹　红色：当前 IMU 位置与方向。仅显示，不控制车辆；不是导航地图。</small></header><canvas id="map"></canvas>
<script>
const c=document.getElementById('map'),ctx=c.getContext('2d');let follow=true;
function draw(d){const box=c.getBoundingClientRect();c.width=box.width*devicePixelRatio;c.height=box.height*devicePixelRatio;ctx.scale(devicePixelRatio,devicePixelRatio);const w=box.width,h=box.height;
let center=d.pose?d.pose.slice(0,2):[0,0];if(!follow&&d.points.length){center=[0,1].map(i=>(Math.min(...d.points.map(p=>p[i]))+Math.max(...d.points.map(p=>p[i])))/2)}
const s=+document.getElementById('zoom').value;const xy=p=>[w/2+(p[0]-center[0])*s,h/2-(p[1]-center[1])*s];ctx.strokeStyle='#253646';ctx.lineWidth=1;
for(let i=-Math.ceil(w/s);i<=Math.ceil(w/s);i++){let x=xy([Math.floor(center[0])+i,0])[0];ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}
for(let i=-Math.ceil(h/s);i<=Math.ceil(h/s);i++){let y=xy([0,Math.floor(center[1])+i])[1];ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
ctx.fillStyle='#5ec4c7';for(const p of d.points){let a=xy(p);ctx.fillRect(a[0],a[1],2,2)}ctx.strokeStyle='#ffb454';ctx.lineWidth=2;ctx.beginPath();d.path.forEach((p,i)=>{let a=xy(p);i?ctx.lineTo(...a):ctx.moveTo(...a)});ctx.stroke();
if(d.pose){let a=xy(d.pose);ctx.save();ctx.translate(...a);ctx.rotate(-d.pose[3]);ctx.fillStyle='#ff6f69';ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-8,-6);ctx.lineTo(-8,6);ctx.closePath();ctx.fill();ctx.restore()}
ctx.fillStyle='#c3d4e4';ctx.fillText('网格 1 m · trial_map 坐标',15,h-18);
let live=d.running&&d.cloud_age!==null&&d.cloud_age<3;document.getElementById('status').textContent=(live?'实时更新':'已停止或数据未更新')+' | 点数 '+d.points.length+' | 点云距今 '+(d.cloud_age===null?'无':d.cloud_age.toFixed(1)+'s')+(d.failure?' | '+d.failure:'');}
async function tick(){try{const r=await fetch('/data',{cache:'no-store'});if(!r.ok)throw Error(r.status);draw(await r.json())}catch(e){document.getElementById('status').textContent='连接中断：画面不再更新'}setTimeout(tick,1000)}tick();
</script>'''

def main():
    rclpy.init()
    node = rclpy.create_node('read_only_mapping_preview')
    lock = threading.Lock()
    voxels = collections.OrderedDict()
    path = collections.deque(maxlen=3000)
    state = dict(pose=None, cloud_time=None, sampled=0.0, run=None)

    def cloud(msg):
        now = time.monotonic()
        if now-state['sampled'] < 1.0:
            return
        state['sampled'] = now
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z')).reshape(-1)
        pts = pts[::max(1, math.ceil(len(pts)/3000))]
        with lock:
            for p in pts:
                v = tuple(float(p[k]) for k in ('x', 'y', 'z'))
                if not all(math.isfinite(x) for x in v):
                    continue
                key = tuple(round(x/.12) for x in v)
                voxels[key] = tuple(round(x, 3) for x in v)
                if len(voxels) > 30000:
                    voxels.popitem(last=False)
            state['cloud_time'] = now

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v = [p.x, p.y, p.z, yaw]
        if not all(math.isfinite(x) for x in v):
            return
        with lock:
            state['pose'] = v
            if not path or math.dist(v[:3], path[-1]) > .03:
                path.append(v[:3])

    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(PointCloud2, '/cloud_registered', cloud, qos)
    node.create_subscription(Odometry, '/Odometry', odom, qos)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == '/':
                body, typ = PAGE.encode(), 'text/html; charset=utf-8'
            elif self.path == '/data':
                try:
                    current = json.loads((ROOT/'STATE.json').read_text())
                except (OSError, ValueError):
                    current = {}
                with lock:
                    if current.get('run') != state['run']:
                        voxels.clear(); path.clear()
                        state.update(run=current.get('run'), pose=None, cloud_time=None)
                        saved = pathlib.Path(current.get('run') or '.')
                        if not current.get('running') and saved.parent == ROOT/'runs':
                            try:
                                pts = np.load(saved/'accumulated_map.npz')['xyz']
                                for i, p in enumerate(pts[::max(1, math.ceil(len(pts)/30000))]):
                                    voxels[i] = p.tolist()
                                rows = json.loads((saved/'poses.json').read_text())
                                path.extend([v[1:4] for v in rows[::max(1, len(rows)//3000)]])
                            except (OSError, ValueError, KeyError):
                                pass
                    data = dict(points=list(voxels.values()), path=list(path), pose=state['pose'],
                                cloud_age=None if state['cloud_time'] is None else time.monotonic()-state['cloud_time'],
                                running=current.get('running', False), failure=current.get('failure'))
                body, typ = json.dumps(data, allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type', typ)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', 18765), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    deadline = time.monotonic()+3600
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        server.shutdown(); server.server_close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
