#!/usr/bin/env python3
"""Inspect dominant planar supports in training scenes only; no calibration acceptance."""
import pathlib,sys,json
import numpy as np
from scipy.optimize import linear_sum_assignment
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--session',default='corner_20261125_01',choices=['corner_20261125_01','corner_20261125_02']);ap.add_argument('--seed',type=int,default=20261125);args=ap.parse_args()
SESSION=ROOT/'部署准备/lidar_commission_20261125'/args.session
OUT=pathlib.Path(__file__).resolve().parent
rng=np.random.default_rng(args.seed)
def cloud(scene,role):
 z=np.load(SESSION/scene/(role+'.npz'));p=np.concatenate([z['first'],z['second']]);p=p[np.isfinite(p).all(1)];r=np.linalg.norm(p,axis=1)
 return reg.voxel(p[(r>1)&(r<10)],.04)
def extract(p):
 normals,good=reg.normals(p);rest=np.flatnonzero(good);planes=[]
 for _ in range(8):
  if len(rest)<1000:break
  seeds=rng.choice(rest,size=min(600,len(rest)),replace=False);samples=rng.choice(rest,size=min(4000,len(rest)),replace=False)
  n=normals[seeds];d=np.sum(n*p[seeds],1)
  residual=abs(p[samples]@n.T-d);counts=(residual<.025).sum(0);best=int(np.argmax(counts))
  keep=rest[abs(p[rest]@n[best]-d[best])<.025]
  if len(keep)<1000:break
  for _ in range(4):
   q=p[keep];center=q.mean(0);ev,v=np.linalg.eigh((q-center).T@(q-center)/len(q));normal=v[:,0];distance=normal@center
   keep=rest[abs(p[rest]@normal-distance)<.025]
  if distance<0:normal=-normal;distance=-distance
  q=p[keep];r=q@normal-distance
  planes.append(dict(normal=normal.tolist(),distance_m=float(distance),support=len(keep),rms_m=float(np.sqrt(np.mean(r*r))),center=q.mean(0).tolist(),tangent_std_m=np.sqrt(ev[1:]).tolist()))
  rest=rest[abs(p[rest]@normal-distance)>=.04]
 return planes
report=json.loads((ROOT/'标定稳定性复核_20261125/round2_AB_preview.json').read_text()) if args.session.endswith('_02') else json.loads((SESSION/'result/report.json').read_text());out={}
for scene in (('A','B') if args.session.endswith('_02') else ('A2','B')):
 T=np.array(next(t['transform'] for t in report['trials'] if t['scene']==scene and t['initial_yaw_deg']==0))
 a=extract(reg.apply(cloud(scene,'front_left'),T));b=extract(cloud(scene,'rear_right'))
 cost=np.ones((len(a),len(b)))*1e6;detail={}
 for i,x in enumerate(a):
  for j,y in enumerate(b):
   na=np.array(x['normal']);nb=np.array(y['normal']);sign=1 if na@nb>=0 else -1
   angle=float(np.degrees(np.arccos(np.clip(abs(na@nb),-1,1))));dist=float(x['distance_m']-sign*y['distance_m'])
   if angle<10 and abs(dist)<.2:
    cost[i,j]=angle/10+abs(dist)/.2;detail[i,j]=dict(front_plane=i,rear_plane=j,normal_angle_deg=angle,origin_plane_offset_difference_m=dist)
 pairs=[detail[i,j] for i,j in zip(*linear_sum_assignment(cost)) if (i,j) in detail]
 out[scene]=dict(front_transformed=a,rear=b,matched_planes=pairs)
 print(scene,json.dumps(pairs),flush=True)
(OUT/((('round2_dominant_planes_audit' if args.session.endswith('_02') else 'dominant_planes_audit')+('' if args.seed==20261125 else '_seed'+str(args.seed))+'.json'))).write_text(json.dumps(out,indent=2)+'\n')
