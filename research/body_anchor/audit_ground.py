#!/usr/bin/env python3
"""Offline ground-plane diagnostic. No absolute chassis calibration or deployment."""
import pathlib,sys,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
initial=json.loads((ROOT/'双雷达标定工具/config/initial_body_estimate.json').read_text())
candidate=ROOT/'标定稳定性复核_20261125/common_planes_candidate_v1/candidate.json'
T=np.array(json.loads(candidate.read_text())['T_rear_right_from_front_left'])
rear=reg.pose(initial['sensors']['rear_right']);poses={'rear_right':rear,'front_left':rear@T}
rng=np.random.default_rng(20261125)

def fit_floor(points,pose):
 # CAD is only used to find plausible low points; never force plane tilt/height.
 body=reg.apply(points,pose);mask=(body[:,2]>-.40)&(body[:,2]<.18)&(np.linalg.norm(body[:,:2],axis=1)>.7)
 idx=np.flatnonzero(mask)
 if len(idx)<500:raise ValueError('too few plausible ground points')
 q=body[idx];best=None
 sample=q[rng.choice(len(q),min(5000,len(q)),replace=False)]
 for _ in range(600):
  a,b,c=q[rng.choice(len(q),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
  if length<1e-8:continue
  n/=length
  if n[2]<0:n=-n
  if n[2]<np.cos(np.deg2rad(15)):continue
  d=n@a;count=int((abs(sample@n-d)<.02).sum())
  if best is None or count>best[0]:best=(count,n,d)
 if best is None:raise ValueError('no near-horizontal candidate')
 _,n,d=best
 for _ in range(5):
  selected=idx[abs(q@n-d)<.02];p=body[selected];center=p.mean(0);ev,v=np.linalg.eigh((p-center).T@(p-center)/len(p));n=v[:,0]
  if n[2]<0:n=-n
  d=n@center
 if len(p)<500 or np.sqrt(ev[1])<.3 or n[2]<np.cos(np.deg2rad(15)):raise ValueError('insufficient floor support/extent')
 ns=pose[:3,:3].T@n;ds=float(d-n@pose[:3,3]);height=-ds
 return dict(height_above_candidate_plane_m=height,sensor_z_tilt_to_plane_normal_deg=float(np.degrees(np.arccos(np.clip(ns[2],-1,1)))),normal_sensor=ns.tolist(),plane_distance_sensor_m=ds,normal_in_provisional_body=n.tolist(),plane_z_at_provisional_origin_m=float(d/n[2]),support=len(p),plane_rmse_m=float(np.sqrt(np.mean((p@n-d)**2))),tangent_std_m=np.sqrt(ev[1:]).tolist(),xy_bounds=[p[:,:2].min(0).tolist(),p[:,:2].max(0).tolist()]),body,selected

report={'status':'GROUND_DIAGNOSTIC_ONLY_BODY_FRAME_UNVERIFIED','candidate_sha256':hashlib.sha256(candidate.read_bytes()).hexdigest(),'assumptions':['Candidate low broad plane must be visually confirmed as floor','Plane normal is ground normal, not verified chassis up','No chassis x/y/yaw inferred; optical height is not base_link z'],'scenes':{}}
plot_data={}
for scene in ['A','B','C','D']:
 item={}
 for role,pose in poses.items():
  path=ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02'/scene/(role+'.npz');z=np.load(path)
  p=np.concatenate([z['first'],z['second']]);p=p[np.isfinite(p).all(1)];dist=np.linalg.norm(p,axis=1);p=reg.voxel(p[(dist>1)&(dist<6)],.04)
  try: result,body,selected=fit_floor(p,pose)
  except ValueError as exc:
   item[role]={'eligible':False,'reason':str(exc)};continue
  result['eligible']=True;result['source_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
  result['halves']=[]
  for key in ['first','second']:
   q=z[key];q=q[np.isfinite(q).all(1)];dist=np.linalg.norm(q,axis=1);q=reg.voxel(q[(dist>1)&(dist<6)],.04)
   try:
    part,_,_=fit_floor(q,pose);result['halves'].append({k:part[k] for k in ['height_above_candidate_plane_m','sensor_z_tilt_to_plane_normal_deg']})
   except ValueError as exc:
    result['halves'].append({'eligible':False,'reason':str(exc)})
  item[role]=result
  if scene=='D':plot_data[role]=(body,selected)
 if not all(item[role]['eligible'] for role in poses):
  report['scenes'][scene]=item;continue
 nr=np.array(item['rear_right']['normal_sensor']);nf=np.array(item['front_left']['normal_sensor'])
 item['cross_sensor_ground_normal_difference_deg']=float(np.degrees(np.arccos(np.clip(nr@(T[:3,:3]@nf),-1,1))))
 item['height_difference_constraint_residual_m']=float(item['front_left']['height_above_candidate_plane_m']-item['rear_right']['height_above_candidate_plane_m']-nr@T[:3,3])
 report['scenes'][scene]=item
 print(scene,json.dumps({k: {j:v[j] for j in ['height_above_candidate_plane_m','sensor_z_tilt_to_plane_normal_deg','support']} for k,v in item.items() if isinstance(v,dict)}),flush=True)
(OUT/'ground_audit.json').write_text(json.dumps(report,indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(12,9))
for col,(role,(p,idx)) in enumerate(plot_data.items()):
 for row,dims in enumerate([(0,1),(0,2)]):
  ax=axes[row,col];ax.scatter(p[::8,dims[0]],p[::8,dims[1]],s=.5,c='gray',alpha=.25);ax.scatter(p[idx,dims[0]],p[idx,dims[1]],s=1,c='#168f75');ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('X (m), provisional body');ax.set_ylabel(['Y (m)','Z (m)'][row]);ax.set_title(role)
  if row==1:ax.set_ylim(-.4,.5)
fig.suptitle('D ground candidate (green) / CAD only selects search region / no body calibration');fig.tight_layout();fig.savefig(OUT/'ground_review_D.png',dpi=140)
