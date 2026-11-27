#!/usr/bin/env python3
"""Experimental equal-plane calibration from training A/B only.
Plane associations are exploratory and need visual review plus a new holdout capture.
No device access or deployment. Existing formal result is never overwritten.
"""
import pathlib,json,sys,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];AUDIT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'双雷达标定工具'))
from calibrate import capture_data,evaluate,initial_transform,plot

def solve_planes(source_normals,target_normals,source_distances,target_distances):
 ns=np.asarray(source_normals,dtype=float);nt=np.asarray(target_normals,dtype=float)
 ds=np.asarray(source_distances,dtype=float);dt=np.asarray(target_distances,dtype=float)
 if len(ns)<3 or ns.shape!=nt.shape or ns.shape[1]!=3:raise ValueError('need >=3 paired plane normals')
 if not all(np.isfinite(x).all() for x in [ns,nt,ds,dt]):raise ValueError('non-finite plane data')
 if not np.allclose(np.linalg.norm(ns,axis=1),1,atol=1e-5) or not np.allclose(np.linalg.norm(nt,axis=1),1,atol=1e-5):raise ValueError('normals must be unit vectors')
 singular=np.linalg.svd(nt,compute_uv=False)
 if singular[-1]/singular[0]<.15:raise ValueError('plane directions cannot constrain translation')
 rot,_=Rotation.align_vectors(nt,ns);t=np.linalg.lstsq(nt,dt-ds,rcond=None)[0]
 T=np.eye(4);T[:3,:3]=rot.as_matrix();T[:3,3]=t
 normal_angles=np.degrees(np.arccos(np.clip(np.sum((ns@rot.as_matrix().T)*nt,axis=1),-1,1)))
 return T,dict(normal_angle_deg=normal_angles.tolist(),plane_distance_residual_m=(nt@t-(dt-ds)).tolist(),direction_singular_values=singular.tolist())

def delta(a,b):
 return dict(translation_m=float(np.linalg.norm(a[:3,3]-b[:3,3])),rotation_deg=float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())))

def main():
 out=AUDIT/'common_planes_candidate_v1'
 if out.exists():raise SystemExit('Refuse overwrite existing candidate')
 planes=json.loads((AUDIT/'round2_dominant_planes_audit.json').read_text());preview=json.loads((AUDIT/'round2_AB_preview.json').read_text())
 data={};scene_results={};all_ns=[];all_nt=[];all_ds=[];all_dt=[];chosen={}
 for scene in ['A','B']:
  p=planes[scene];prior=np.array(next(x['transform'] for x in preview['trials'] if x['scene']==scene and x['initial_yaw_deg']==0))
  ns=[];nt=[];ds=[];dt=[];chosen[scene]=[]
  for pair in p['matched_planes']:
   a=p['front_transformed'][pair['front_plane']];b=p['rear'][pair['rear_plane']]
   if min(a['support'],b['support'])<5000 or min(a['tangent_std_m']+b['tangent_std_m'])<=.35:continue
   na=np.array(a['normal']);nb=np.array(b['normal']);sign=1 if na@nb>=0 else -1
   ns.append(prior[:3,:3].T@na);nt.append(nb*sign);ds.append(a['distance_m']-na@prior[:3,3]);dt.append(b['distance_m']*sign)
   chosen[scene].append(dict(front_plane=pair['front_plane'],rear_plane=pair['rear_plane'],front_support=a['support'],rear_support=b['support']))
  T,e=solve_planes(ns,nt,ds,dt);scene_results[scene]=dict(transform=T.tolist(),residuals=e)
  all_ns.extend(ns);all_nt.extend(nt);all_ds.extend(ds);all_dt.extend(dt)
 T,e=solve_planes(all_ns,all_nt,all_ds,all_dt)
 # Candidate and selections are fixed by A/B before reading C.
 settings=json.loads((ROOT/'双雷达标定工具/config/settings.json').read_text());session=ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02'
 for scene in ['A','B','C']:_,data[scene]=capture_data(session/scene,settings)
 scores={s:evaluate(c['front_left'],c['rear_right'],T) for s,c in data.items()}
 spread=delta(np.array(scene_results['A']['transform']),np.array(scene_results['B']['transform']))
 out.mkdir();candidate=dict(status='EXPERIMENTAL_NOT_DEPLOYED_FRESH_VALIDATION_REQUIRED',T_rear_right_from_front_left=T.tolist(),
  convention='p_rear_right = R @ p_front_left + t; meters',training_scenes=['A','B'],diagnostic_validation='C already evaluated with original method; new holdout D required before acceptance',
  selection_rule='Matched broad planes, >=5000 retained planar points each sensor and both tangent standard deviations >0.35m; equal weight per plane; no forced right angles.',
  selection=chosen,training_scene_difference=spread,per_scene=scene_results,joint_plane_residuals=e,scores=scores,
  limits_unchanged=settings['limits'],checks_passed=False,relative_calibrated=False,calibrated=False,
  limitations=['Automatic plane correspondences require visual/physical review','No internal range/angle calibration','No absolute chassis anchor calibration','No runtime config modified'],
  script_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest())
 (out/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
 plot(data,['C'],initial_transform(),T,out)
 print(json.dumps({'status':candidate['status'],'training_difference':spread,'scores':scores},indent=2))
if __name__=='__main__':main()
