#!/usr/bin/env python3
"""Training consistency preview. Independent validation is still mandatory."""
import json,pathlib,sys,numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
from calibrate import capture_data,initial_transform,evaluate
s=json.loads((ROOT/'双雷达标定工具/config/settings.json').read_text());trials=[];clouds={}
for scene in ['A','B']:
 _,clouds[scene]=capture_data(ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02'/scene,s)
 for yaw in (0,-2,2):
  t=initial_transform();t[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@t[:3,:3]
  t,h=reg.fit(clouds[scene]['front_left'],clouds[scene]['rear_right'],t)
  trials.append(dict(scene=scene,initial_yaw_deg=yaw,transform=t.tolist(),information_eigenvalues=h[-1]['normalized_information_eigenvalues']))
  print('Fitted',scene,yaw,flush=True)
for trial in trials:
 t=np.array(trial['transform']);trial['scores']={n:evaluate(c['front_left'],c['rear_right'],t) for n,c in clouds.items()}
 trial['score']=float(np.mean([x['plane_rmse_m'] for x in trial['scores'].values()]))
best=min(trials,key=lambda x:x['score']);t=np.array(best['transform'])
shift=max(float(np.linalg.norm(np.array(x['transform'])[:3,3]-t[:3,3])) for x in trials)
angle=max(float(np.degrees(Rotation.from_matrix(np.array(x['transform'])[:3,:3]@t[:3,:3].T).magnitude())) for x in trials)
limits=s['limits'];out=dict(status='TRAINING_PREVIEW_NOT_DEPLOYED',translation_spread_m=shift,rotation_spread_deg=angle,
 training_consistency_passed=shift<=limits['max_trial_translation_spread_m'] and angle<=limits['max_trial_rotation_spread_deg'],
 best=best,trials=trials,independent_validation='Not collected; no final acceptance')
(ROOT/'标定稳定性复核_20261125/round2_AB_preview.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items() if k not in ['best','trials']},indent=2),flush=True)
