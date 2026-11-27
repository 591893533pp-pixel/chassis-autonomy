#!/usr/bin/env python3
"""Diagnostic only: reverse-fit sensitivity on A2/B. Never uses validation C to select parameters."""
import sys,json,pathlib,time
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
from calibrate import evaluate
SESSION=ROOT/'部署准备/lidar_commission_20261125/corner_20261125_01'
OUT=pathlib.Path(__file__).resolve().parent
old=json.loads((SESSION/'result/report.json').read_text())
def delta(a,b):
 return dict(translation_m=float(np.linalg.norm(a[:3,3]-b[:3,3])),rotation_deg=float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())))
def cloud(scene,role):
 d=np.load(SESSION/scene/(role+'.npz'));p=np.concatenate([d['first'],d['second']]);p=p[np.isfinite(p).all(1)]
 r=np.linalg.norm(p,axis=1);return reg.voxel(p[(r>1)&(r<10)],.04)
results={}
for scene in ('A2','B'):
 source=cloud(scene,'front_left');target=cloud(scene,'rear_right')
 forward=np.array(next(x['transform'] for x in old['trials'] if x['scene']==scene and x['initial_yaw_deg']==0))
 reverse,h=reg.fit(target,source,np.linalg.inv(forward));reverse=np.linalg.inv(reverse)
 results[scene]={'forward_transform':forward.tolist(),'reverse_transform':reverse.tolist(),'direction_disagreement':delta(forward,reverse),
  'forward_score':evaluate(source,target,forward),'reverse_score':evaluate(source,target,reverse)}
 print(scene,results[scene]['direction_disagreement'],flush=True)
results['across_scenes']={key:delta(np.array(results['A2'][key]),np.array(results['B'][key])) for key in ('forward_transform','reverse_transform')}
results['scope']='Diagnostic A2/B only; no change to acceptance limits, no validation C selection, not deployed.'
(OUT/'training_direction_audit.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results['across_scenes'],indent=2),flush=True)
