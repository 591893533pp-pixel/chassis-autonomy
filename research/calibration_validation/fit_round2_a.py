#!/usr/bin/env python3
"""Single scene diagnostic candidate; never certification or deployment."""
import json,pathlib,sys,numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
from calibrate import capture_data,initial_transform,evaluate
s=json.loads((ROOT/'双雷达标定工具/config/settings.json').read_text())
p=ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02/A'
m,c=capture_data(p,s);source=c['front_left'];target=c['rear_right'];initial=initial_transform();trials=[]
for yaw in (0,-2,2):
 t=initial.copy();t[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@t[:3,:3]
 t,h=reg.fit(source,target,t);score=evaluate(source,target,t)
 trials.append(dict(initial_yaw_deg=yaw,transform=t.tolist(),score=score,information_eigenvalues=h[-1]['normalized_information_eigenvalues']))
 print('A',yaw,score,flush=True)
out={'status':'SINGLE_SCENE_ONLY_NOT_DEPLOYED','trials':trials,'note':'B independent training and C validation not yet captured. No parameter selected using C.'}
(ROOT/'标定稳定性复核_20261125/round2_A_diagnostic.json').write_text(json.dumps(out,indent=2)+'\n')
