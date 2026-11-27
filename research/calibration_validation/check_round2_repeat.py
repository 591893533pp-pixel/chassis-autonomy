#!/usr/bin/env python3
"""Same-pose repeatability diagnostic, not independent scene validation."""
import pathlib,json,sys,numpy as np
from scipy.spatial.transform import Rotation
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
from calibrate import capture_data,initial_transform,evaluate
settings=json.loads((ROOT/'双雷达标定工具/config/settings.json').read_text());out=ROOT/'标定稳定性复核_20261125';prior=json.loads((out/'round2_AB_preview.json').read_text())
_,c=capture_data(ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02/B_repeat',settings)
t,h=reg.fit(c['front_left'],c['rear_right'],initial_transform())
def diff(t,b):return dict(translation_m=float(np.linalg.norm(t[:3,3]-b[:3,3])),rotation_deg=float(np.degrees(Rotation.from_matrix(t[:3,:3]@b[:3,:3].T).magnitude())))
comparisons={scene:diff(t,np.array(next(x['transform'] for x in prior['trials'] if x['scene']==scene and x['initial_yaw_deg']==0))) for scene in ['A','B']}
r=dict(status='REPEATABILITY_DIAGNOSTIC_NOT_DEPLOYED',transform=t.tolist(),comparisons=comparisons,score=evaluate(c['front_left'],c['rear_right'],t),note='B_repeat is the same physical pose as B. It is not independent validation and does not replace C.')
(out/'round2_B_repeat_diagnostic.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
