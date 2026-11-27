#!/usr/bin/env python3
import pathlib,json,numpy as np
from fit_common_planes_candidate import solve_planes,delta
root=pathlib.Path(__file__).resolve().parent
preview=json.loads((root/'round2_AB_preview.json').read_text());fixed=json.loads((root/'common_planes_candidate_v1/candidate.json').read_text());allresults={}
for seed in [20261125,7,42]:
 path=root/('round2_dominant_planes_audit'+('' if seed==20261125 else '_seed'+str(seed))+'.json');planes=json.loads(path.read_text());scenes={};allpairs=[[],[],[],[]]
 for scene in ['A','B']:
  p=planes[scene];prior=np.array(next(x['transform'] for x in preview['trials'] if x['scene']==scene and x['initial_yaw_deg']==0));data=[[],[],[],[]]
  for pair in p['matched_planes']:
   a=p['front_transformed'][pair['front_plane']];b=p['rear'][pair['rear_plane']]
   if min(a['support'],b['support'])<5000 or min(a['tangent_std_m']+b['tangent_std_m'])<=.35:continue
   na=np.array(a['normal']);nb=np.array(b['normal']);sign=1 if na@nb>=0 else -1
   for dest,value in zip(data,[prior[:3,:3].T@na,nb*sign,a['distance_m']-na@prior[:3,3],b['distance_m']*sign]):dest.append(value)
  scenes[scene],_=solve_planes(*data)
  for dest,values in zip(allpairs,data):dest.extend(values)
 joint,_=solve_planes(*allpairs)
 allresults[str(seed)]={'scene_difference':delta(scenes['A'],scenes['B']),'joint_vs_frozen':delta(joint,np.array(fixed['T_rear_right_from_front_left'])),'planes_per_scene':len(allpairs[0])//2}
(root/'common_planes_candidate_v1/seed_stability.json').write_text(json.dumps(allresults,indent=2)+'\n');print(json.dumps(allresults,indent=2))
