#!/usr/bin/env python3
import pathlib,json,sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[1];AUDIT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'双雷达标定工具'))
import registration as reg
from calibrate import initial_transform
out=AUDIT/'common_planes_candidate_v1';cand=json.loads((out/'candidate.json').read_text());anchor=reg.pose(json.loads((ROOT/'双雷达标定工具/config/initial_body_estimate.json').read_text())['sensors']['rear_right'])
def cloud(scene,role):
 p=ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02'/scene/(role+'.npz');z=np.load(p);p=np.concatenate([z['first'],z['second']]);p=p[np.isfinite(p).all(1)];r=np.linalg.norm(p,axis=1);return reg.voxel(p[(r>1)&(r<10)],.04)
fig,axes=plt.subplots(2,2,figsize=(12,10))
for col,T in enumerate([initial_transform(),np.array(cand['T_rear_right_from_front_left'])]):
 for role,M,color in [('front_left',anchor@T,'#087fce'),('rear_right',anchor,'#ed7518')]:
  p=reg.apply(cloud('C',role),M);q=p[(p[:,2]>.15)&(p[:,2]<2.5)];axes[0,col].scatter(q[:,0],q[:,1],s=.4,c=color,alpha=.5,label=role)
  q=p[abs(p[:,1])<3];axes[1,col].scatter(q[:,0],q[:,2],s=.4,c=color,alpha=.5,label=role)
 for row in [0,1]:
  ax=axes[row,col];ax.set_aspect('equal');ax.set_xlim(-7,7);ax.set_ylim((-7,7) if row==0 else (-.5,4));ax.grid(alpha=.2);ax.legend(markerscale=8);ax.set_xlabel('X (m)');ax.set_ylabel(['Y (m)','Z (m)'][row])
 axes[0,col].set_title(['Initial','Common-plane candidate - NOT DEPLOYED'][col])
fig.suptitle('C diagnostic only / fresh holdout D required / approximate body display frame');fig.tight_layout();fig.savefig(out/'validation_C.png',dpi=150);plt.close(fig)
planes=json.loads((AUDIT/'round2_dominant_planes_audit.json').read_text());prior=json.loads((AUDIT/'round2_AB_preview.json').read_text());colors=['#0072B2','#D55E00','#009E73']
fig=plt.figure(figsize=(13,11))
for row,scene in enumerate(['A','B']):
 T=np.array(next(x['transform'] for x in prior['trials'] if x['scene']==scene and x['initial_yaw_deg']==0))
 for col,role in enumerate(['front_left','rear_right']):
  p=cloud(scene,role);p=reg.apply(p,T) if col==0 else p;body=reg.apply(p,anchor)
  ax=fig.add_subplot(2,2,2*row+col+1,projection='3d');ax.scatter(*body[::12].T,s=.3,c='lightgray',alpha=.25)
  for i,pair in enumerate(cand['selection'][scene]):
   plane=planes[scene]['front_transformed' if col==0 else 'rear'][pair['front_plane' if col==0 else 'rear_plane']]
   q=body[abs(p@np.array(plane['normal'])-plane['distance_m'])<.025][::4]
   ax.scatter(*q.T,s=.6,color=colors[i],label=f'Paired surface {i+1}')
  ax.set_title(f'{scene} / {role}');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_zlabel('Z (m)');ax.set_xlim(-7,7);ax.set_ylim(-7,7);ax.set_zlim(-.5,4);ax.view_init(elev=24,azim=-55);ax.legend(markerscale=5,fontsize=8)
fig.suptitle('Training plane associations / colors pair surfaces within each row / approximate body frame');fig.tight_layout();fig.savefig(out/'training_plane_review.png',dpi=150)
