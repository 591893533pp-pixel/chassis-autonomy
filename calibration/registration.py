#!/usr/bin/env python3
"""Offline point-to-plane registration, adapted from deployed FAST-LIO-SAM tool."""
import argparse,json,pathlib
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
ROOT=pathlib.Path(__file__).resolve().parents[1]

def pose(e):
 t=np.eye(4);t[:3,:3]=R.from_euler('xyz',e['rpy_deg'],degrees=True).as_matrix();t[:3,3]=e['xyz_m'];return t

def apply(p,t):return p@t[:3,:3].T+t[:3,3]

def voxel(p,size):
 _,idx=np.unique(np.floor(p/size).astype(np.int32),axis=0,return_index=True);return p[idx]

def normals(p):
 tree=cKDTree(p);out=np.zeros_like(p);good=np.zeros(len(p),dtype=bool)
 for i in range(0,len(p),5000):
  d,idx=tree.query(p[i:i+5000],k=20,workers=2);neighbors=p[idx];center=neighbors.mean(1)
  q=neighbors-center[:,None,:];cov=np.einsum('nki,nkj->nij',q,q)/20
  ev,vec=np.linalg.eigh(cov);out[i:i+5000]=vec[:,:,0]
  good[i:i+5000]=(ev[:,0]/np.maximum(ev[:,1],1e-12)<.15)&(d[:,-1]<.45)
 return out,good

def prepare(p,size):
 p=voxel(p,size);n,g=normals(p);return p,n,g,cKDTree(p)

def fit(source,target,initial):
 transform=initial.copy();history=[]
 for size,radius in [(.12,.4),(.07,.25),(.04,.12)]:
  sp,sn,sg,_=prepare(source,size);tp,tn,tg,tree=prepare(target,size)
  for iteration in range(35):
   moved=apply(sp,transform);d,idx=tree.query(moved,workers=2)
   normal=tn[idx];alignment=np.abs(np.sum((sn@transform[:3,:3].T)*normal,axis=1))
   keep=(d<radius)&sg&tg[idx]&(alignment>.85)
   if keep.sum()<500:raise ValueError('Too few shared planar correspondences')
   x=moved[keep];n=normal[keep];res=np.sum((x-tp[idx[keep]])*n,axis=1)
   a=np.column_stack((np.cross(x,n),n));weights=1/np.sqrt(1+(res/.035)**2)
   aw=a*weights[:,None];b=-res*weights
   delta=np.linalg.lstsq(aw,b,rcond=None)[0]
   # Conservative trust region; converge instead of jumping to distant surfaces.
   scale=max(np.linalg.norm(delta[:3])/np.deg2rad(1),np.linalg.norm(delta[3:])/.04,1)
   delta/=scale;update=np.eye(4);update[:3,:3]=R.from_rotvec(delta[:3]).as_matrix();update[:3,3]=delta[3:]
   transform=update@transform
   if np.linalg.norm(delta[:3])<1e-5 and np.linalg.norm(delta[3:])<1e-4:break
  normalized=aw/np.maximum(np.linalg.norm(aw,axis=0),1e-12)
  eig=np.linalg.eigvalsh(normalized.T@normalized)
  history.append(dict(voxel_m=size,iterations=iteration+1,matches=int(keep.sum()),
      planar_residual_rmse_m=float(np.sqrt(np.mean(res**2))),normalized_information_eigenvalues=eig.tolist()))
 return transform,history

def metrics(source,target,transform):
 s=voxel(apply(source,transform),.04);t=voxel(target,.04);d,idx=cKDTree(t).query(s,workers=2)
 reverse=cKDTree(s).query(t,workers=2)[0];both=np.concatenate([d,reverse])
 n,g=normals(t);mask=(d<.15)&g[idx];plane=np.sum((s[mask]-t[idx[mask]])*n[idx[mask]],axis=1)
 return dict(bidirectional_within_10cm=float(np.mean(both<.1)),bidirectional_median_nearest_m=float(np.median(both)),
             matched_planar_points=int(mask.sum()),matched_planar_rmse_m=float(np.sqrt(np.mean(plane**2))),
             matched_plane_abs_p95_m=float(np.percentile(abs(plane),95)))
