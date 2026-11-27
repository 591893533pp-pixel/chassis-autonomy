#!/usr/bin/env python3
"""Conservative convex-footprint vs occupancy-grid and swept-path checks (metres)."""
import math
import numpy as np

def inflate(poly,padding):
    p=np.asarray(poly,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all() or padding<0:raise ValueError('Invalid polygon/padding')
    area=np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))
    if area<0:p=p[::-1]
    d=np.roll(p,-1,axis=0)-p;n=np.column_stack([d[:,1],-d[:,0]])
    n/=np.linalg.norm(n,axis=1)[:,None];b=np.sum(n*p,axis=1)+padding
    return np.array([np.linalg.solve(np.array([n[i-1],n[i]]),np.array([b[i-1],b[i]])) for i in range(len(p))])

def placed(poly,pose):
    x,y,a=pose;c,s=math.cos(a),math.sin(a)
    return np.asarray(poly)@np.array([[c,s],[-s,c]])+[x,y]

def intersects(a,b):
    for polygon in (a,b):
        edges=np.roll(polygon,-1,axis=0)-polygon
        for e in edges:
            axis=np.array([-e[1],e[0]]);pa=a@axis;pb=b@axis
            if pa.max()<pb.min()-1e-12 or pb.max()<pa.min()-1e-12:return False
    return True

def collision(grid,resolution,origin,poly,pose,threshold=65):
    if resolution<=0 or not np.isfinite([resolution,*origin,*pose]).all():raise ValueError('Invalid map/pose')
    grid=np.asarray(grid);p=placed(poly,pose);origin=np.asarray(origin)
    lo=np.floor((p.min(0)-origin)/resolution).astype(int);hi=np.floor((p.max(0)-origin)/resolution).astype(int)
    if (lo<0).any() or hi[0]>=grid.shape[1] or hi[1]>=grid.shape[0]:return 'outside_map'
    for y in range(lo[1],hi[1]+1):
        for x in range(lo[0],hi[0]+1):
            value=grid[y,x]
            if value>=0 and value<threshold:continue
            left,bottom=origin+np.array([x,y])*resolution
            square=np.array([[left,bottom],[left+resolution,bottom],[left+resolution,bottom+resolution],[left,bottom+resolution]])
            if intersects(p,square):return 'unknown' if value<0 else 'occupied'
    return None

def check_path(grid,resolution,origin,footprint,poses,padding=.03,step=.01):
    path=np.asarray(poses,dtype=float)
    if path.ndim!=2 or path.shape[1]!=3 or len(path)==0 or not np.isfinite(path).all() or step<=0:raise ValueError('Invalid path')
    # At most step metres of ANY footprint vertex movement between samples.
    # Inflate by that step as well so a collision between samples is not skipped.
    poly=inflate(footprint,padding+step);radius=np.linalg.norm(poly,axis=1).max();checked=0
    previous=path[0]
    for segment,current in enumerate(path):
        delta=current-previous;delta[2]=(delta[2]+math.pi)%(2*math.pi)-math.pi
        count=max(1,math.ceil((np.linalg.norm(delta[:2])+radius*abs(delta[2]))/step))
        for k in range(count+1):
            pose=previous+delta*(k/count);reason=collision(grid,resolution,origin,poly,pose);checked+=1
            if reason:return {'clear':False,'reason':reason,'segment':segment,'pose':pose.tolist(),'checked':checked}
        previous=current
    return {'clear':True,'checked':checked,'padding_m':padding,'sampling_envelope_m':step}
