#!/usr/bin/env python3
"""Project final optimized keyframe scans; unknown remains unknown outside observed rays.
Requires an explicitly measured/estimated floor height in the optimized map frame.
Never fill the bounding box of a PCD with free space.
"""
import argparse,json,math,hashlib
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from pcd_io import read_pcd

def cells_between(start,end):
    x0,y0=map(int,start);x1,y1=map(int,end);dx=abs(x1-x0);dy=-abs(y1-y0)
    sx=1 if x0<x1 else -1;sy=1 if y0<y1 else -1;err=dx+dy
    while True:
        yield x0,y0
        if (x0,y0)==(x1,y1):break
        e2=2*err
        if e2>=dy:err+=dy;x0+=sx
        if e2<=dx:err+=dx;y0+=sy

def main():
    ap=argparse.ArgumentParser();ap.add_argument('optimized',type=Path);ap.add_argument('--lio-config',type=Path,required=True)
    ap.add_argument('--floor-z',type=float,required=True);ap.add_argument('--resolution',type=float,default=.025)
    ap.add_argument('--min-height',type=float,default=.02);ap.add_argument('--max-height',type=float,default=.38)
    ap.add_argument('--unobserved-radius',type=float,default=1.0);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if not np.isfinite([a.floor_z,a.resolution,a.min_height,a.max_height,a.unobserved_radius]).all() or not .01<=a.resolution<=.1 or not 0<=a.min_height<a.max_height or a.unobserved_radius<1.0:raise ValueError('Invalid height/resolution; this mapping source has a 1 m blind crop')
    if a.output.exists():raise ValueError('Output must be new')
    meta=json.loads((a.optimized/'graph_summary.json').read_text())
    if not meta.get('final_update_completed') or not meta.get('sam_enabled'):raise ValueError('Final graph update missing')
    p=yaml.safe_load(a.lio_config.read_text())['/**']['ros__parameters'];ri=np.array(p['mapping/extrinsic_R']).reshape(3,3);ti=np.array(p['mapping/extrinsic_T'])
    rows=np.loadtxt(a.optimized/'KEY_FRAMES/keyframe_pose.txt',ndmin=2)
    if rows.shape!=(meta['keyframes'],7) or not len(rows) or not np.isfinite(rows).all():raise ValueError('Pose count/data mismatch')
    observations=[];bounds=[]
    for stamp,x,y,z,roll,pitch,yaw in rows:
        scan=a.optimized/f'KEY_FRAMES/scans/{stamp:.9f}.pcd'
        xyz=read_pcd(scan);rot=Rotation.from_euler('xyz',[roll,pitch,yaw]).as_matrix();trans=np.array([x,y,z])
        xyz=(xyz@ri.T+ti)@rot.T+trans;origin=ti@rot.T+trans
        selected=xyz[(xyz[:,2]>=a.floor_z+a.min_height)&(xyz[:,2]<=a.floor_z+a.max_height),:2]
        if not len(selected):continue
        # Closest eligible return per angular bin, to avoid carving behind an obstacle.
        delta=selected-origin[:2];dist=np.linalg.norm(delta,axis=1);theta=np.arctan2(delta[:,1],delta[:,0]);bins=np.floor((theta+math.pi)*1440/(2*math.pi)).astype(int)
        order=np.argsort(dist);_,idx=np.unique(bins[order],return_index=True);selected=selected[order[idx]]
        observations.append((origin[:2],selected));bounds.extend([selected.min(0),selected.max(0),origin[:2]])
    if not observations:raise ValueError('No points in chosen height band; check floor and model')
    lo=np.floor(np.min(bounds,axis=0)/a.resolution)*a.resolution-.5;hi=np.max(bounds,axis=0)+.5
    size=np.ceil((hi-lo)/a.resolution).astype(int)
    if np.prod(size)>40_000_000:raise ValueError('Map too large; inspect units/trajectory')
    grid=np.full((size[1],size[0]),-1,dtype=np.int8);occupied=set()
    for origin,points in observations:
        for endpoint in points:
            delta=endpoint-origin;distance=np.linalg.norm(delta)
            end=np.floor((endpoint-lo)/a.resolution).astype(int)
            occupied.add(tuple(end))
            if distance<=a.unobserved_radius:continue
            first=origin+delta*(a.unobserved_radius/distance)
            start=np.floor((first-lo)/a.resolution).astype(int);end=np.floor((endpoint-lo)/a.resolution).astype(int)
            ray=list(cells_between(start,end))
            for x,y in ray[:-1]:grid[y,x]=0
            occupied.add(tuple(end))
    for x,y in occupied:grid[y,x]=100
    a.output.mkdir(parents=True)
    image=np.where(grid<0,205,np.where(grid>=65,0,254)).astype(np.uint8)[::-1]
    (a.output/'map.pgm').write_bytes(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+image.tobytes())
    (a.output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','mode':'trinary','resolution':a.resolution,'origin':[float(lo[0]),float(lo[1]),0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.25},sort_keys=False))
    np.savez_compressed(a.output/'grid.npz',grid=grid,resolution=a.resolution,origin=lo)
    report={'source':str(a.optimized.resolve()),'loop_factors':meta['loop_factors'],'keyframes':len(rows),'floor_z':a.floor_z,'height_band':[a.min_height,a.max_height],'unknown_cells':int((grid<0).sum()),'occupied_cells':int((grid==100).sum()),'free_cells':int((grid==0).sum()),'navigation_validated':False,'note':'Height-filtered planar ray projection; physical clearance/near-field/localization acceptance still required'}
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
