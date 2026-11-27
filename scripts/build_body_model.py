#!/usr/bin/env python3
"""Derive a conservative shape from the URDF tree, visual origins and STL vertices.
CAD -Y -> ROS +X, CAD +X -> ROS +Y is a candidate, not a measured body anchor.
Continuous joints use analytic full-revolution bounds, not a zero-angle snapshot.
"""
import argparse, json, struct, hashlib, shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
def transform(origin):
    m=np.eye(4)
    if origin is not None:
        m[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        m[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return m

def vertices(path):
    with path.open('rb') as f: header=f.read(84)
    count=struct.unpack('<I',header[80:84])[0]
    if path.stat().st_size != 84+50*count: raise ValueError('Expected binary STL: '+str(path))
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    return np.memmap(path,offset=84,dtype=dtype,mode='r',shape=(count,))['v'].reshape(-1,3)

def corners(lo,hi):
    return np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])

def swept_bounds(points,axis):
    axis=np.asarray(axis)/np.linalg.norm(axis)
    parallel=np.outer(points@axis,axis);perp=points-parallel
    cross=np.cross(axis,perp);amplitude=np.sqrt(perp**2+cross**2)
    return (parallel-amplitude).min(0),(parallel+amplitude).max(0)

def build(source,out):
    out.mkdir(parents=True,exist_ok=True)
    urdf=source/'urdf/chassis_id_me.urdf';robot=ET.parse(urdf).getroot()
    links={e.get('name'):e for e in robot.findall('link')}; joints=robot.findall('joint')
    bychild={j.find('child').get('link'):j for j in joints};roots=set(links)-set(bychild)
    if len(roots)!=1:raise ValueError('URDF must be a tree')
    root=next(iter(roots));cache={root:np.eye(4)}
    def pose(n):
        if n not in cache:
            j=bychild[n];cache[n]=pose(j.find('parent').get('link'))@transform(j.find('origin'))
        return cache[n]
    body_root=np.eye(4);body_root[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    body_root=body_root@np.linalg.inv(pose('chassis_base'))
    all_bounds=[];details=[];light=ET.Element('robot',name='chassis_body_preview')
    ET.SubElement(light,'link',name='chassis_model_base')
    for name,link in links.items():
        for vi,visual in enumerate(link.findall('visual')):
            mesh=visual.find('geometry/mesh')
            if mesh is None: raise ValueError('Non-mesh geometry requires explicit handling')
            meshpath=source/'meshes'/Path(mesh.get('filename')).name
            # A mesh's convex hull preserves all coordinate extrema under rigid transforms.
            raw=vertices(meshpath);ids=ConvexHull(raw).vertices;v=np.asarray(raw[ids],dtype=float)
            v*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            t=body_root@pose(name)@transform(visual.find('origin'))
            b=v@t[:3,:3].T+t[:3,3]
            ancestors=[];a=name
            while a in bychild:
                j=bychild[a]
                if j.get('type')!='fixed':ancestors.append(j)
                a=j.find('parent').get('link')
            if len(ancestors)>1:raise ValueError('Nested moving joints need a separate envelope calculation')
            if ancestors:
                j=ancestors[0]
                if j.get('type')!='continuous':raise ValueError('Unhandled moving joint')
                jt=body_root@pose(j.find('child').get('link'))
                local=(b-jt[:3,3])@jt[:3,:3]
                axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
                # Analytic extrema in body axes, about the joint origin.
                axis_body=jt[:3,:3]@axis
                lo,hi=swept_bounds(local@jt[:3,:3].T,axis_body);lo+=jt[:3,3];hi+=jt[:3,3]
            else:lo,hi=b.min(0),b.max(0)
            all_bounds.append(corners(lo,hi))
            details.append({'link':name,'min':lo.tolist(),'max':hi.tolist(),'full_rotation':bool(ancestors)})
            # Display a fast, conservative per-part proxy, identical to collision bounds.
            n=f'proxy_{name}_{vi}';l=ET.SubElement(light,'link',name=n)
            for tag in ['visual','collision']:
                e=ET.SubElement(l,tag);ET.SubElement(e,'origin',xyz=' '.join(map(str,(lo+hi)/2)),rpy='0 0 0')
                g=ET.SubElement(e,'geometry');ET.SubElement(g,'box',size=' '.join(map(str,hi-lo)))
                if tag=='visual':
                    mat=ET.SubElement(e,'material',name=n);ET.SubElement(mat,'color',rgba='0.15 0.55 0.85 0.7')
            j=ET.SubElement(light,'joint',name=n+'_fixed',type='fixed');ET.SubElement(j,'parent',link='chassis_model_base');ET.SubElement(j,'child',link=n)
    pts=np.concatenate(all_bounds);xy=pts[:,:2];footprint=xy[ConvexHull(xy).vertices]
    lo,hi=pts.min(0),pts.max(0)
    report={'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'body_anchor_verified':False,
            'frame':'chassis_model_base','cad_to_body_rotation_deg':[0,0,90],
            'body_root_matrix':body_root.tolist(),'min_m':lo.tolist(),'max_m':hi.tolist(),'extent_m':(hi-lo).tolist(),
            'footprint':footprint.tolist(),'padding_m':.03,'parts':details,
            'method':'Convex hull of per-part world AABBs, analytic continuous-joint sweeps; conservative CAD-only envelope',
            'limitations':['body-to-LiDAR anchor unverified','cables and current physical additions unmeasured','2D footprint does not certify vertical clearance']}
    (out/'body_geometry.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    ET.indent(light);ET.ElementTree(light).write(out/'chassis_proxy.urdf',encoding='utf-8',xml_declaration=True)
    # Original appearance, prefixed frames; no map->root static transform.
    full=ET.fromstring(ET.tostring(robot))
    for e in full.findall('link'):e.set('name','cad_'+e.get('name'))
    for j in full.findall('joint'):
        j.set('name','cad_'+j.get('name'))
        for tag in ['parent','child']:j.find(tag).set('link','cad_'+j.find(tag).get('link'))
    for mesh in full.findall('.//mesh'):
        mesh.set('filename','package://chassis_id_me/meshes/'+Path(mesh.get('filename')).name)
    ET.SubElement(full,'link',name='chassis_model_base')
    j=ET.SubElement(full,'joint',name='model_to_cad',type='fixed');ET.SubElement(j,'parent',link='chassis_model_base');ET.SubElement(j,'child',link='cad_'+root)
    ET.SubElement(j,'origin',xyz=' '.join(map(str,body_root[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(body_root[:3,:3]).as_euler('xyz'))))
    ET.indent(full);ET.ElementTree(full).write(out/'chassis_visual.urdf',encoding='utf-8',xml_declaration=True)
    print(json.dumps({k:report[k] for k in ['extent_m','min_m','max_m','footprint']},indent=2))
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=ROOT/'assets/local/cad/chassis_id_me');ap.add_argument('--output',type=Path,default=ROOT/'model');a=ap.parse_args();build(a.source,a.output)
