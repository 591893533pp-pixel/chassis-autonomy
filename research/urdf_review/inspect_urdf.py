from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np,json
from scipy.spatial.transform import Rotation as R
from collections import Counter
p=Path('/home/pc/下载/chassis_urdf/chassis_id_me');r=E.parse(p/'urdf/chassis_id_me.urdf').getroot();links={x.get('name'):x for x in r.findall('link')};js=r.findall('joint');child={j.find('child').get('link'):j for j in js};roots=set(links)-set(child);assert len(roots)==1 and len(child)==len(js)
def tr(e):
 t=np.eye(4)
 if e is not None:t[:3,3]=np.fromstring(e.get('xyz','0 0 0'),sep=' ');t[:3,:3]=R.from_euler('xyz',np.fromstring(e.get('rpy','0 0 0'),sep=' ')).as_matrix()
 return t
cache={next(iter(roots)):np.eye(4)}
def pose(n,stack=()):
 assert n not in stack
 if n not in cache:
  j=child[n];cache[n]=pose(j.find('parent').get('link'),stack+(n,))@tr(j.find('origin'))
 return cache[n]
for n in links:pose(n)
rows={}
for n in ['chassis_base','mid_360','mid_360_1','yis320','01','01_1','01_2','01_3']:
 t=pose(n);b=np.linalg.inv(pose('chassis_base'))@t
 rows[n]={'root_xyz_m':t[:3,3].tolist(),'root_rpy_deg':R.from_matrix(t[:3,:3]).as_euler('xyz',degrees=True).tolist(),'chassis_xyz_m':b[:3,3].tolist(),'chassis_rpy_deg':R.from_matrix(b[:3,:3]).as_euler('xyz',degrees=True).tolist()}
bad=[]
for n,l in links.items():
 e=l.find('inertial/inertia')
 if e is None:continue
 a={k:float(v) for k,v in e.attrib.items()};m=np.array([[a['ixx'],a['ixy'],a['ixz']],[a['ixy'],a['iyy'],a['iyz']],[a['ixz'],a['iyz'],a['izz']]]);ev=np.linalg.eigvalsh(m)
 if min(ev)<=0 or ev[2]>ev[0]+ev[1]+1e-12:bad.append(n)
report={'source':str(p),'root':list(roots),'links':len(links),'joints':dict(Counter(j.get('type') for j in js)),'mass_kg':sum(float(x.get('value')) for x in r.findall('.//mass')),'collision_count':len(r.findall('.//collision')),'invalid_inertias':bad,'poses_zero_joint_positions':rows,'note':'CAD link frames; not verified sensor optical frames, floor reference, or physical front direction.'}
Path(__file__).with_name('解析结果.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(rows,ensure_ascii=False,indent=2))
