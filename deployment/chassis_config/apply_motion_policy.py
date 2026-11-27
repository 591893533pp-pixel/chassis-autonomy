#!/usr/bin/env python3
"""Validate every consumer before writing generated policy; no hardware actions."""
import json,math,os,re,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def sub(pattern,repl,text,where):
    text,n=re.subn(pattern,repl,text)
    if n<1:raise ValueError(f'{where}: pattern not found: {pattern}')
    return text

def generate(root):
    p=json.loads((root/'config/chassis_motion_policy.json').read_text());a=p['active'];target=p['approved_targets']
    values=[a[k] for k in ('max_planar_speed','planar_acceleration','planar_deceleration','planar_jerk','wheel_limit_rad_s')]+[target['max_planar_speed']]
    if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in values):raise ValueError('nonfinite policy')
    if not (0<a['max_planar_speed']<=target['max_planar_speed']<=1.5 and a['planar_acceleration']==a['planar_deceleration'] and 0<a['planar_acceleration']<=1 and 0<a['planar_jerk']<=.5 and a['wheel_limit_rad_s']==25):raise ValueError('policy outside approved limits')
    v=a['max_planar_speed'];acc=a['planar_acceleration'];updates={}
    def data(rel):return json.loads((root/rel).read_text())
    def put(rel,obj):updates[root/rel]=json.dumps(obj,indent=2)+'\n'
    for prefix in ('RK3588con','rk_web/vendor'):
        rel=prefix+'/config/motion_limits.json';c=data(rel)
        c.update(chassis_max_linear_m_s=v,chassis_final_max_linear_m_s=target['max_planar_speed'],chassis_acceleration_m_s2=acc,chassis_deceleration_m_s2=acc,chassis_jerk_m_s3=a['planar_jerk']);put(rel,c)
    for rel in ('remote control/program/remote/bridge.cpp','RK3588con/program/bridge_r2.cpp'):
        s=(root/rel).read_text()
        s=sub(r'BodyControl body\([.\d]+\)',f'BodyControl body({acc})',s,rel)
        s=sub(r'std::hypot\(c.x,c.y\)>[.\d]+',f'std::hypot(c.x,c.y)>{target["max_planar_speed"]+1e-6:.6f}',s,rel)
        updates[root/rel]=s
    rel='navigation/config/nav2_params.yaml';s=(root/rel).read_text()
    for key,value in {'min_vel_x':-v,'min_vel_y':-v,'max_vel_x':v,'max_vel_y':v,'max_speed_xy':v,'acc_lim_x':acc,'acc_lim_y':acc,'decel_lim_x':-acc,'decel_lim_y':-acc}.items():s=sub(r'(?m)^(\s*'+key+r':) [-.\d]+$',rf'\g<1> {value}',s,rel)
    for key,value in {'max_velocity':v,'min_velocity':-v,'max_accel':acc,'max_decel':-acc}.items():s=sub(r'('+key+r':\n    - )[-.\d]+\n    - [-.\d]+',rf'\g<1>{value}\n    - {value}',s,rel)
    updates[root/rel]=s
    for rel,key in (('remote control/config/defaults.json','max_linear_speed'),('navigation/config/defaults.json','max_linear_m_s')):
        c=data(rel);c[key]=v;c['final_max_linear_m_s']=target['max_planar_speed'];put(rel,c)
    return updates

def main():
    updates=generate(ROOT)  # missing regex, invalid policy or unreadable file: write nothing
    staged=[]
    try:
        for path,text in updates.items():
            fd,name=tempfile.mkstemp(prefix='.policy_',dir=path.parent)
            staged.append((path,name))
            with os.fdopen(fd,'w') as out:out.write(text);out.flush();os.fsync(out.fileno())
            os.chmod(name,path.stat().st_mode&0o777)
        for path,name in staged:os.replace(name,path)
    finally:
        for _,name in staged:
            if os.path.exists(name):os.unlink(name)
    print('Generated validated motion policy; rebuild bridges before use.')
if __name__=='__main__':main()
