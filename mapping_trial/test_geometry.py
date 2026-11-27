#!/usr/bin/env python3
"""Synthetic integration checks: exact transform chain and timestamp rejection."""
import json,pathlib,numpy as np
from scipy.spatial.transform import Rotation
from cloud_adapter import adapt_points
ROOT=pathlib.Path(__file__).resolve().parent
cfg=json.loads((ROOT/'config/extrinsics_active.json').read_text())
def pose(e):
 t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',e['rpy_deg'],degrees=True).as_matrix();t[:3,3]=e['xyz_m'];return t
rear=pose(cfg['sensors']['rear_right']);front=pose(cfg['sensors']['front_left']);rel=np.linalg.inv(rear)@front
arr=np.zeros(4,dtype=[('x','f8'),('y','f8'),('z','f8'),('intensity','f4'),('tag','u1'),('line','u1'),('timestamp','f8'),('sensor_id','u1')]);arr['sensor_id']=[0,1,0,1];arr['timestamp']=1.795598e18+np.array([30,10,20,0])*1e6;arr['intensity']=[1,2,3,4]
points=np.array([[2,1,.2],[-3,2,1],[2,0,-1],[1,2,3]])
expected=[]
for i,p in enumerate(points):
 m=rear if arr['sensor_id'][i]==0 else front
 b=m[:3,:3]@p+m[:3,3]
 for j,k in enumerate('xyz'):arr[k][i]=b[j]
 expected.append(p if arr['sensor_id'][i]==0 else rel[:3,:3]@p+rel[:3,3])
order=np.argsort(arr['timestamp']);actual,ns=adapt_points(arr,cfg['sensors']['rear_right'])
assert np.allclose(np.column_stack([actual[k] for k in 'xyz']),np.array(expected)[order],atol=1e-12)
assert np.array_equal(actual['timestamp'],arr['timestamp'][order])
assert np.array_equal(actual['intensity'],arr['intensity'][order])
assert ns==int(arr['timestamp'].min())
for why,bad in [('missing_sensor',arr[arr['sensor_id']==0].copy()),('long_scan',arr.copy())]:
 if why=='long_scan':bad['timestamp'][0]+=200e6
 try:adapt_points(bad,cfg['sensors']['rear_right'])
 except ValueError:pass
 else:raise AssertionError(why+' was accepted')
print('PASS: body-to-anchor cancellation, relative transform direction, unchanged fields/timestamps, sorting, both-sensor and scan-window rejection')
