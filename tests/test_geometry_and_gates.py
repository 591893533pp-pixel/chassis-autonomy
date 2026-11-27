import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from footprint_check import check_path,collision,inflate,placed
from build_body_model import swept_bounds
class Geometry(unittest.TestCase):
 def setUp(self):
  self.grid=np.zeros((160,160),dtype=np.int8);self.res=.01;self.origin=[-.8,-.8];self.poly=np.array([[-.26,-.26],[.26,-.26],[.26,.26],[-.26,.26]])
 def corridor(self,width):
  g=self.grid.copy();y=np.arange(160)*.01-.8;g[(y< -width/2)|(y>=width/2),:]=100;return g
 def test_corridor_too_narrow(self):
  self.assertFalse(check_path(self.corridor(.55),self.res,self.origin,self.poly,[[-.3,0,0],[.3,0,0]])['clear'])
 def test_corridor_straight(self):
  self.assertTrue(check_path(self.corridor(.64),self.res,self.origin,self.poly,[[-.3,0,0],[.3,0,0]])['clear'])
 def test_rotation_sweep_hits_despite_clear_endpoints(self):
  g=self.corridor(.64);p=inflate(self.poly,.03)
  self.assertIsNone(collision(g,.01,self.origin,p,[0,0,0]));self.assertIsNone(collision(g,.01,self.origin,p,[0,0,np.pi/2]))
  self.assertFalse(check_path(g,.01,self.origin,self.poly,[[0,0,0],[0,0,np.pi/2]])['clear'])
 def test_unknown_inside_polygon(self):
  self.grid[80,80]=-1
  self.assertEqual(collision(self.grid,.01,self.origin,self.poly,[0,0,0]),'unknown')
 def test_thin_obstacle_between_waypoints(self):
  self.grid[:,80]=100
  self.assertFalse(check_path(self.grid,.01,self.origin,self.poly,[[-.4,0,0],[.4,0,0]])['clear'])
 def test_outside_map(self):self.assertEqual(collision(self.grid,.01,self.origin,self.poly,[1,0,0]),'outside_map')
 def test_analytic_wheel_envelope(self):
  from scipy.spatial.transform import Rotation
  v=np.array([[.1,.2,.3],[-.2,.1,-.1]]);axis=np.array([1.,2.,3.]);axis/=np.linalg.norm(axis);lo,hi=swept_bounds(v,axis)
  for a in np.linspace(0,2*np.pi,401):
   pts=v@Rotation.from_rotvec(axis*a).as_matrix().T
   self.assertTrue(np.all(pts>=lo-1e-12) and np.all(pts<=hi+1e-12))

from navigation_gate import velocity_command,require_acceptance
class NavigationGates(unittest.TestCase):
 def valid(self):return {'session':'test','boot_id':'boot','monotonic':10.,'velocity':[.03,.04,.04]}
 def test_vector_speed_not_axis_speed(self):
  d=self.valid();d['velocity']=[.05,.05,0.]
  with self.assertRaises(RuntimeError):velocity_command(d,10.1,'test','boot')
 def test_stale_and_wrong_session(self):
  with self.assertRaises(RuntimeError):velocity_command(self.valid(),10.21,'test','boot')
  with self.assertRaises(RuntimeError):velocity_command(self.valid(),10.1,'other','boot')
 def test_nan_refused(self):
  d=self.valid();d['velocity'][0]=float('nan')
  with self.assertRaises(RuntimeError):velocity_command(d,10.1,'test','boot')
 def test_not_commissioned(self):
  with self.assertRaisesRegex(RuntimeError,'现场验收'):require_acceptance()
 def test_valid_velocity(self):self.assertEqual(velocity_command(self.valid(),10.1,'test','boot'),(.03,.04,.04))

if __name__=='__main__':unittest.main()
