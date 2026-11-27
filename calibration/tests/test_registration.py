"""Known-transform recovery and single-plane degeneracy, independent of live hardware."""
import pathlib,sys,unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import registration as reg
from calibrate import evaluate,label
class RegistrationTest(unittest.TestCase):
 def test_corner_known_transform(self):
  rng=np.random.default_rng(1125);n=6500
  # Three planes with sufficient extent and noise, sampled independently.
  def corner():
   a=rng.uniform(-2,2,(n,3));b=rng.uniform(-2,2,(n,3));c=rng.uniform(-2,2,(n,3))
   a[:,0]=2;b[:,1]=2;c[:,2]=-1
   return np.concatenate([a,b,c])+rng.normal(0,.002,(3*n,3))
  target=corner();t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',[1.4,-1.1,.8],degrees=True).as_matrix();t[:3,3]=[.028,-.019,.011]
  source=reg.apply(corner(),np.linalg.inv(t));actual,h=reg.fit(source,target,np.eye(4))
  self.assertLess(np.linalg.norm(actual[:3,3]-t[:3,3]),.005)
  self.assertLess(np.degrees(Rotation.from_matrix(actual[:3,:3]@t[:3,:3].T).magnitude()),.15)
  self.assertGreater(min(h[-1]['normalized_information_eigenvalues']),.015)
  self.assertLess(evaluate(source,target,actual)['plane_rmse_m'],.01)
 def test_single_plane_degenerate(self):
  rng=np.random.default_rng(12);p=rng.uniform(-2,2,(12000,3));p[:,2]=0
  t,h=reg.fit(p,p,np.eye(4))
  self.assertLess(min(h[-1]['normalized_information_eigenvalues']),.015)
 def test_label_rejects_traversal(self):
  with self.assertRaises(Exception):label('../A')
if __name__=='__main__':unittest.main()
