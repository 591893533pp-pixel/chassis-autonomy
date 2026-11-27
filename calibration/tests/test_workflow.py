"""Complete multi-scene workflow on synthetic geometry; no ROS/hardware needed."""
import argparse,contextlib,io,json,pathlib,sys,tempfile,unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import calibrate as app
import registration as reg
class WorkflowTest(unittest.TestCase):
 def test_three_independent_static_scenes(self):
  rng=np.random.default_rng(355);settings=app.read(app.ROOT/'config/settings.json');initial=app.initial_transform()
  change=np.eye(4);change[:3,:3]=Rotation.from_euler('xyz',[.6,-.5,.2],degrees=True).as_matrix();change[:3,3]=[.006,-.004,.003];truth=change@initial
  with tempfile.TemporaryDirectory() as td:
   root=pathlib.Path(td)
   for i,label in enumerate(['A','B','C']):
    folder=root/label;folder.mkdir();scene=np.eye(4);scene[:3,:3]=Rotation.from_euler('z',i*18,degrees=True).as_matrix();scene[:3,3]=[i*.2,i*.1,0]
    def sample():
     n=4500;a=rng.uniform(-2,2,(n,3));b=rng.uniform(-2,2,(n,3));c=rng.uniform(-2,2,(n,3));a[:,0]=2.5;b[:,1]=2.5;c[:,2]=-1.3
     return reg.apply(np.concatenate([a,b,c])+rng.normal(0,.002,(3*n,3)),scene)
    for role in ['front_left','rear_right']:
     halves=[sample(),sample()]
     if role=='front_left':halves=[reg.apply(p,np.linalg.inv(truth)) for p in halves]
     np.savez_compressed(folder/(role+'.npz'),first=halves[0],second=halves[1])
    app.write(folder/'metadata.json',{'health_ok':True,'sensors':settings['sensors'],'settings_sha256':app.signature(app.ROOT/'config/settings.json'),'pose_note':f'SYNTHETIC scene {i}','capture_id':f'synthetic-{i}'})
   args=argparse.Namespace(session=str(root),train=['A','B'],validation=['C'])
   with contextlib.redirect_stdout(io.StringIO()):code=app.solve(args)
   report=app.read(root/'result/report.json')
   self.assertEqual(code,0,report['reasons'])
   t=np.array(report['T_rear_right_from_front_left'])
   self.assertLess(np.linalg.norm(t[:3,3]-truth[:3,3]),.01)
   self.assertLess(np.degrees(Rotation.from_matrix(t[:3,:3]@truth[:3,:3].T).magnitude()),.3)
   self.assertFalse(app.read(root/'result/body_candidate_NOT_DEPLOYED.json')['calibrated'])
   self.assertTrue((root/'result/validation_C.png').is_file())
   with self.assertRaisesRegex(ValueError,'已存在'):app.solve(args)
 def test_train_validation_leak_rejected(self):
  with self.assertRaisesRegex(ValueError,'不同'):
   app.solve(argparse.Namespace(session='/tmp/unused-calibration-test',train=['A','B'],validation=['A']))
if __name__=='__main__':unittest.main()
