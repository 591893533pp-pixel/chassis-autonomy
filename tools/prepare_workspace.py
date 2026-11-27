#!/usr/bin/env python3
"""Create ignored output directories and register imported CAD meshes. Never starts hardware."""
from pathlib import Path
import argparse,json
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cad',type=Path,default=ROOT/'assets/local/cad/chassis_id_me');a=p.parse_args()
 for rel in ['reports','mapping_trial/logs','control/logs','calibration/sessions','.runtime']:(ROOT/rel).mkdir(parents=True,exist_ok=True)
 cad=a.cad.resolve()
 if cad.is_dir():
  prefix=ROOT/'.runtime/model_prefix';index=prefix/'share/ament_index/resource_index/packages';index.mkdir(parents=True,exist_ok=True)
  (index/'chassis_id_me').write_text('')
  target=prefix/'share/chassis_id_me'
  if target.is_symlink():target.unlink()
  if target.exists():raise SystemExit('Refusing to replace existing CAD package: '+str(target))
  target.symlink_to(cad,target_is_directory=True)
 print(json.dumps({'root':str(ROOT),'cad_available':(cad/'meshes').is_dir(),'hardware_started':False},ensure_ascii=False))
if __name__=='__main__':main()
