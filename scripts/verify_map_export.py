#!/usr/bin/env python3
import argparse,json,csv
from pathlib import Path
import numpy as np
from pcd_io import read_pcd

def verify(root):
    s=json.loads((root/'graph_summary.json').read_text());poses=list(csv.DictReader((root/'graph_poses.csv').open()))
    n=s['keyframes'];cloud=read_pcd(root/'KEY_FRAMES/global.pcd')
    frames=list((root/'KEY_FRAMES/scans').glob('*.pcd'))
    factors=(root/'loop_factors.csv').read_text().splitlines() if (root/'loop_factors.csv').exists() else []
    if not(s['sam_enabled'] and s['loop_enabled'] and s['final_update_completed']):raise ValueError('Backend did not run')
    if n<1 or len(poses)!=n or len(frames)!=n or len(factors)!=s['loop_factors'] or len(cloud)==0:raise ValueError('Incomplete export')
    if not np.isfinite([[float(v) for v in p.values()] for p in poses]).all():raise ValueError('Invalid poses')
    return {'export_valid':True,'keyframes':n,'points':len(cloud),'loop_factors':len(factors),'loop_observed':len(factors)>0,'note':'Zero loop factors means no loop closure was accepted; enabling the backend alone is not a loop test.'}
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('optimized',type=Path);a=ap.parse_args();print(json.dumps(verify(a.optimized),indent=2))
