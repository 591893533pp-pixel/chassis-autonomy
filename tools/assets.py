#!/usr/bin/env python3
"""Import/verify external assets using hashes, without network access or overwriting conflicts."""
import argparse,hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['list','import','verify']);p.add_argument('--group',default='all',choices=['all','cad','mapping','ptp','initial_lidar']);p.add_argument('--source-root',type=Path,help='Exported asset directory, with cad/, mapping/, ptp/ subdirectories');a=p.parse_args()
 rows=json.loads((ROOT/'assets/manifest.json').read_text())['assets'];rows=[x for x in rows if a.group=='all' or x['path'].split('/')[0]==a.group]
 if a.action=='list':
  print(json.dumps({'files':len(rows),'bytes':sum(x['bytes'] for x in rows),'groups':sorted({x['path'].split('/')[0] for x in rows})},indent=2));return
 errors=[]
 for x in rows:
  rel=Path(x['path'])
  if rel.is_absolute() or '..' in rel.parts:raise ValueError('Invalid manifest path')
  dest=ROOT/'assets/local'/rel
  try:
   if a.action=='import' and not dest.exists():
    src=a.source_root/rel if a.source_root else Path(x['original_path'])
    if src.stat().st_size!=x['bytes'] or sha(src)!=x['sha256']:raise ValueError('source hash/size mismatch')
    dest.parent.mkdir(parents=True,exist_ok=True);temp=dest.with_name(dest.name+'.importing');shutil.copy2(src,temp);temp.replace(dest)
   if dest.stat().st_size!=x['bytes'] or sha(dest)!=x['sha256']:raise ValueError('destination hash/size mismatch; preserved, not overwritten')
  except (OSError,ValueError) as e:errors.append({'asset':str(rel),'error':str(e)})
 print(json.dumps({'checked':len(rows),'errors':errors},ensure_ascii=False,indent=2))
 if errors:raise SystemExit(1)
if __name__=='__main__':main()
