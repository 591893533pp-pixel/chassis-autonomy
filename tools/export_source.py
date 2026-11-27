#!/usr/bin/env python3
"""Export Git-visible source files (including reviewed uncommitted files) with checksums, excluding data/builds."""
import argparse,hashlib,json,tarfile
from pathlib import Path
from check_repository import ROOT,source_files,main as check

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=ROOT/'release/chassis-autonomy-source-20261127.tar.gz');a=p.parse_args()
 if check():raise SystemExit('Source checks failed')
 files=[x for x in source_files() if x.name!='SOURCE_MANIFEST.sha256']
 manifest=ROOT/'SOURCE_MANIFEST.sha256'
 manifest.write_text(''.join(hashlib.sha256(x.read_bytes()).hexdigest()+'  '+x.relative_to(ROOT).as_posix()+'\n' for x in files))
 a.output.parent.mkdir(parents=True,exist_ok=True)
 if a.output.resolve().is_relative_to(ROOT) and not a.output.resolve().is_relative_to(ROOT/'release'):raise SystemExit('Output inside repository must be under ignored release/')
 with tarfile.open(a.output,'w:gz') as archive:
  for x in [*files,manifest]:archive.add(x,arcname='chassis-autonomy/'+x.relative_to(ROOT).as_posix(),recursive=False)
 print(json.dumps({'archive':str(a.output),'source_files':len(files)+1,'sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'assets_included':False,'git_history_included':False},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
