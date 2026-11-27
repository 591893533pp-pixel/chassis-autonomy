#!/usr/bin/env python3
"""Copy a clean Git source snapshot to a NEW remote directory. No build, service or motor startup."""
import argparse,re,shlex,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--target',required=True,help='SSH user@host');p.add_argument('--destination',required=True,help='New absolute directory');p.add_argument('--identity',type=Path);p.add_argument('--apply',action='store_true');a=p.parse_args()
 if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+',a.target):p.error('Invalid target')
 if not re.fullmatch(r'/[A-Za-z0-9_./-]+',a.destination) or '..' in Path(a.destination).parts:p.error('Destination must be an absolute ASCII path without ..')
 print(f'Source -> {a.target}:{a.destination}; no services started.')
 if not a.apply:print('Dry run. Add --apply to transfer.');return
 if subprocess.check_output(['git','status','--porcelain','--untracked-files=all'],cwd=ROOT).strip():raise SystemExit('Commit reviewed changes before staging; dirty tree refused')
 ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8']
 if a.identity:ssh+=['-i',str(a.identity)]
 q=shlex.quote(a.destination)
 subprocess.run([*ssh,a.target,f'test ! -e {q} && mkdir -p -- {q}'],check=True)
 with tempfile.TemporaryFile() as f:
  subprocess.run(['git','archive','--format=tar','HEAD'],cwd=ROOT,stdout=f,check=True);f.seek(0)
  subprocess.run([*ssh,a.target,f'tar -xf - -C {q}'],stdin=f,check=True)
 print('Source staged. External data/models must be imported separately; no build or activation performed.')
if __name__=='__main__':main()
