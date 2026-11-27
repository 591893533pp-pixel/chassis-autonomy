#!/usr/bin/env python3
"""Audit Git-visible sources, maintained document links and candidate gates. Never imports runtime code."""
import ast,hashlib,json,re,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote
ROOT=Path(__file__).resolve().parents[1]
def source_files():
 out=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT)
 return sorted({ROOT/p.decode() for p in out.split(b'\0') if p})
def main():
 errors=[];files=source_files();total=0
 # A local ignored manifest can make in-place builds pass while fresh deployments fail.
 for rel in ['sensors/driver_workspace/src/livox_ros_driver2/package.xml',
             'mapping_ws/src/fast_lio_sam/package.xml',
             'build_support/livox_ros_driver2/package.xml']:
  p=ROOT/rel
  if p not in files or not p.is_file():errors.append(rel+': required ROS package manifest missing from Git source set')
 # Patterns are a bounded source audit, not a guarantee of complete secret detection.
 secret=re.compile(rb'-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|AKIA[0-9A-Z]{16}')
 for p in files:
  rel=str(p.relative_to(ROOT))
  if p.is_symlink():errors.append(rel+': symlink should not be packaged');continue
  if not p.is_file():errors.append(rel+': missing tracked source');continue
  b=p.read_bytes();total+=len(b)
  if len(b)>5_000_000:errors.append(rel+': source asset exceeds 5 MB; use asset manifest')
  if b.startswith(b'\x7fELF'):errors.append(rel+': compiled ELF in source set')
  if secret.search(b):errors.append(rel+': possible private credential')
  try:
   if p.suffix=='.py':ast.parse(b,filename=rel)
   if p.suffix=='.sh':
    v=subprocess.run(['bash','-n',str(p)],capture_output=True,text=True)
    if v.returncode:errors.append(rel+': '+v.stderr.strip())
  except (SyntaxError,UnicodeError) as e:errors.append(rel+': '+str(e))
 for d in ['config','mapping_trial/config','calibration/config']:
  for p in (ROOT/d).iterdir():
   try:
    if p.suffix=='.json':json.loads(p.read_text())
    elif p.suffix in ['.yaml','.yml']:
     import yaml;yaml.safe_load(p.read_text())
    elif p.suffix=='.xml':ET.parse(p)
   except Exception as e:errors.append(str(p.relative_to(ROOT))+': '+str(e))
 for p in (ROOT/'model').glob('*.urdf'):
  try:ET.parse(p)
  except ET.ParseError as e:errors.append(str(e))
 # Only actively maintained docs; dated vendor/research/historical docs deliberately retain original locators.
 maintained=[ROOT/'README.md',ROOT/'THIRD_PARTY_NOTICES.md',ROOT/'docs/STATUS.md',*list((ROOT/'docs/guides').glob('*.md')),*list((ROOT/'docs/engineering').glob('*.md')),ROOT/'docs/history/README.md']
 for p in maintained:
  for target in re.findall(r'\]\(([^)]+)\)',p.read_text()):
   target=unquote(target.split('#',1)[0].strip('<>'))
   if not target or '://' in target or target.startswith('mailto:'):continue
   if target.startswith('/') or not (p.parent/target).exists():errors.append(f'{p.relative_to(ROOT)}: unresolved/nonportable link {target}')
 acceptance=json.loads((ROOT/'config/acceptance.json').read_text())
 if any(x.get('passed') and not x.get('evidence') for x in acceptance['checks'].values()):errors.append('Acceptance passed without evidence')
 result={'passed':not errors,'files':len(files),'bytes':total,'hardware_started':False,'errors':errors}
 print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if not errors else 1
if __name__=='__main__':raise SystemExit(main())
