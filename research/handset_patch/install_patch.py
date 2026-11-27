"""Install reviewed joystick files with hash preconditions and recoverable backups."""
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

source = Path(__file__).resolve().parent
app = Path('/home/pc/rk3588 link')
prepared = Path('/home/pc/chassis_deploy_20261123')
before = json.loads((source/'before_hashes.json').read_text())
after = json.loads((source/'after_hashes.json').read_text())
lock = open(f'/tmp/poke-remote-{os.getuid()}.lock', 'a')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
active = []
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name) == os.getpid():
        continue
    try:
        args = (entry/'cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if any(Path(arg.decode(errors='replace')).name in
           ('remote_session.py', 'remote_bridge', 'run_remote.py', 'cmd_vel_bridge.py')
           for arg in args if arg):
        active.append(entry.name)
if active:
    raise SystemExit('Control processes still active: '+','.join(active))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

targets = [app, prepared/'rootfs/home/pc/rk3588 link']
for rel, digest in after.items():
    assert sha(source/'files'/rel) == digest, rel
    compile((source/'files'/rel).read_text(), rel, 'exec')
    for root in targets:
        path = root/rel
        if rel in before:
            assert sha(path) == before[rel], f'Unexpected existing changes: {path}'
        else:
            assert not path.exists(), f'New file already exists: {path}'

stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
backup = app/'deployment_backups'/('handset_xinput_'+stamp)
backup.mkdir(parents=True)
for index, root in enumerate(targets):
    for rel in before:
        dest = backup/str(index)/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root/rel, dest)
shutil.copy2(prepared/'manifest.json', backup/'prepared_manifest.json')
written = []
try:
    for root in targets:
        for rel in after:
            dest = root/rel
            temp = dest.with_name(dest.name+'.handset_tmp')
            shutil.copy2(source/'files'/rel, temp)
            temp.replace(dest)
            written.append((root, rel))
            assert sha(dest) == after[rel]
    manifest = json.loads((prepared/'manifest.json').read_text())
    by_path = {item['path']: item for item in manifest}
    for rel in after:
        path = 'rootfs/home/pc/rk3588 link/'+rel
        if path in by_path:
            by_path[path]['sha256'] = after[rel]
        else:
            manifest.append(dict(path=path, sha256=after[rel]))
    (prepared/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
except BaseException:
    for root, rel in reversed(written):
        if rel in before:
            shutil.copy2(backup/str(targets.index(root))/rel, root/rel)
        else:
            (root/rel).unlink(missing_ok=True)
    shutil.copy2(backup/'prepared_manifest.json', prepared/'manifest.json')
    raise
receipt = dict(installed_at=stamp, backup=str(backup), files=after,
               targets=[str(root) for root in targets], motors_started=False)
(backup/'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
(source/'installed_receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(receipt, ensure_ascii=False, indent=2))
