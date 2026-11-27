#!/usr/bin/env python3
"""Check every file of the package against manifest.json (sha256 / symlink target)."""
import hashlib, json, os, sys
from pathlib import Path
r = Path(__file__).resolve().parent; bad = []
def h(p):
    d = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): d.update(b)
    return d.hexdigest()
items = json.loads((r / 'manifest.json').read_text())
for x in items:
    p = r / x['path']
    ok = (p.is_symlink() and os.readlink(p) == x['link']) if 'link' in x else (p.is_file() and not p.is_symlink() and h(p) == x['sha256'])
    if not ok: bad.append(x['path'])
print('verify: %d entries, %d bad' % (len(items), len(bad)))
for b in bad[:50]: print('  BAD', b)
sys.exit(1 if bad else 0)
