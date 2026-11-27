"""Validate trusted live raw timing status; no hardware writes."""
import json,math,os,pathlib,stat,time
from timing_policy import IPS,POLICY,RAW_PATH
BOOT=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
def finite(v):return type(v) in (int,float) and math.isfinite(v)
def validate(d,now,boot=BOOT,generation=None):
    if not isinstance(d,dict) or d.get('schema')!=POLICY or d.get('boot_id')!=boot:raise RuntimeError('raw_identity')
    if not d.get('generation') or generation is not None and d['generation']!=generation:raise RuntimeError('raw_generation')
    if not finite(d.get('monotonic')) or not 0<=now-d['monotonic']<.15:raise RuntimeError('raw_status_stale')
    if d.get('ready') is not True or d.get('fault') is not None:raise RuntimeError('raw_not_ready:'+str(d.get('fault')))
    if d.get('capture_stats',{}).get('drops')!=0:raise RuntimeError('raw_capture_drops')
    for ip in IPS:
        for port in (56300,56400):
            s=d.get('streams',{}).get(f'{ip}:{port}',{})
            if not finite(s.get('seen')) or not 0<=now-s['seen']<.05:raise RuntimeError('raw_stream_stale:'+ip)
            if not finite(s.get('age_ns')) or abs(s['age_ns'])>5_000_000:raise RuntimeError('raw_age:'+ip)
    return d

def read(generation=None):
    try:
        with RAW_PATH.open() as f:
            meta=os.fstat(f.fileno())
            if not stat.S_ISREG(meta.st_mode) or meta.st_uid!=0 or meta.st_mode & 0o022:raise RuntimeError('raw_untrusted_file')
            d=json.load(f)
        return validate(d,time.monotonic(),generation=generation)
    except (OSError,ValueError,TypeError,AttributeError) as e:raise RuntimeError('raw_read:'+str(e)) from e
