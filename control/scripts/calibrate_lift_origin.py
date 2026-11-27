#!/usr/bin/env python3
"""Read-only CAN sampling; optional explicit lower-stop software reference save."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'lift_reference'))
from lift_pv_remote import Bus
from lift_origin import ORIGIN_PATH, calibrate_samples


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--confirm-at-lower-stop',action='store_true',help='操作者已确认处于机械最低处；保存软件参考，不使能、不运动')
    args=ap.parse_args()
    lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=ROOT/'logs'/('lift_origin_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);print('日志：'+str(folder),flush=True)
    bus=None
    try:
        bus=Bus(folder);samples=[];started=time.monotonic()
        for _ in range(50):
            state=dict(elapsed_s=time.monotonic()-started,
                raw_counts=bus.sdo(0x6064,signed=True),velocity_counts_s=bus.sdo(0x606c,signed=True),
                statusword=bus.sdo(0x6041),mode=bus.sdo(0x6061,signed=True),
                error_code=bus.sdo(0x603f),target_velocity_counts_s=bus.sdo(0x60ff,signed=True))
            samples.append(state)
            with (folder/'samples.jsonl').open('a') as log:log.write(json.dumps(state)+'\n')
            # Fail before continuing if enabled, faulted, or a motion target exists.
            if state['statusword']&0x4f!=0x40 or state['error_code'] or state['target_velocity_counts_s']:
                raise RuntimeError('drive_not_disabled_idle; calibration refused')
            time.sleep(.025)
        result=calibrate_samples(samples)
        result['utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        result['evidence']=str(folder)
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        if args.confirm_at_lower_stop:
            if ORIGIN_PATH.exists():raise RuntimeError('existing origin: review/backup before recalibration')
            # Same directory, atomic replacement; owning CAN lock prevents live changes.
            tmp=ORIGIN_PATH.with_suffix('.tmp')
            with tmp.open('w') as out:
                out.write(json.dumps(result,indent=2)+'\n');out.flush();os.fsync(out.fileno())
            os.replace(tmp,ORIGIN_PATH)
            print('已保存软件下限参考：'+str(ORIGIN_PATH))
        else:print('仅采样验证，未保存参考；确认处于最低处后才可使用 --confirm-at-lower-stop')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:
        if bus:bus.sock.close();bus.log.close()

if __name__=='__main__':main()
