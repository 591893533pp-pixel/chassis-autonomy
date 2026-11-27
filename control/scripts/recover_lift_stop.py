#!/usr/bin/env python3
"""Recover an orphaned PV lift by zeroing and disabling, never enabling."""
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
from lift_pv_remote import Bus, operation_enabled
from stationary_gate import StationaryGate
from lift_limits import MAX_POSITION_MM,within_position_fault_range

def verify_stop(bus,event,disabled=False):
    gate=StationaryGate(10)
    deadline=time.monotonic()+1.0
    while time.monotonic()<deadline:
        state=bus.snapshot()
        event('verification',**state)
        if disabled and operation_enabled(state['statusword']):
            raise RuntimeError('失能读回失败：驱动仍为OperationEnabled')
        if not within_position_fault_range(state['position_mm'],MAX_POSITION_MM):
            raise RuntimeError(f'位置超出-1–{MAX_POSITION_MM+1:g}mm；停止自动恢复')
        now=time.monotonic()
        if gate.observe(now,[state['velocity_counts_s']],now):return state
        time.sleep(.025)
    raise RuntimeError('未确认停车；不要启动遥控，请使用现场安全停止措施')

def recover(bus,event):
    before=bus.snapshot();event('before',**before)
    if before['mode']!=3:
        raise RuntimeError('不属于PV模式3，拒绝猜测控制方式；未写入')
    bus.zero();event('zero_ack')
    if bus.sdo(0x60ff,signed=True)!=0:
        raise RuntimeError('目标速度读回不为0；不继续接管')
    verify_stop(bus,event)
    bus.sdo(0x6040,0,2);event('disable_ack')
    bus.send(0,bytes([128,1]));event('preop_node1')
    after=verify_stop(bus,event,disabled=True)
    if bus.sdo(0x60ff,signed=True)!=0:
        raise RuntimeError('失能后目标速度读回不为0')
    event('completed',**after)
    return {'before':before,'after':after,'target_counts_s':0,'disabled':True}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hardware',action='store_true',help='只写零速和失能；不使能、不归零、不清故障')
    args=ap.parse_args()
    if not args.hardware:
        print('预览：加 --hardware 执行 node1 PV零速→停车确认→失能→回读；未访问CAN。');return 0
    lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    folder=ROOT/'logs'/('recover_lift_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);print('恢复日志：'+str(folder),flush=True)
    bus=None
    with (folder/'events.jsonl').open('w',buffering=1) as out:
        def event(name,**kw):
            out.write(json.dumps(dict(t=time.monotonic(),event=name,**kw),ensure_ascii=False)+'\n')
        try:
            bus=Bus(folder);result=recover(bus,event)
            (folder/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
            print(json.dumps(result,ensure_ascii=False,indent=2));return 0
        except Exception as e:
            event('failed',message=str(e));raise
        finally:
            if bus:bus.sock.close();bus.log.close()

if __name__=='__main__':
    try:raise SystemExit(main())
    except (RuntimeError,BlockingIOError) as exc:raise SystemExit(str(exc))
