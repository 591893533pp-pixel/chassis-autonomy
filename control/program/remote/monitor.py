#!/usr/bin/env python3
"""Display latest feedback without opening CAN or sending commands."""
import argparse
import csv
import json
from pathlib import Path
import time
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('directory', nargs='?', default=str(Path(__file__).resolve().parents[1]/'logs/remote-latest'))
p.add_argument('--once', action='store_true')
a=p.parse_args();folder=Path(a.directory)
try:
 while True:
  print('\033[2J\033[H',end='')
  print('日志:',folder.resolve())
  try:
   with (folder/'feedback.csv').open() as f:
    header=f.readline().strip().split(',')
    # Read only the tail, so long sessions do not grow monitoring memory usage.
    f.seek(0,2);size=f.tell();f.seek(max(0,size-8192));lines=f.read().splitlines()
   row=next((r for r in reversed(list(csv.reader(lines))) if len(r)==len(header) and r[0]!=header[0]),None)
   if row:
    d=dict(zip(header,row));age=time.monotonic()-float(d['monotonic_s'])
    print(f"数据年龄 {age:.2f}s | 实机={d['hardware']} | 遥控使能={d['enabled']} | 周期={d['loop_ms']} ms")
    print('请求 vx/vy/wz:',*[d[k] for k in ('requested_vx','requested_vy','requested_wz')])
    print('实际下发 vx/vy/wz:',*[d[k] for k in ('applied_vx','applied_vy','applied_wz')])
    print('轮速推算 vx/vy/wz:',*[d[k] for k in ('feedback_vx','feedback_vy','feedback_wz')])
    print('轮反馈有效:',d['feedback_valid'],'最大年龄(ms):',d['feedback_age_ms'])
    for i in range(1,5):
     print(f"ID{i}: 指令={d[f'wheel{i}_cmd_rad_s']} 反馈={d[f'wheel{i}_feedback_rad_s']} rad/s 温度={d[f'wheel{i}_temp_C']}°C 故障={d[f'wheel{i}_fault']}")
    if 'body_control_active' in d:
     print('车体外环/直行保持:',d['body_control_active'],d['body_hold_active'],'横向误差(m):',d['cross_error_m'],'航向误差(rad):',d['segment_yaw_rad'],'修正 vy/wz:',d['cross_trim_m_s'],d['yaw_trim_rad_s'])
    print('轮速闭环启用:',d.get('wheel_balance_active','旧版本无此字段'))
    if 'wheel1_trim_rad_s' in d:
     print('ID1–4 补偿rad/s:',*[d[f'wheel{i}_trim_rad_s'] for i in range(1,5)])
    print('升降: mm/mm_s/target/status/error/age:',*[d[k] for k in ('lift_mm','lift_mm_s','lift_target_mm','lift_statusword','lift_error_cached','lift_feedback_age_s')])
    if age>1: print('*** 日志已停止更新；不能据旧日志判断设备当前状态 ***')
    if d['hardware']=='0': print('*** DRY RUN：零反馈是占位值，不是实测 ***')
   else: print('等待反馈记录')
   for name in ('events.jsonl','bridge.log'):
    with (folder/name).open('rb') as f:
     f.seek(0,2);size=f.tell();f.seek(max(0,size-4096));tail=f.read().decode('utf-8', errors='replace').splitlines()[-3:]
    print(name, '\n'.join(tail))
  except FileNotFoundError: print('还没有测试日志，请先启动遥控脚本。')
  if a.once: break
  time.sleep(.5)
except KeyboardInterrupt: pass
