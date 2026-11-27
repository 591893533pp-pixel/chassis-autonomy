#!/usr/bin/env python3
import json,os,signal,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=root/'STATE.json'
if not p.exists():raise SystemExit('无本目录活动会话记录。')
s=json.loads(p.read_text())
if not s.get('running'):raise SystemExit('本目录会话已结束。')
pid=int(s['pid'])
try:
    start=Path(f'/proc/{pid}/stat').read_text().split()[21]
    cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
except FileNotFoundError:raise SystemExit('记录的会话进程已退出。')
if start!=s.get('start_ticks') or str(root/'scripts').encode() not in cmd:raise SystemExit('进程身份不符，不发送信号。')
if '--status' in sys.argv:print(json.dumps(s,ensure_ascii=False,indent=2))
else:os.kill(pid,signal.SIGINT);print('已请求正常停止；请看主终端的停车确认/保存结果。')
