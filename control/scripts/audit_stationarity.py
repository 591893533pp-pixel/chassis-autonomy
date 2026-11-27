#!/usr/bin/env python3
"""Read-only replay of the reported 20260930 idle-feedback interruptions.

No CAN or joystick access; prints JSON. Times are monotonic log times, not
control parameters. Replaying recorded feedback is not a hardware test.
"""
import argparse
import csv
import json
import statistics
from pathlib import Path
from stationary_gate import StationaryGate

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    wheels=list(csv.DictReader((args.run/'feedback.csv').open()))
    lift=[json.loads(s) for s in (args.run/'lift_feedback.jsonl').read_text().splitlines()]
    commands=[json.loads(s) for s in (args.run/'commands.jsonl').read_text().splitlines()]
    dt=[b['t']-a['t'] for a,b in zip(commands,commands[1:])]
    result={'source':str(args.run.resolve()),'hardware_started':False,
        'command_interval_ms':{'median':statistics.median(dt)*1000,'maximum':max(dt)*1000},
        'comparison':[], 'limitation':'Recorded feedback replay only; excludes first 300 ms of each window. No corrected physical trajectory is simulated.'}
    for begin,end in [(1014.3,1017.6),(1090,1180)]:
        gate=StationaryGate(.1,4,release_limit=.15)
        counts=dict(total=0,old_blocked_samples=0,new_blocked_samples=0)
        for row in wheels:
            t=float(row['monotonic_s'])
            if not begin<=t<=end:continue
            v=[float(row[f'wheel{i}_feedback_rad_s']) for i in range(1,5)]
            good=gate.observe(t,v,t,
                valid=row['feedback_valid']=='1' and 0<=float(row['feedback_age_ms'])<200
                      and all(float(row[f'wheel{i}_fault'])==0 for i in range(1,5)),
                zero_command=all(float(row[f'wheel{i}_cmd_rad_s'])==0 for i in range(1,5)))
            if t>begin+.3:
                counts['total']+=1
                counts['old_blocked_samples']+=int(max(map(abs,v))>=.1)
                counts['new_blocked_samples']+=int(not good)
        result['comparison'].append(dict(sensor='wheels',interval=[begin,end],**counts))
    gate=StationaryGate(10)
    counts=dict(total=0,old_blocked_samples=0,new_blocked_samples=0)
    for row in lift:
        t=row['t']
        if not 1090<=t<=1180:continue
        good=gate.observe(t,[row['velocity_counts_s']],t,zero_command=row['target_counts_s']==0)
        if t>1090.3:
            counts['total']+=1
            counts['old_blocked_samples']+=int(abs(row['velocity_counts_s'])>10)
            counts['new_blocked_samples']+=int(not good)
    result['comparison'].append(dict(sensor='lift',interval=[1090,1180],**counts))
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
