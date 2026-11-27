#!/usr/bin/env python3
"""Replay only the already-adapted scan and primary IMU; never start a hardware driver.
Use a separate ROS domain and output directory. Check every process exit and export.
"""
import argparse,os,signal,subprocess,time,datetime,json
from pathlib import Path
from verify_map_export import verify
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--rate',type=float,default=1.);p.add_argument('--domain',type=int,default=72);a=p.parse_args()
    if not (a.bag/'metadata.yaml').is_file() or not .1<=a.rate<=2. or a.domain in (0,31) or not 1<=a.domain<=100:raise ValueError('Use an existing bag, rate 0.1..2, isolated domain other than 0/31')
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    run=ROOT/'replay_runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');run.mkdir(parents=True);fast=None;play=None;forced=False
    executable=ROOT/'mapping_ws/install/lib/fast_lio_sam/fastlio_mapping'
    with (run/'fastlio.log').open('w') as log,(run/'bag.log').open('w') as baglog:
      try:
        fast=subprocess.Popen([str(executable),'--ros-args','--params-file',str(ROOT/'mapping_trial/config/lio.yaml'),'-p','use_sim_time:=true','-p','result_save/output_dir:='+str(run/'optimized')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        # Discover the real data subscriptions before playback; no fixed timing assumption.
        check=subprocess.run(['python3',str(ROOT/'scripts/wait_slam_subscriptions.py')],env=env,timeout=25,capture_output=True,text=True)
        (run/'discovery.log').write_text(check.stdout+check.stderr)
        if check.returncode or fast.poll() is not None:raise RuntimeError('SLAM did not become ready')
        play=subprocess.Popen(['ros2','bag','play',str(a.bag.resolve()),'--clock','100','--rate',str(a.rate),'--topics','/slam/points','/rear_right/imu'],env=env,stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
        while play.poll() is None:
            if fast.poll() is not None:raise RuntimeError('SLAM exited before playback finished')
            time.sleep(.2)
        if play.returncode:raise RuntimeError('Bag playback failed')
        fast.wait(timeout=90)
        if fast.returncode:raise RuntimeError('SLAM failed')
        report=verify(run/'optimized');report.update(bag=str(a.bag.resolve()),domain=a.domain,hardware_started=False)
        (run/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));print(run)
      finally:
        for child in [play,fast]:
            if child is not None and child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=60)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait();forced=True
        if forced:(run/'FORCED_CLEANUP').write_text('Do not accept this replay export.\n')
if __name__=='__main__':main()
