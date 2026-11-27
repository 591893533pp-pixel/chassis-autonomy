#!/usr/bin/env python3
"""Run the user's exact software-PTP command, with read-only health queries."""
import argparse,fcntl,json,os,pathlib,re,signal,subprocess,time,uuid
ROOT=pathlib.Path(__file__).resolve().parents[1]
RUNTIME=pathlib.Path('/run/livox-ptp')
BOOT=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
COMMAND=['/usr/sbin/ptp4l','-i','enp85s0','-f',str(ROOT/'config/ptp4l.conf'),'-S','-m']
stop=False

def stop_signal(*_):
    global stop
    stop=True

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=pathlib.Path,default=ROOT/'config/ptp4l.conf')
    args=parser.parse_args()
    config=args.config.resolve(strict=True)
    COMMAND[COMMAND.index('-f')+1]=str(config)
    if os.geteuid()!=0:raise SystemExit('Administrator rights required')
    lock=open('/run/lock/livox-ptp.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    RUNTIME.mkdir(mode=0o755,exist_ok=True)
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    (ROOT/'logs/ptp').mkdir(parents=True,exist_ok=True)
    log=(ROOT/'logs/ptp/software-ptp4l.log').open('a',buffering=1)
    state_log=(ROOT/'logs/ptp/software-ptp-monitor.log').open('a',buffering=1)
    p=None;generation=None;port='UNKNOWN';timescale=None;port_seen=0.;good_since=None;restarts=0;retry_at=0.
    def health(ok,reason):
        now=time.monotonic()
        d=dict(boot_id=BOOT,monotonic=now,utc=time.time(),healthy=ok,reason=reason,
            mode='software',generation=generation,interface='enp85s0',port_state=port,
            port_age_s=now-port_seen if port_seen else None,ptp_timescale=timescale,
            utc_offset_applied_seconds=0,ptp_pid=p.pid if p else None,
            supervisor_pid=os.getpid(),restart_count=restarts,command=COMMAND)
        tmp=RUNTIME/'health.tmp';tmp.write_text(json.dumps(d,indent=2));tmp.chmod(0o644)
        os.replace(tmp,RUNTIME/'health.json')
    try:
        (ROOT/'logs/ptp/owner.json').write_text(json.dumps(dict(pid=os.getpid(),started=time.time())))
        health(False,'starting_software_ptp')
        while not stop:
            now=time.monotonic()
            if p is None and now>=retry_at:
                generation=uuid.uuid4().hex;port='UNKNOWN';timescale=None;good_since=None
                # Do not send SET commands or alter the user's /etc configuration.
                p=subprocess.Popen(COMMAND,stdout=log,stderr=subprocess.STDOUT)
                state_log.write(f'{time.time()} start {generation} pid={p.pid}\n')
            if p and p.poll() is not None:
                health(False,'software_ptp_exited');state_log.write(f'{time.time()} exit {p.returncode}\n')
                p=None;restarts+=1;good_since=None;retry_at=time.monotonic()+2
            if p:
                try:
                    r=subprocess.run(['/usr/sbin/pmc','-u','-b','0','-s','/var/run/ptp4l',
                        'GET PORT_DATA_SET','GET TIME_PROPERTIES_DATA_SET'],capture_output=True,text=True,timeout=1)
                    text=r.stdout
                    port='MASTER' if re.search(r'portState\s+MASTER\b',text) else 'NOT_MASTER'
                    m=re.search(r'ptpTimescale\s+([01])\b',text)
                    timescale=int(m[1]) if m else None;port_seen=time.monotonic()
                    if not (ROOT/'logs/ptp/software-ptp-properties.txt').exists() or port=='MASTER':
                        (ROOT/'logs/ptp/software-ptp-properties.txt').write_text(text)
                except subprocess.TimeoutExpired:port='UNRESPONSIVE';timescale=None
            now=time.monotonic()
            reason='ok'
            if not p or p.poll() is not None:reason='software_ptp_not_running'
            elif port!='MASTER':reason='software_ptp_not_master'
            elif timescale!=0:reason='unexpected_software_timescale'
            if reason=='ok':
                if good_since is None:good_since=now
                if now-good_since<10:reason='warming_up'
            else:good_since=None
            health(reason=='ok',reason);time.sleep(.25)
    finally:
        health(False,'stopped')
        if p and p.poll() is None:
            p.terminate()
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        log.close();state_log.close()
if __name__=='__main__':main()
