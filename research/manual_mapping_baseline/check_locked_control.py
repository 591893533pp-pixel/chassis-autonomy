"""Stationary locked controller test. Never publishes permission to an unlocked controller."""
import datetime,json,pathlib,subprocess,sys,time,uuid
from motion_lease import publish
ROOT=pathlib.Path(__file__).resolve().parent
run=ROOT/'control_checks'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');run.mkdir(parents=True)
permit=run/'lease.json';session=str(uuid.uuid4());boot=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
proc=None
try:
    with (run/'controller.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,str(ROOT/'guarded_remote.py'),'--lock-motion','--seconds','10','--session-id',session,'--motion-enable-file',str(permit),'--log-dir',str(run/'control')],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+5
        while proc.poll() is None and time.monotonic()<deadline:
            publish(permit,session,boot);time.sleep(.04)
        expiry_started=time.monotonic()
        # Deliberately keep the last file in place to reproduce a dead publisher.
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=18)
        done=time.monotonic()
finally:
    permit.unlink(missing_ok=True)
    if proc and proc.poll() is None:proc.terminate();proc.wait(timeout=18)
rows=[json.loads(s) for s in (run/'control/commands.jsonl').read_text().splitlines()] if (run/'control/commands.jsonl').exists() else []
log=(run/'controller.log').read_text()
bridge=(run/'control/bridge.log').read_text() if (run/'control/bridge.log').exists() else ''
report={'mode':'STATIONARY_LOCKED_FAULT_INJECTION','commands':len(rows),'permission_seen':any(r.get('permitted') for r in rows),'all_commands_disabled_and_zero':bool(rows) and all(not r['enabled'] and all(r[k]==0 for k in ['vx','vy','yaw','lift_direction']) for r in rows),'lease_loss_detected':'Mapping health lease lost' in log,'controller_returncode':proc.returncode,'cleanup_duration_after_refresh_stopped_s':done-expiry_started,'bridge_exited':'Bridge exited' in bridge,'bridge_cleanup_failure':' FAILED' in bridge,'run':str(run)}
report['passed']=report['permission_seen'] and report['all_commands_disabled_and_zero'] and report['lease_loss_detected'] and report['bridge_exited'] and not report['bridge_cleanup_failure'] and proc.returncode==1
(run/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(0 if report['passed'] else 2)
