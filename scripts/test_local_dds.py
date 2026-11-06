#!/usr/bin/env python3
"""Native integration test with fake lowstate on lo/domain 77 only."""
import argparse,json,signal,socket,subprocess,tempfile,time
from pathlib import Path
import yaml


def test(root,fixture,control):
    with tempfile.TemporaryDirectory(prefix='g1_throw_offline_') as tmp:
        work=Path(tmp);(work/'policy').mkdir();(work/'validation').mkdir()
        cfg=yaml.safe_load((root/'policy/deploy.yaml').read_text())
        cfg['release_socket']=str(work/'release.sock')
        (work/'policy/deploy.yaml').write_text(yaml.safe_dump(cfg))
        (work/'policy/policy.onnx').symlink_to(root/'policy/policy.onnx')
        (work/'validation/golden_rollout.json').symlink_to(root/'validation/golden_rollout.json')
        receiver=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);receiver.bind(cfg['release_socket']);receiver.setblocking(False)
        stages=set();events=[];states=[];duration=12 if control else 6
        with (work/'fixture.log').open('w') as flog,(work/'controller.log').open('w') as clog:
            producer=subprocess.Popen([str(fixture),str(work),str(duration)],stdout=flog,stderr=subprocess.STDOUT)
            time.sleep(.8)
            runner=subprocess.Popen([str(root/'bin/x86_64/g1_throw'),'--root',str(work),'--network','lo','--dds-domain','77',
                                     '--control' if control else '--monitor'],stdout=clog,stderr=subprocess.STDOUT)
            try:
                deadline=time.monotonic()+20
                while producer.poll() is None and time.monotonic()<deadline:
                    if runner.poll() is not None:raise RuntimeError((work/'controller.log').read_text())
                    status=work/'status.json'
                    if status.exists():
                        try:d=json.loads(status.read_text());stages.add(d.get('state'));states.append(d)
                        except json.JSONDecodeError:pass
                    try:events.append(json.loads(receiver.recv(8192)))
                    except BlockingIOError:pass
                    time.sleep(.05)
                if producer.poll() is None:raise RuntimeError('Fixture did not finish')
            finally:
                runner.send_signal(signal.SIGINT);runner.wait(timeout=8)
                if producer.poll() is None:producer.terminate();producer.wait(timeout=5)
                receiver.close()
        log=(work/'fixture.log').read_text()
        metrics=json.loads(next(line for line in reversed(log.splitlines()) if line.startswith('{')))
        faults=[x.get('fault') for x in states if x.get('fault')]
        if control:
            passed=(stages>={'Passive','FixStand','Throw','Hold'} and len(events)==1 and
                    any(x.get('state')=='Hold' and x.get('trial_id')==1 for x in states) and
                    metrics['lowcmd_received']>100 and not faults)
        else:passed=metrics['lowcmd_received']==0 and stages=={'Passive'} and len(events)==0
        passed &= metrics['double_pd_count']==0 and not metrics['nonfinite']
        result=dict(passed=passed,interface='lo',dds_domain=77,control_mode=control,
                    stages=sorted(x for x in stages if x),events=events,transport=metrics,faults=faults,
                    real_robot_connected=False)
        if not passed:result['controller_log']=(work/'controller.log').read_text()
        return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True)
    a=p.parse_args();root=a.root.resolve();fixture=a.fixture.resolve()
    control=test(root,fixture,True);monitor=test(root,fixture,False)
    report=dict(passed=control['passed'] and monitor['passed'],control=control,monitor=monitor)
    (root/'validation/local_dds_test.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
