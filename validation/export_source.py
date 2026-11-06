"""Export the constrained throwing actor and record an executable I/O contract."""
import argparse,hashlib,json,os,shutil,sys
from dataclasses import asdict
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'unitree_rl_mjlab')]
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import onnx
import onnxruntime as ort
import torch
import yaml
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner,RslRlVecEnvWrapper
from apple_throw.hardware_task import make_env,make_agent,nominal_environment,monitor
from apple_throw.hardware_limits import JOINTS,CONTRACT_SHA256,LIMIT_FILE,torque_envelope
from apple_throw.variants import restore_reward_state


@torch.inference_mode()
def main():
    p=argparse.ArgumentParser();p.add_argument('checkpoint',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();out=a.output.resolve();(out/'policy').mkdir(parents=True,exist_ok=True);(out/'validation').mkdir(exist_ok=True)
    saved=torch.load(a.checkpoint,map_location='cpu',weights_only=False)
    infos=saved.get('infos',{})
    if infos.get('task')!='g1_apple_throw_hardware_v10' or infos.get('hardware_contract_sha256')!=CONTRACT_SHA256:
        raise ValueError('Wrong task or hardware contract')
    cfg=nominal_environment(make_env(1));cfg.seed=390;cfg.observations['actor'].enable_corruption=False
    torch.manual_seed(390)
    env=ManagerBasedRlEnv(cfg,device='cuda:0');restore_reward_state(env,a.checkpoint)
    wrapper=RslRlVecEnvWrapper(env);runner=MjlabOnPolicyRunner(wrapper,asdict(make_agent()),device='cuda:0')
    runner.load(str(a.checkpoint),load_cfg={'actor':True},strict=True)
    runner.export_policy_to_onnx(str(out/'policy'))
    policy=runner.get_inference_policy(device='cuda:0');obs,_=wrapper.reset();term=monitor(env)
    names=list(term.target_names)
    # Contract list follows the SDK's named JointIndex enum, checked in source.
    sdk_names=list(JOINTS)
    mapping=[sdk_names.index(n) for n in names]
    defaults=term._offset[0].cpu().tolist()
    arrays={k:term.p[k].cpu().tolist() for k in ('kp','kd','lower','upper','velocity','effort','scale')}
    config=dict(task='g1_apple_throw_hardware_v10',checkpoint=a.checkpoint.name,
        actor_dim=126,action_dim=30,policy_dt=.02,servo_dt=.002,context_period_s=8.,
        policy_duration_s=4.1,release_threshold=.5,joint_names=names,sdk_joint_names=sdk_names,
        joint_ids_map=mapping,default_joint_pos=defaults,**arrays,
        obs_normalization='embedded in policy.onnx; do not normalize externally',
        quaternion_order='wxyz',gyro_frame='pelvis body frame',
        hand_controller='external Inspire hand; this package never publishes finger commands',
        release_socket='/tmp/g1_throw_release.sock',state_timeout_s=.1,
        preparation_s=2.,position_failure_tolerance_rad=.001745329,speed_failure_ratio=1.01,
        deployment_ready=False)
    for name,term_cfg in cfg.observations['actor'].terms.items():
        if term_cfg.scale is not None and term_cfg.scale!=1.:
            raise ValueError(f'Unexpected scaling in {name}: {term_cfg.scale}')
    (out/'policy/deploy.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    shutil.copy2(LIMIT_FILE,out/'policy/hardware_limits.json')
    original=term.process_actions
    captured={}
    def capture_action(actions):
        original(actions);captured['target']=term.desired[0].clone()
    term.process_actions=capture_action
    fixtures=[]
    for i in range(250):
        r=env.scene['robot'].data
        # Convert all joint vectors explicitly to motor/SDK order.
        q=np.zeros(29,dtype=np.float32);qd=q.copy()
        q[mapping]=r.joint_pos[0,term.target_ids].cpu().numpy()
        qd[mapping]=r.joint_vel[0,term.target_ids].cpu().numpy()
        current_q=r.joint_pos[0,term.target_ids].clone();current_v=r.joint_vel[0,term.target_ids].clone()
        row=dict(step=i,time_s=i*.02,q_sdk=q.tolist(),qd_sdk=qd.tolist(),
            quaternion_wxyz=r.root_link_quat_w[0].cpu().tolist(),gyro=obs['actor'][0,:3].cpu().tolist(),
            observation=obs['actor'][0].cpu().tolist())
        action=policy(obs);row['action']=action[0].cpu().tolist()
        obs,_,done,_=wrapper.step(action)
        target=captured['target'];lo,hi=torque_envelope(current_v,term.p['effort'],term.p['velocity'])
        torque=(term.p['kp']*(target-current_q)-term.p['kd']*current_v).clamp(lo,hi)
        row.update(target=target.cpu().tolist(),torque=torque.cpu().tolist())
        fixtures.append(row)
        if bool(done[0]):break
    env.close()
    model=onnx.load(out/'policy/policy.onnx');onnx.checker.check_model(model)
    for k,v in dict(task=config['task'],joint_names=','.join(names),hardware_contract_sha256=CONTRACT_SHA256,
                    observation_normalization='embedded',release_output_index='29').items():
        item=model.metadata_props.add();item.key=k;item.value=v
    onnx.save(model,out/'policy/policy.onnx')
    opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(out/'policy/policy.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
    error=0.
    for row in fixtures:
        action=session.run(None,{session.get_inputs()[0].name:np.array([row['observation']],np.float32)})[0][0]
        error=max(error,float(np.abs(action-row['action']).max()))
    if error>1e-4:raise ValueError(f'ONNX parity failed {error}')
    (out/'validation/golden_rollout.json').write_text(json.dumps(dict(rows=fixtures),separators=(',',':')))
    audit=json.loads((ROOT/'runs/apple_throw_hardware_v10/deployment_input_audit.json').read_text())
    audit.update(checkpoint=str(a.checkpoint),remaining=[
        'hardware commissioning and timing on target robot',
        'external Inspire hand release integration and latency calibration',
        'coupled ankle/waist and continuous/thermal characteristics',
        'self-collision unavailable to online encoder/IMU-only runtime'],robot_commands_sent=False)
    (out/'policy/observation_contract.json').write_text(json.dumps(audit,indent=2))
    report=dict(passed=True,checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
        onnx_sha256=hashlib.sha256((out/'policy/policy.onnx').read_bytes()).hexdigest(),
        hardware_contract_sha256=CONTRACT_SHA256,actor_dim=126,action_dim=30,
        golden_policy_steps=len(fixtures),pytorch_onnx_max_action_error=error,
        original_training_normalization_embedded=True,robot_commands_sent=False)
    (out/'validation/export_parity.json').write_text(json.dumps(report,indent=2))
    print('EXPORT_OK',json.dumps(report),flush=True)


if __name__=='__main__':main()
