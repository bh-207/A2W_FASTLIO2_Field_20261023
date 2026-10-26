"""Real-data sensitivity experiment, NOT a valid or proposed initializer.

Frozen rejected parameters and training p/v are replayed with one altered mean
component. Never produces a candidate, filter commit, corrected map, or truth.
"""
import csv,json,argparse,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent/'joint_init'))
from joint_cli import propagate_state


def run(base,output):
    base,output=Path(base),Path(output)
    if output.exists():raise ValueError('new counterfactual output required')
    output.mkdir(parents=True)
    old=base.parent/'field_frontend_z_init_20261024_001'
    q=np.load(old/'joint_motion_150_215_001/lidar_poses.npz');c=json.loads((old/'candidate_motion_final.json').read_text())
    e=json.loads((base/'old_reference_eiv_diagnostic.json').read_text());a=json.loads((old/'candidate_motion_first_error_audit.json').read_text())
    origin=int(q['stamp_ns'][0]);t=(q['stamp_ns']-origin)*1e-9;last=np.flatnonzero(q['stamp_ns']==c['training_last_stamp_ns'])[0]
    rows=list(csv.DictReader((old/'input_audit/imu_raw.csv').open()));it=(np.array([int(r['header_ns'])for r in rows],np.int64)-origin)*1e-9
    acc=np.array([[float(r[k])for k in ['ax','ay','az']]for r in rows]);gyro=np.array([[float(r[k])for k in ['gx','gy','gz']]for r in rows])
    state=dict(t=t[last],p=np.array(a['training_p_WI']),v=np.array(a['training_v_estimated']),R=q['R'][last].copy(),g=np.array(c['g_W']),ba=np.array(c['ba_I']),bg=np.array(c['bg_I']),cov=np.zeros((18,18)))
    ols=np.array(e['eiv_pose_diagnostic']['unweighted_s2_g_ba']);pull=np.array(e['eiv_pose_diagnostic']['correction'])[-1,3:]
    variants={'original_frozen':{},'only_commit_R_conditional_linear_pull':{'R':state['R']@Rotation.from_rotvec(pull).as_matrix()},
              'only_g_unweighted_seed':{'g':ols[:3]},'g_ba_unweighted_seed_group':{'g':ols[:3],'ba':ols[3:]}}
    results={}
    for name,change in variants.items():
        s=dict(state);s.update(change);pred=propagate_state(s,it,acc,gyro,t[last+1:],c['acc_scale'],.035,.002)
        errors=[]
        for k,pr in zip(range(last+1,len(t)),pred):
            truth=q['p'][k]-q['R'][k]@np.array([-.00862,.00914,.03916]);dp=pr['p']-truth
            errors.append(dict(seconds_from_train_end=float(t[k]-t[last]),position_error_m=float(np.linalg.norm(dp)),position_error_W=dp.tolist()))
        results[name]=dict(changed=list(change),max_position_error_m=max(x['position_error_m']for x in errors),final_position_error_W=errors[-1]['position_error_W'],checks=errors)
    record=dict(status='diagnostic_counterfactual_not_a_candidate',physical_interpretation='conditional linear pose pull is large and not verified; cannot be committed',
                unchanged='raw IMU, alpha, timestamps, training p/v, bg, heldout physical scan identities; only listed mean components altered',
                commit_R_pull_deg=float(np.linalg.norm(pull)*180/np.pi),results=results,
                no_frontend_commit=True,no_map_generated=True)
    (output/'counterfactual.json').write_text(json.dumps(record,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(12,5))
    for name,result in results.items():ax.plot([x['seconds_from_train_end']for x in result['checks']],[x['position_error_m']for x in result['checks']],label=name)
    ax.set(xlabel='seconds from train end',ylabel='position mismatch vs same LiDAR reference (m)',title='Single-mean-component counterfactuals: diagnostic only, no valid initializer')
    ax.legend(fontsize=8);ax.grid(alpha=.3);fig.tight_layout();fig.savefig(output/'single_component_counterfactual.png',dpi=150);plt.close(fig)
    print({k:v['max_position_error_m']for k,v in results.items()});return record


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('base');p.add_argument('output');a=p.parse_args();run(a.base,a.output)
