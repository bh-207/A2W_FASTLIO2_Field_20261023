"""Reproduce independent motion experiments. No rejected/fallback map replay."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
E=Path(__file__).resolve().parent;W=E.parents[1];OLD=W/'experiments/field_frontend_z_init_20261024';DATA=W/'captures/field_frontend_z_init_20261024_001'


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve()
    if out.exists():raise ValueError('fresh output required; nothing overwritten')
    if shutil.disk_usage(out.parent).free<3*1024**3:raise ValueError('need >=3GiB for analysis; accepted full reconstruction requires >=12GiB')
    manifest=Path('/tmp/a2w_frontend_z_init_20261024_001/runs/B_008/reconstruction/preprocess_index.csv')
    if not manifest.exists():manifest=DATA/'runs/B_008/reconstruction/preprocess_index.csv'
    if not manifest.exists():raise ValueError('original frozen preprocessed scans/manifest required')
    original=DATA/'joint_motion_150_215_001/lidar_poses.npz';imu=DATA/'input_audit/imu_raw.csv'
    for f in [original,imu,DATA/'ground_eval/final_comparison/native_z_fixed_up_profiles.json']:
        if not f.exists():raise ValueError('missing frozen source '+str(f))
    out.mkdir();env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'mpl_cache'))
    commands=[]
    def call(args,label,allow_rejection=False):
        args=list(map(str,args));print('RUN',label,flush=True)
        with (out/(label+'.log')).open('x') as f:code=subprocess.run(args,env=env,stdout=f,stderr=subprocess.STDOUT,cwd=W).returncode
        commands.append(dict(command=args,exit=code,label=label));(out/'commands.json').write_text(json.dumps(commands,indent=2))
        if code and not(allow_rejection and code==2):raise RuntimeError('failed '+label+'; see log')
    call([sys.executable,E/'build_reference.py'],'build')
    call([sys.executable,E/'shape_geometry.py',DATA/'ground_eval/final_comparison/native_z_fixed_up_profiles.json',out/'shape'],'shape')
    call([sys.executable,E/'joint_init/joint_cli.py',original,imu,out/'old_reference_eiv_diagnostic.json','--acc-scale','1.0136393576435327'],'old_eiv_diagnostic')
    call([sys.executable,E/'reference_sequence.py',manifest,out/'pair_full_rate','--polish','--match',original],'pair_full_rate',True)
    cases={'original_reference':original}
    for name,deskew in [('local_submap','previous'),('no_deskew','none'),('current_motion_iterated','current_iterated')]:
        dest=out/name
        call([sys.executable,E/'reference_sequence.py',manifest,dest,'--mode','submap','--polish','--deskew',deskew,'--match',original],name,True)
        r=json.loads((dest/'registration.json').read_text())
        if r['complete']:cases[name]=dest/'matched_original213.npz'
    results={}
    for name,poses in cases.items():
        dest=out/(name+'_candidate.json');call([sys.executable,E/'consistent_init.py',poses,imu,dest],name+'_fit');results[name]=json.loads(dest.read_text())
    accepted=[(name,c)for name,c in results.items()if c['accepted']]
    summary=dict(candidates={k:dict(accepted=c['accepted'],reason=c['reason'],heldout=c.get('heldout_validation'),chi2=c.get('residual_reduced_chi2'))for k,c in results.items()},
        frontend_commits=0,maps_generated=0,production_changed=False,hardware_connected=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    if accepted:
        if shutil.disk_usage(out).free<12*1024**3:raise RuntimeError('accepted candidate retained; >=12GiB required for actual full replay')
        # No old objects/private binary required. Build the unchanged published
        # frontend/backend sources independently, then verify real commit BEFORE
        # allowing Bundle reconstruction. A failed/unused candidate creates no map.
        root=out/'accepted_full_reconstruction';root.mkdir()
        call([sys.executable,OLD/'build.py','--build',root/'build'],'full_build')
        name,_=accepted[0];candidate=out/(name+'_candidate.json');prepared=out/'frontend_candidate.json'
        call([sys.executable,E/'prepare_candidate.py',candidate,DATA/'runs/B_009/reconstruction/post_index.csv',DATA/'input_audit/cloud_timing.csv',prepared],'prepare')
        for repeat in [1,2]:
            run_id='JOINT_'+str(repeat)
            call([sys.executable,OLD/'run_consumption.py',run_id,'--root',root,'--candidate',prepared,'--export-seconds','0'],'full_'+str(repeat))
            run=root/'runs'/run_id;receipt=json.loads((run/'reconstruction/joint_initialization_receipt.json').read_text())
            if not receipt.get('committed'):raise RuntimeError('NO actual JOINT_INIT_COMMIT: run retained; map construction refused')
            summary['frontend_commits']+=1
            call([sys.executable,OLD/'input_audit/audit_consumption_stages.py','--run',run,'--original',DATA/'input_audit','--output',out/('input_'+str(repeat))],'input_'+str(repeat))
            binary=root/'build/backend_install/a2w_fastlio_mapping/lib/a2w_fastlio_mapping/field_consumption_backend'
            reference=W/'captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle'
            call(['bash',OLD/'run.sh',binary,run/'reconstruction/post_index.csv',reference,out/('map_'+str(repeat)),'--source-manifest',run/'build_manifest.json'],'backend_'+str(repeat))
            call([sys.executable,OLD/'export_raw_map.py',out/('map_'+str(repeat))],'raw_'+str(repeat))
            call([sys.executable,OLD/'ground_eval/ground_eval.py','evaluate','--frozen',DATA/'ground_eval/B004_frozen','--index',run/'reconstruction/post_index.csv','--contract',OLD/'config/identity_contract.json','--preprocess',run/'reconstruction/preprocess_identity.csv','--out',out/('geometry_'+str(repeat))],'geometry_'+str(repeat))
            summary['maps_generated']+=1
        summary['geometry_acceptance']='REQUIRES_ALL_FROZEN_GEOMETRY_AND_SELF_REPEAT_REVIEW; map generation never implies pass'
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items()if k!='candidates'},indent=2))


if __name__=='__main__':main()
