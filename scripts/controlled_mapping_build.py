#!/usr/bin/env python3
"""Build a separate frontend+backend; never writes production install."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime,timezone

REPO=Path(__file__).resolve().parents[1]

def build_plan(repo,output):
    repo=Path(repo).resolve()
    requested=Path(output).absolute()
    for path in [requested,*requested.parents]:
        if path.is_symlink():raise ValueError('controlled output ancestor is a symlink: '+str(path))
    output=requested.resolve()
    forbidden=[repo/'install',repo/'build',repo/'maps',repo/'src']
    if output==repo or any(output==p or p in output.parents for p in forbidden):
        raise ValueError('use a separate controlled build root, not production/data/source')
    for relative in ['backend_build','backend_install','colcon_logs','frontend',
                     'frontend/source','frontend/build','frontend/install','build_identity.json']:
        child=output/relative
        for path in [child,*child.parents]:
            if path==output:break
            if path.is_symlink():raise ValueError('controlled output is a symlink: '+str(path))
        if output not in child.resolve().parents:
            raise ValueError('controlled output escapes build root: '+str(child))
        if child.is_dir():
            for directory,dirs,files in os.walk(child,followlinks=False):
                for name in [*dirs,*files]:
                    existing=Path(directory)/name
                    if existing.is_symlink() and output not in existing.resolve().parents:
                        raise ValueError('existing build output symlink escapes root: '+str(existing))
    return [
        ['/usr/bin/colcon','--log-base',str(output/'colcon_logs'),'build',
         '--base-paths',str(repo/'src'),'--executor','sequential',
         '--packages-up-to','a2w_fastlio_mapping',
         '--build-base',str(output/'backend_build'),'--install-base',str(output/'backend_install'),
         '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=ON',
         '-DPython3_EXECUTABLE=/usr/bin/python3','-DPYTHON_EXECUTABLE=/usr/bin/python3'],
        ['/usr/bin/python3',str(repo/'scripts/diagnostics/frontend_init_v4_tools.py'),
         'prepare',str(output/'frontend')],
        ['/usr/bin/python3',str(repo/'scripts/diagnostics/frontend_init_v4_tools.py'),
         'build',str(output/'frontend')],
    ]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def invoke(command,log,repo):
    print('执行：'+' '.join(command),flush=True)
    with log.open('x') as stream:
        result=subprocess.run(command,cwd=repo,stdout=stream,stderr=subprocess.STDOUT)
    if result.returncode:raise RuntimeError(f'build failed ({result.returncode}); see {log}')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(os.environ.get('A2W_CONTROLLED_BUILD_ROOT',REPO/'.controlled_mapping')))
    parser.add_argument('--plan',action='store_true')
    parser.add_argument('--resume',action='store_true',help='explicitly resume isolated outputs, preserving previous logs')
    parser.add_argument('--verified-frontend',type=Path,help='use an already built, identity-verified isolation; default compiles new')
    args=parser.parse_args();commands=build_plan(REPO,args.root);root=args.root.resolve()
    if args.plan:print(json.dumps(commands,indent=2));return
    if root.exists() and not args.resume:raise ValueError('controlled root exists; use a new root or explicit --resume')
    root.mkdir(parents=True,exist_ok=True)
    attempt=root/('attempt_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));attempt.mkdir()
    submodules=subprocess.check_output(['git','submodule','status','--recursive'],cwd=REPO,text=True)
    if any(line.startswith(('-', '+','U')) for line in submodules.splitlines()):
        raise ValueError('pinned submodules missing/mismatched; clone --recurse-submodules or run setup first')
    (attempt/'submodules.txt').write_text(submodules)
    invoke(commands[0],attempt/'backend.log',REPO)
    isolation=(args.verified_frontend or root/'frontend').resolve()
    if args.verified_frontend is None:
        if not isolation.exists():invoke(commands[1],attempt/'frontend_prepare.log',REPO)
        command=commands[2]+(['--resume'] if (isolation/'build').exists() else [])
        invoke(command,attempt/'frontend_build.log',REPO)
    fm=json.loads((isolation/'build_manifest.json').read_text())
    patch=REPO/'patches/fast_lio_v4/0001-frontend-init-quality-units.patch'
    if fm['patch_sha256']!=sha(patch):raise ValueError('frontend does not match current v4 patch')
    frontend=Path(fm['binaries']['fastlio_mapping']['path']).resolve()
    if sha(frontend)!=fm['binaries']['fastlio_mapping']['sha256']:raise ValueError('frontend binary identity changed')
    install=root/'backend_install'
    setup=install/'setup.bash'
    binaries={'frontend':frontend,**{key:install/'a2w_fastlio_mapping/lib/a2w_fastlio_mapping'/name
        for key,name in [('backend','mapping_backend_node'),('ingress','mapping_ingress_node')]}}
    identities={}
    for key,binary in binaries.items():
        command=['bash','--noprofile','--norc','-c','source "$1"; ldd "$2"','identity',str(setup),str(binary)]
        ldd=subprocess.check_output(command,text=True,env=os.environ.copy())
        if 'not found' in ldd:raise ValueError('unresolved binary library: '+str(binary))
        libraries={}
        for match in re.finditer(r'^\s*(\S+)\s+=>\s+(/\S+)',ldd,re.MULTILINE):
            name,file=match.groups();file=Path(file).resolve()
            if name.startswith('liba2w_') and root not in file.parents:
                raise ValueError('mixed production/experimental common library: '+str(file))
            libraries[name]={'path':str(file),'sha256':sha(file)}
        identities[key]={'path':str(binary.resolve()),'sha256':sha(binary),'ldd':ldd,'libraries':libraries}
    identity={'schema':'controlled_mapping_build_v1','root':str(root),'frontend_binary':str(frontend),
        'frontend_isolation':str(isolation),'backend_install':str(install),
        'frontend_build_manifest':str(isolation/'build_manifest.json'),'patch_sha256':sha(patch),
        'binaries':identities,'backend_experiment_enabled':False,'production_install_changed':False,
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()}
    (root/'build_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    print('受控构建完成：'+str(root/'build_identity.json'),flush=True)

if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError,subprocess.CalledProcessError) as error:
        print(str(error),file=sys.stderr);raise SystemExit(2)
