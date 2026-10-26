#!/usr/bin/env python3
"""Export unchanged saved keyframe body points under their raw frontend poses."""
import argparse,csv,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def read(path):
 h={}
 with path.open('rb')as f:
  while True:
   line=f.readline()
   if not line:raise ValueError('missing header')
   k,*v=line.decode().split()
   if k.startswith('#'):continue
   h[k]=v
   if k=='DATA':break
  if h['DATA']!=['binary'] or h['FIELDS']!=['x','y','z','intensity'] or h['SIZE']!=['4']*4 or h['TYPE']!=['F']*4:raise ValueError('explicit XYZI binary layout required')
  a=np.frombuffer(f.read(),dtype='<f4').reshape(-1,4).copy()
  if len(a)!=int(h['POINTS'][0]):raise ValueError('PCD size mismatch')
  return a

def export(directory):
 bundle=directory/'map_bundle';rows=list(csv.DictReader((bundle/'keyframes/index.csv').open()));target=directory/'frontend_native_raw.pcd'
 if target.exists():raise ValueError('fresh native raw output required')
 total=0
 for r in rows:
  with(bundle/r['cloud']).open('rb')as f:
   while True:
    line=f.readline().decode()
    if line.startswith('POINTS '):total+=int(line.split()[1]);break
 header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {total}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {total}\nDATA binary\n'
 n=0
 with target.open('xb')as f:
  f.write(header.encode())
  for r in rows:
   a=read(bundle/r['cloud']);R=Rotation.from_quat([float(r[k])for k in ['qx','qy','qz','qw']]).as_matrix();p=np.array([float(r[k])for k in ['tx','ty','tz']]);a[:,:3]=a[:,:3].astype(float)@R.T+p
   f.write(a.astype('<f4').tobytes());n+=len(a)
 if n!=total or target.stat().st_size!=len(header.encode())+total*16:raise ValueError('map export count/size failed')
 report=dict(points=n,keyframes=len(rows),source='same keyframe body points, raw frontend poses, no voxel/filter/ICP/leveling',sha256=hashlib.sha256(target.read_bytes()).hexdigest())
 (directory/'frontend_native_raw.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();export(a.directory)
