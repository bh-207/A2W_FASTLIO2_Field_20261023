"""Strict binary/ASCII PCD or explicit point-identity NPZ reader."""
from pathlib import Path
import numpy as np


def read_cloud(path,point_ids_path=None):
    path = Path(path)
    if path.suffix == '.npz':
        with np.load(path,allow_pickle=False) as z:
            body,ids = np.asarray(z['body'],float),np.asarray(z['point_id'],np.int64)
    else:
        header = {}
        with path.open('rb') as f:
            for _ in range(100):
                line = f.readline(65536).decode('ascii').strip()
                if not line or line.startswith('#'):
                    continue
                k,*v = line.split()
                header[k] = v
                if k == 'DATA':
                    break
            else:
                raise ValueError('PCD DATA header missing')
            names = header['FIELDS']
            sizes = list(map(int,header['SIZE']))
            types = header['TYPE']
            counts = list(map(int,header.get('COUNT',['1']*len(names))))
            if len({len(names),len(sizes),len(types),len(counts)}) != 1 or len(set(names)) != len(names):
                raise ValueError('invalid PCD field layout')
            dtype = np.dtype([(n,'<'+{'F':'f','I':'i','U':'u'}[t]+str(s)) if c==1 else
                              (n,'<'+{'F':'f','I':'i','U':'u'}[t]+str(s),(c,))
                              for n,s,t,c in zip(names,sizes,types,counts)])
            n = int(header['POINTS'][0])
            if n<0 or not {'x','y','z'}.issubset(names):
                raise ValueError('PCD requires XYZ and nonnegative POINTS')
            if header['DATA']==['binary']:
                payload = f.read()
                if len(payload)!=n*dtype.itemsize:
                    raise ValueError('PCD binary payload size mismatch')
                records = np.frombuffer(payload,dtype=dtype)
                body = np.column_stack([records[a] for a in 'xyz']).astype(float)
                ids = records['point_id'].astype(np.int64) if 'point_id' in names else np.arange(n,dtype=np.int64)
            elif header['DATA']==['ascii']:
                values = np.loadtxt(f,ndmin=2) if n else np.empty((0,sum(counts)))
                if values.shape!=(n,sum(counts)):
                    raise ValueError('PCD ASCII payload size mismatch')
                offsets = np.cumsum([0]+counts[:-1])
                fields = {name:values[:,off] for name,off in zip(names,offsets)}
                body = np.column_stack([fields[a] for a in 'xyz'])
                ids = fields['point_id'].astype(np.int64) if 'point_id' in names else np.arange(n,dtype=np.int64)
            else:
                raise ValueError('compressed PCD requires explicit decoding')
    if point_ids_path is not None:
        ids = np.fromfile(point_ids_path,dtype='<u4').astype(np.int64)
        if not np.array_equal(np.sort(ids),np.arange(len(body))):
            raise ValueError('point-ID sidecar must be complete unique preprocess indices 0..N-1')
    if body.shape!=(len(ids),3) or len(set(map(int,ids)))!=len(ids) or not np.isfinite(body).all():
        raise ValueError('invalid cloud shape, nonfinite coordinates or duplicate point identities')
    return body,ids
