"""Frozen PCD reader reused verbatim from existing gravity_geometry; no transforms."""
from pathlib import Path
import numpy as np

def read_pcd(path):
    """Read actual ASCII/binary PCD without dropping intensity or normal fields."""
    path = Path(path)
    header = []
    values = {}
    with path.open('rb') as stream:
        for _ in range(100):
            raw = stream.readline(65536)
            if not raw:
                raise ValueError('PCD DATA header missing')
            line = raw.decode('ascii').strip()
            header.append(line)
            if not line or line.startswith('#'):
                continue
            key, *parts = line.split()
            values[key] = parts
            if key == 'DATA':
                break
        else:
            raise ValueError('PCD header too long')
        names = values['FIELDS']
        sizes = list(map(int, values['SIZE']))
        types = values['TYPE']
        counts = list(map(int, values.get('COUNT', ['1'] * len(names))))
        if len({len(names), len(sizes), len(types), len(counts)}) != 1:
            raise ValueError('inconsistent PCD field layout')
        if len(set(names)) != len(names) or any(count < 1 for count in counts):
            raise ValueError('invalid PCD field count')
        dtype = np.dtype([(name, '<'+{'F':'f', 'I':'i', 'U':'u'}[kind]+str(size))
                          if count == 1 else
                          (name, '<'+{'F':'f', 'I':'i', 'U':'u'}[kind]+str(size), (count,))
                          for name, size, kind, count in zip(names, sizes, types, counts)])
        count = int(values['POINTS'][0])
        if count < 0 or not {'x','y','z'}.issubset(names):
            raise ValueError('PCD requires x/y/z and nonnegative POINTS')
        if values['DATA'] == ['binary']:
            payload = stream.read()
            if len(payload) != count*dtype.itemsize:
                raise ValueError('PCD payload size mismatch')
            data = np.frombuffer(payload, dtype=dtype).copy()
        elif values['DATA'] == ['ascii']:
            payload = np.loadtxt(stream, ndmin=2) if count else np.empty((0, sum(counts)))
            if payload.shape != (count, sum(counts)):
                raise ValueError('PCD payload size mismatch')
            data = np.empty(count, dtype=dtype)
            offset = 0
            for name, field_count in zip(names, counts):
                data[name] = payload[:, offset] if field_count == 1 else payload[:, offset:offset+field_count]
                offset += field_count
        else:
            raise ValueError('only actual ASCII/binary PCD supported; compressed input must be decoded explicitly')
    return header, data


def xyz(data):
    return np.column_stack([data['x'], data['y'], data['z']]).astype(np.float64)

