"""Pure offline geometry. T_A_B maps B coordinates into A coordinates.

The shortest-arc implementation is reused from the repository's existing
gravity_contract. No ROS, sensor calibration, or runtime TF is changed.
"""
import importlib.util
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPOSITORY = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    'a2w_existing_gravity_contract', REPOSITORY / 'scripts/diagnostics/gravity_contract.py')
_contract = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_contract)
rigid = _contract.rigid
transform_points = _contract.transform_points


def pose_matrix(position, quaternion):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,) or
            not np.isfinite(position).all() or not np.isfinite(quaternion).all() or
            np.linalg.norm(quaternion) < 1e-12):
        raise ValueError('invalid pose; quaternion must be xyzw')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position
    return rigid(result)


def inverse_se3(transform):
    transform = rigid(transform)
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return rigid(inverse)


def map_camera_transform(camera_body, map_body):
    return rigid(rigid(map_body) @ inverse_se3(camera_body))


def alignment_with_provenance(gravity, source_frame, reference_stamp_ns, cloud_stamp_ns,
                              max_age_ns, source):
    reference = dict(vector=list(gravity), frame_id=source_frame,
                     stamp_ns=reference_stamp_ns, quality_valid=True, source=source)
    # quality_valid here confirms a valid saved estimator reference and units;
    # it does not assert independent physical accuracy or calibrated extrinsics.
    return _contract.alignment(reference, source_frame, cloud_stamp_ns,
                               max_reference_age_ns=max_age_ns)


def level_transform(gravity):
    """Pure shortest SO(3): anti-gravity -> +Z, no additional twist, t=0.

    Gravity cannot observe yaw. The shortest arc is a deterministic coordinate
    choice; it does not guarantee preservation of an Euler yaw decomposition.
    """
    return alignment_with_provenance(gravity, 'source', 0, 0, 0,
                                     'caller_supplied_vector_pure_geometry')['matrix']


def transform_vectors(vectors, transform):
    vectors = np.asarray(vectors, dtype=float)
    if vectors.shape[-1:] != (3,) or not np.isfinite(vectors).all():
        raise ValueError('expected finite vectors with final dimension three')
    return vectors @ rigid(transform)[:3, :3].T


def transform_plane(normal, offset, transform):
    normal = np.asarray(normal, dtype=float)
    if normal.shape != (3,) or not np.isfinite(offset):
        raise ValueError('invalid plane')
    changed_normal = transform_vectors(normal, transform)
    return changed_normal, float(offset - changed_normal @ rigid(transform)[:3, 3])


def transform_state(state, transform):
    transform = rigid(transform)
    pose = transform @ pose_matrix(state['position'], state['quaternion'])
    return dict(position=pose[:3, 3],
                quaternion=Rotation.from_matrix(pose[:3, :3]).as_quat(),
                velocity=transform_vectors(state['velocity'], transform),
                gravity=transform_vectors(state['gravity'], transform))


def synthetic_tilt_sensitivity(length_m, tilt_deg, headings_deg, axis='x'):
    """Rotate a physically horizontal route into a tilted coordinate system.

    Length means true 3D horizontal distance. Thus vertical difference is
    L*sin(tilt) in the maximum-sensitivity direction, zero along the tilt axis.
    L*tan(tilt) instead assumes L is projected horizontal distance in tilted axes.
    """
    rotation = Rotation.from_euler(axis, tilt_deg, degrees=True)
    result = []
    for heading in headings_deg:
        theta = math.radians(heading)
        horizontal = np.array([length_m*math.cos(theta), length_m*math.sin(theta), 0.])
        tilted = rotation.apply(horizontal)
        result.append(dict(provenance='synthetic', axis=axis, tilt_deg=float(tilt_deg),
                           physical_horizontal_length_m=float(length_m), heading_deg=float(heading),
                           vertical_difference_m=float(tilted[2]),
                           projected_xy_length_m=float(np.linalg.norm(tilted[:2])),
                           total_distance_m=float(np.linalg.norm(tilted))))
    return result


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


def write_pcd(path, header, data, viewpoint=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new_header = []
    for line in header:
        key = line.split(maxsplit=1)[0] if line else ''
        if key == 'DATA':
            line = 'DATA binary'
        elif key == 'VIEWPOINT' and viewpoint is not None:
            quaternion = Rotation.from_matrix(viewpoint[:3, :3]).as_quat()
            values = [*viewpoint[:3, 3], quaternion[3], *quaternion[:3]]
            line = 'VIEWPOINT ' + ' '.join(format(float(value), '.17g') for value in values)
        new_header.append(line)
    with path.open('xb') as stream:
        stream.write(('\n'.join(new_header)+'\n').encode('ascii'))
        stream.write(data.tobytes())


def transform_pcd(source, target, transform):
    if Path(source).resolve() == Path(target).resolve():
        raise ValueError('source and target must differ')
    transform = rigid(transform)
    header, data = read_pcd(source)
    changed = transform_points(xyz(data), transform)
    for index, field in enumerate(('x', 'y', 'z')):
        data[field] = changed[:, index]
    normal_fields = ('normal_x', 'normal_y', 'normal_z')
    normal_presence = [name in data.dtype.names for name in normal_fields]
    if any(normal_presence):
        if not all(normal_presence):
            raise ValueError('PCD has incomplete normal fields')
        normals = np.column_stack([data[name] for name in normal_fields])
        rotated = transform_vectors(normals, transform)
        for index, field in enumerate(normal_fields):
            data[field] = rotated[:, index]
    original_viewpoint = np.eye(4)
    for line in header:
        if line.startswith('VIEWPOINT '):
            v = list(map(float, line.split()[1:]))
            original_viewpoint = pose_matrix(v[:3], [*v[4:], v[3]])
    write_pcd(target, header, data, transform @ original_viewpoint)
    return len(data)
