"""Offline SE3/gravity conventions. Never infer missing gravity from map Z.

T_A_B maps coordinates in B into A. This module changes coordinates only;
it neither edits physical sensor extrinsics nor publishes a TF/map.
"""
import hashlib
import json
import numpy as np

VERSION = "gravity-rp-v3-1"


def rigid(matrix):
    t = np.asarray(matrix, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1], atol=1e-10, rtol=0) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-8, rtol=0) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-8, rtol=0)):
        raise ValueError("expected a finite proper rigid SE3 transform")
    return t


def alignment(reference, frame_id, stamp_ns, input_version="raw-v1",
              max_reference_age_ns=0):
    """Return A_normalized_original and its explicit provenance.

    Minimal rotation takes the measured anti-gravity direction to +Z; yaw is
    not independently observed by gravity. Caller must supply a valid reference
    for this exact coordinate frame/epoch, never a future optimized estimate.
    """
    if type(stamp_ns) is not int or stamp_ns < 0:
        raise ValueError("cloud timestamp must be nonnegative integer nanoseconds")
    if type(max_reference_age_ns) is not int or max_reference_age_ns < 0:
        raise ValueError("gravity age budget must be nonnegative integer nanoseconds")
    if input_version.startswith("gravity-") or frame_id.endswith("_gravity_rp"):
        raise ValueError("input is already gravity normalized")
    if reference is None:
        raise ValueError("gravity reference is missing")
    if reference.get("frame_id") != frame_id:
        raise ValueError("gravity frame does not match cloud frame")
    if reference.get("quality_valid") is not True:
        raise ValueError("gravity quality is unconfirmed")
    reference_stamp = reference.get("stamp_ns")
    if type(reference_stamp) is not int or reference_stamp < 0:
        raise ValueError("gravity timestamp is missing or invalid")
    if reference_stamp > stamp_ns:
        raise ValueError("future gravity estimate cannot normalize this cloud")
    if stamp_ns - reference_stamp > max_reference_age_ns:
        raise ValueError("gravity reference age exceeds explicit budget")
    if not reference.get("source"):
        raise ValueError("gravity provenance source is missing")
    gravity = np.asarray(reference.get("vector"), dtype=float)
    if gravity.shape != (3,) or not np.isfinite(gravity).all():
        raise ValueError("gravity must be a finite three-vector in m/s^2")
    norm = float(np.linalg.norm(gravity))
    if not 8.0 <= norm <= 12.0:
        raise ValueError("gravity norm/units are invalid; expected m/s^2")
    up = -gravity / norm
    z = np.array([0., 0., 1.])
    dot = float(np.clip(up @ z, -1., 1.))
    cross = np.cross(up, z)
    sine = float(np.linalg.norm(cross))
    if sine < 1e-15:
        rotation = np.eye(3) if dot >= 0 else np.diag([1., -1., -1.])
    else:
        axis = cross / sine
        skew = np.array([[0., -axis[2], axis[1]],
                         [axis[2], 0., -axis[0]],
                         [-axis[1], axis[0], 0.]])
        # Unit-axis Rodrigues avoids cancellation in 1/(1+dot) near pi.
        rotation = np.eye(3) + sine * skew + (1. - dot) * (skew @ skew)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    record = dict(version=VERSION, input_version=input_version,
                  source_frame=frame_id, target_frame=frame_id + "_gravity_rp",
                  cloud_stamp_ns=int(stamp_ns), gravity_stamp_ns=reference_stamp,
                  max_reference_age_ns=max_reference_age_ns,
                  gravity_source=reference["source"], gravity_original=gravity.tolist(),
                  physical_extrinsic=False, matrix=rigid(transform))
    identity = {**record, "matrix": transform.tolist()}
    record["identity_sha256"] = hashlib.sha256(
        json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return record


def transform_points(points, transform):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("expected finite Nx3 points")
    t = rigid(transform)
    return points @ t[:3, :3].T + t[:3, 3]


def restore_registration(normalized_target_source, source_normalization, target_normalization):
    """Restore T_original_target_original_source = A_t^-1 T_normalized A_s."""
    source = rigid(source_normalization)
    target = rigid(target_normalization)
    normalized = rigid(normalized_target_source)
    return rigid(np.linalg.inv(target) @ normalized @ source)
