"""Explicit, configuration-only authorization; never certifies calibration."""
import copy
from pathlib import Path

STATUS = 'unverified_experimental_initial_conditions'
POLICY_PATH = 'snapshots/experimental_initial_conditions.json'
EXPECTED = dict(extrinsic_R=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                extrinsic_T=[0, 0, 0], offset_s=0.0, extrinsic_est_en=True)


def initial_values(params):
    mapping = params.get('mapping', {})
    return dict(extrinsic_R=mapping.get('extrinsic_R'), extrinsic_T=mapping.get('extrinsic_T'),
                offset_s=params.get('common', {}).get('time_offset_lidar_to_imu'),
                extrinsic_est_en=mapping.get('extrinsic_est_en'))


def experimental_policy_errors(policy, contract, params, base_sha256):
    errors = []
    if not isinstance(policy, dict) or policy.get('enabled') is not True:
        return ['unverified experimental initial conditions require explicit session authorization']
    if policy.get('schema') != 'mapping_experimental_initials_v1':
        errors.append('experimental initial-condition policy schema unsupported')
    if not isinstance(policy.get('authorization_note'), str) or not policy['authorization_note'].strip():
        errors.append('explicit operator authorization note required')
    if policy.get('base_config_sha256') != base_sha256:
        errors.append('experimental authorization does not bind actual base config')
    if initial_values(params) != EXPECTED or params.get('mapping', {}).get('extrinsic_est_en') is not True:
        errors.append('authorization only covers existing R=I, T=0, offset=0, extrinsic_est_en=true')
    if params.get('common', {}).get('time_sync_en') is not False:
        errors.append('experimental authorization cannot enable automatic time synchronization')
    if policy.get('initial_conditions') != EXPECTED:
        errors.append('experimental authorized initial values changed')
    if contract.get('calibration_status') != STATUS or policy.get('calibration_status') != STATUS:
        errors.append('explicit unverified experimental calibration status required')
    if policy.get('physical_calibration_verified') is not False:
        errors.append('experimental policy must not claim physical calibration')
    for key in ['time_offset_evidence', 'extrinsic_evidence']:
        evidence = contract.get(key, {})
        if (evidence.get('physical_calibration_verified') is not False or
                evidence.get('evidence_scope') != 'configured_value_only' or
                evidence.get('sha256') != base_sha256):
            errors.append(key + ' must bind configuration-only evidence, not calibration')
    for key, expected in [('source_provenance', 'real'), ('gyro_unit', 'rad/s'),
                          ('accel_unit', 'm/s^2'), ('imu_frame', 'hesai_imu_aligned'),
                          ('lidar_frame', 'hesai_lidar_aligned'), ('imu_minus_lidar_offset_ns', 0)]:
        if contract.get(key) != expected:
            errors.append('JT128 experimental policy requires ' + key + '=' + str(expected))
    return errors


def authorize_existing_initials(source, base_source, params, base_sha256,
                               authorization_note, authorized=False):
    if authorized is not True:
        raise ValueError('explicit --authorize-existing-unverified-initials is required; default is off')
    contract = copy.deepcopy(source)
    policy = dict(schema='mapping_experimental_initials_v1', enabled=True,
                  authorization_note=authorization_note, base_config_sha256=base_sha256,
                  calibration_status=STATUS, physical_calibration_verified=False,
                  initial_conditions=copy.deepcopy(EXPECTED))
    evidence = dict(source=str(Path(base_source).resolve()), config=str(Path(base_source).resolve()),
                    version='observed configuration SHA256 ' + base_sha256, sha256=base_sha256,
                    evidence_scope='configured_value_only', physical_calibration_verified=False,
                    authorization_note=authorization_note,
                    note='Existing operator-authorized experimental initial value; physical calibration unverified.')
    contract.update(calibration_status=STATUS, imu_minus_lidar_offset_ns=0,
                    time_offset_evidence=dict(evidence),
                    extrinsic_evidence=dict(evidence, base_config_sha256=base_sha256))
    errors = experimental_policy_errors(policy, contract, params, base_sha256)
    if errors:
        raise ValueError('; '.join(errors))
    return contract, policy


def condition_labels(metadata):
    return dict(calibration_status=metadata.get('calibration_status', 'not_independently_certified'),
                physical_calibration_verified=False,
                experimental_initials_authorized=metadata.get('experimental_initials_authorized') is True)
