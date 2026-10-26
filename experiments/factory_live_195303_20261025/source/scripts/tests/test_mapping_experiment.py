"""Offline regression for explicitly authorized, uncalibrated trial sessions."""
import copy
import importlib
import io
import json
import os
from contextlib import redirect_stdout
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import mapping_capture as cli
import mapping_capture_lib as capture


class ExperimentalInitialConditionsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.base = self.root / 'observed.yaml'
        self.base.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'common': {'lid_topic': '/lidar_points', 'imu_topic': '/lidar_imu',
                       'time_sync_en': False, 'time_offset_lidar_to_imu': 0.0},
            'mapping': {'extrinsic_R': [1, 0, 0, 0, 1, 0, 0, 0, 1],
                        'extrinsic_T': [0, 0, 0], 'extrinsic_est_en': True}}}}))
        self.source = capture.interface_template()
        self.source.update(gyro_unit='rad/s', accel_unit='m/s^2',
                           imu_frame='hesai_imu_aligned', lidar_frame='hesai_lidar_aligned')
        evidence = dict(source='saved-driver-source', version='audited-version',
                        config='saved-driver-config', sha256='a' * 64)
        for key in ['gyro_unit_evidence', 'accel_unit_evidence', 'frame_evidence']:
            self.source[key] = copy.deepcopy(evidence)
        self.interface = self.root / 'interface.json'
        capture.write_json(self.interface, self.source, exclusive=True)

    def policy_module(self):
        self.assertTrue((SCRIPTS / 'mapping_initial_conditions.py').is_file(),
                        'Explicit default-off experiment policy is missing')
        return importlib.import_module('mapping_initial_conditions')

    def authorize(self, **kwargs):
        return self.policy_module().authorize_existing_initials(
            self.source, self.base, capture.parameters(self.base), capture.sha256(self.base),
            authorization_note='Operator authorized existing unverified experimental initial values',
            **kwargs)

    def session(self):
        _, policy = self.authorize(authorized=True)
        return cli.init_session('trial', self.root / 'captures', self.root / 'no-build',
                                self.interface, base=self.base, experimental_policy=policy)

    def test_policy_is_default_off(self):
        with self.assertRaises(ValueError):
            self.authorize()

    def test_configuration_evidence_is_not_calibration_and_requires_session_policy(self):
        original = copy.deepcopy(self.source)
        contract, policy = self.authorize(authorized=True)
        self.assertEqual(original, self.source)
        self.assertEqual(0, contract['imu_minus_lidar_offset_ns'])
        self.assertEqual('unverified_experimental_initial_conditions', contract['calibration_status'])
        for key in ['time_offset_evidence', 'extrinsic_evidence']:
            self.assertIs(False, contract[key]['physical_calibration_verified'])
            self.assertEqual('configured_value_only', contract[key]['evidence_scope'])
            self.assertEqual(capture.sha256(self.base), contract[key]['sha256'])
        self.assertTrue(capture.interface_errors(contract, self.base))
        self.assertEqual([], capture.interface_errors(contract, self.base, experimental_policy=policy))

    def test_authorization_cannot_waive_units_frames_offset_or_base_hash(self):
        contract, policy = self.authorize(authorized=True)
        for key, value in [('gyro_unit', 'unknown'), ('imu_frame', 'unknown'),
                           ('imu_minus_lidar_offset_ns', 1), ('source_provenance', 'synthetic')]:
            bad = copy.deepcopy(contract); bad[key] = value
            with self.subTest(key=key):
                self.assertTrue(capture.interface_errors(bad, self.base, experimental_policy=policy))
        bad_policy = dict(policy, base_config_sha256='b' * 64)
        self.assertTrue(capture.interface_errors(contract, self.base, experimental_policy=bad_policy))

    def test_policy_rejects_driver_rotation_as_extrinsic_and_changed_initials(self):
        document = yaml.safe_load(self.base.read_text())
        for section, key, value in [
                ('mapping', 'extrinsic_R', [0, 0, -1, -1, 0, 0, 0, 1, 0]),
                ('mapping', 'extrinsic_T', [0.1, 0, 0]),
                ('mapping', 'extrinsic_est_en', False),
                ('common', 'time_offset_lidar_to_imu', 355.9),
                ('common', 'time_sync_en', True)]:
            changed = copy.deepcopy(document)
            changed['/**']['ros__parameters'][section][key] = value
            self.base.write_text(yaml.safe_dump(changed))
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.authorize(authorized=True)
        self.base.write_text(yaml.safe_dump(document))

    def test_authorization_is_frozen_and_all_frontend_gates_are_preserved(self):
        original = self.interface.read_bytes()
        session = self.session()
        metadata = capture.read_json(session / 'session.json')
        self.assertIs(True, metadata['experimental_initials_authorized'])
        self.assertIs(False, metadata['physical_calibration_verified'])
        self.assertEqual(original, self.interface.read_bytes())
        identity = capture.read_json(session / 'identity.json')
        self.assertIn('snapshots/experimental_initial_conditions.json', identity['snapshots'])
        status = cli.check_session(session)
        self.assertEqual([], status['contract_errors'])
        self.assertFalse(status['start_allowed'])  # Missing build still blocks.
        params = capture.parameters(session / 'snapshots/frontend.yaml')
        for key in ['input_interface_enabled', 'quality_gate_enabled', 'runtime_state_log_enabled']:
            self.assertIs(True, params['frontend_init_v4'][key])
        self.assertEqual(capture.parameters(self.base)['mapping'], params['mapping'])
        self.assertEqual(0.0, params['common']['time_offset_lidar_to_imu'])
        self.assertIn('configured_value_only', params['frontend_init_v4']['time_offset_evidence'])

    def test_tampered_policy_snapshot_fails_integrity(self):
        session = self.session()
        path = session / 'snapshots/experimental_initial_conditions.json'
        policy = capture.read_json(path); policy['authorization_note'] = 'replaced'
        capture.write_json(path, policy)
        self.assertTrue(cli.check_session(session)['errors'])

    def test_standard_entry_has_no_experimental_authorization_option(self):
        with self.assertRaises(SystemExit):
            cli.parser().parse_args(['init', 'trial', '--authorize-existing-unverified-initials'])

    def test_separate_entry_requires_flag_and_records_the_operator_note(self):
        self.assertTrue((SCRIPTS / 'mapping_capture_experiment.py').is_file())
        entry = importlib.import_module('mapping_capture_experiment')
        args = ['init', 'separate', '--sessions-root', str(self.root / 'captures'),
                '--build-root', str(self.root / 'no-build'), '--interface', str(self.interface),
                '--base-config', str(self.base), '--authorization-note', 'Direct operator request']
        with redirect_stdout(io.StringIO()):
            self.assertEqual(2, entry.main(args))
            self.assertFalse((self.root / 'captures/separate').exists())
            self.assertEqual(0, entry.main(args + ['--authorize-existing-unverified-initials']))
        policy = capture.read_json(self.root / 'captures/separate/snapshots/experimental_initial_conditions.json')
        self.assertEqual('Direct operator request', policy['authorization_note'])

    def test_startup_summary_uses_configuration_label_and_explicit_calibration_status(self):
        session = self.session()
        summary = capture.startup_summary(
            capture.read_json(session / 'identity.json'), capture.read_json(session / 'session.json'),
            capture.parameters(session / 'snapshots/frontend.yaml'), session,
            capture.read_json(session / 'interface.json'))
        self.assertEqual('unverified_experimental_initial_conditions', summary['calibration_status'])
        self.assertNotIn('calibrated_base', summary)
        self.assertEqual(capture.sha256(self.base), summary['base_configuration']['sha256'])

    def test_save_rejects_unequal_or_missing_received_processed_counters(self):
        for reason in ['received=4 processed=3 pending=0 active=none state=saved',
                       'pending=0 active=none state=saved',
                       'received=3 processed=3 received=4 pending=0 active=none state=saved']:
            with self.subTest(reason=reason), self.assertRaises(ValueError):
                cli.validate_save_response({'success': True}, {'stage': 'backend', 'reason': reason})
        cli.validate_save_response({'success': True}, {'stage': 'backend',
            'reason': 'received=3 processed=3 pending=0 active=none state=saved'})

    def test_same_soname_different_runtime_library_is_rejected(self):
        check = getattr(cli, 'assert_build_libraries', None)
        self.assertTrue(callable(check), 'Build/runtime library identity comparison is missing')
        expected = self.root / 'libgtsam.so.4.1.1'; expected.write_bytes(b'4.1-build')
        wrong = self.root / 'libgtsam.so.4.2.0'; wrong.write_bytes(b'4.2-runtime')
        libraries = {'libgtsam.so.4': dict(path=str(expected), sha256=capture.sha256(expected))}
        actual = dict(dsos=[dict(requested_path=str(self.root / 'libgtsam.so.4'),
                                 path=str(wrong), sha256=capture.sha256(wrong))])
        with self.assertRaises(ValueError): check('backend', actual, libraries)
        actual['dsos'][0].update(path=str(expected), sha256=capture.sha256(expected))
        check('backend', actual, libraries)

    def test_binary_identity_refuses_unresolved_relocations(self):
        exe = self.root / 'frontend'; exe.write_bytes(b'executable')
        lib = self.root / 'libdep.so'; lib.write_bytes(b'dependency')
        output = 'libdep.so => ' + str(lib) + ' (0x0001)\nundefined symbol: missing_rekey (' + str(exe) + ')\n'
        with patch.object(cli, 'run', return_value=SimpleNamespace(stdout=output)):
            with self.assertRaises(ValueError):
                cli.binary_identity(exe, capture.sha256(exe), 'frontend', self.root / 'install')

    def test_capture_wrappers_prepend_private_dependencies_after_ros_setup(self):
        session = cli.init_session('shell-env', self.root / 'captures', self.root / 'missing',
                                   self.interface, base=self.base)
        controlled = self.root / 'controlled'
        setup = controlled / 'backend_install/setup.bash'; setup.parent.mkdir(parents=True)
        setup.write_text('export LD_LIBRARY_PATH=/wrong-ros-prefix\n')
        ros = self.root / 'ros.bash'; ros.write_text(':\n')
        deps = self.root / 'deps'; (deps / 'lib').mkdir(parents=True)
        env = dict(os.environ, A2W_ROS_SETUP=str(ros), A2W_CONTROLLED_BUILD_ROOT=str(controlled),
                   A2W_DEPS_PREFIX=str(deps))
        for script in ['mapping_capture.sh', 'mapping_capture_experiment.sh']:
            result = subprocess.run(['bash', str(SCRIPTS / script), 'shell', str(session), '--',
                '/usr/bin/python3', '-c', 'import os; print(os.environ["LD_LIBRARY_PATH"])'],
                env=env, capture_output=True, text=True)
            with self.subTest(script=script):
                self.assertEqual(0, result.returncode, result.stderr)
                self.assertEqual(str(deps / 'lib'), result.stdout.strip().split(':')[0])


if __name__ == '__main__':
    unittest.main()
