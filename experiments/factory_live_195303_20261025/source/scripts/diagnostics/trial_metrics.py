"""ROS-independent plans and evaluation; estimated errors are never ground truth."""
import bisect
from collections import Counter, defaultdict
import math
from pathlib import Path
import re

import numpy as np

SCENARIOS = ('straight', 'turn', 'lateral', 'avoidance')
CORE_TOPICS = ('/Odometry', '/cloud_registered_body', '/localization/pose',
               '/localization/status', '/localization/registration_status', '/tf')
TOPICS = ('/lidar_points', '/lidar_imu', *CORE_TOPICS, '/tf_static', '/clock',
          '/localization/odom', '/localization/path', '/rosout',
          '/route_follower/path', '/route_follower/raw_path',
          '/route_follower/executed_path', '/route_follower/terminal_error_body',
          '/goal_pose', '/plan', '/local_costmap/costmap', '/local_costmap/costmap_updates',
          '/local_costmap/published_footprint', '/a2w/scan', '/collision_monitor_state',
          '/navigation/nav2_ready', '/navigation/safety_status',
          '/a2w_motion_bridge/status', '/a2w_motion_bridge/request_audit',
          '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_safe', '/cmd_vel_guarded',
          '/follow_path/_action/status', '/follow_path/_action/feedback',
          '/navigate_to_pose/_action/status', '/navigate_to_pose/_action/feedback',
          '/a2w_validation/events')


def json_safe(value):
    """Keep legal no-fit sentinels inspectable without invalid JSON numbers."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def statistics(values):
    values = list(values)
    good = sorted(float(v) for v in values if math.isfinite(v))
    result = {'count': len(good), 'invalid_count': len(values)-len(good),
              'min': None, 'p50': None, 'p95': None, 'max': None}
    if not good:
        return result
    def percentile(q):
        index = (len(good)-1)*q
        i = int(index)
        return good[i] + (good[min(i+1, len(good)-1)]-good[i])*(index-i)
    result.update(min=good[0], max=good[-1], p50=percentile(.5), p95=percentile(.95))
    return result


def record_plan(output, scenario, duration, root):
    if scenario not in SCENARIOS or not math.isfinite(duration) or duration <= 0:
        raise ValueError('Known scenario and finite positive duration required')
    output = Path(output).resolve()
    topics = list(TOPICS)
    latched = {'/tf_static', '/route_follower/path', '/route_follower/raw_path',
               '/route_follower/executed_path', '/local_costmap/costmap',
               '/follow_path/_action/status', '/navigate_to_pose/_action/status',
               '/navigation/nav2_ready'}
    qos = {t: {'history': 'keep_last', 'depth': 100 if t not in latched else 1,
               'reliability': 'reliable' if t in latched else 'best_effort',
               'durability': 'transient_local' if t in latched else 'volatile'} for t in topics}
    return {'scenario': scenario, 'duration_s': duration, 'topics': topics, 'qos': qos,
            'root': str(Path(root).resolve()), 'output': str(output),
            'argv': ['ros2', 'bag', 'record', '--include-hidden-topics',
                     '--include-unpublished-topics', '-s', 'sqlite3', '-o', str(output/'bag'),
                     '--max-bag-size', '2147483648', '--max-cache-size', '33554432',
                     '--qos-profile-overrides-path', str(output/'qos.yaml'), *topics],
            'side_effects': ['record existing ROS messages', 'write trial files'],
            'starts_hardware': False, 'publishes_motion': False}


def stamp_ns(stamp):
    return int(stamp.sec)*10**9 + int(stamp.nanosec)


def pose_matrix(value):
    p = getattr(value, 'position', getattr(value, 'translation', None))
    q = getattr(value, 'orientation', getattr(value, 'rotation', None))
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-12 or not all(math.isfinite(v) for v in (p.x, p.y, p.z, x, y, z, w)):
        raise ValueError('Invalid pose')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = [p.x, p.y, p.z]
    return t


def pose_difference(a, b):
    shift = float(np.linalg.norm(a[:3, 3]-b[:3, 3]))
    rotation = a[:3, :3].T @ b[:3, :3]
    angle = math.acos(max(-1., min(1., (np.trace(rotation)-1)/2)))
    return shift, angle


def path_distance(point, points):
    if not points:
        return None
    xy = np.asarray(point[:2])
    if len(points) == 1:
        return float(np.linalg.norm(xy-np.asarray(points[0][:2])))
    best = math.inf
    for a, b in zip(points, points[1:]):
        a, b = np.asarray(a[:2]), np.asarray(b[:2])
        direction = b-a
        u = 0. if direction @ direction == 0 else max(0., min(1., float((xy-a)@direction/(direction@direction))))
        best = min(best, float(np.linalg.norm(xy-(a+u*direction))))
    return best


class TrialEvaluator:
    def __init__(self, goal=None, route_points=None):
        self.goal = list(goal) if goal is not None else None
        self.route_points = list(route_points or [])
        self.topic_counts = Counter()
        self.receipts, self.headers = defaultdict(list), defaultdict(list)
        self.statuses, self.poses, self.registrations, self.velocities = [], [], [], []
        self.odometry = []
        self.transforms = defaultdict(list)
        self.stages, self.events, self.terminal_vectors, self.costmaps = [], [], [], []
        self.invalid_messages = Counter()

    def observe(self, topic, type_name, message, receipt_ns):
        self.topic_counts[topic] += 1
        self.receipts[topic].append(int(receipt_ns))
        header = getattr(message, 'header', None)
        stamp = stamp_ns(header.stamp) if header else None
        if stamp is not None:
            self.headers[topic].append((receipt_ns, stamp))
        if topic == '/cloud_registered_body':
            if message.header.frame_id != 'body' or message.width*message.height == 0:
                self.invalid_messages['body_cloud_contract'] += 1
        elif topic == '/Odometry':
            if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
                self.invalid_messages['odom_frame_contract'] += 1
            else:
                self.odometry.append((receipt_ns, stamp, pose_matrix(message.pose.pose)))
        elif topic == '/localization/pose':
            if message.header.frame_id != 'map':
                self.invalid_messages['localization_pose_frame_contract'] += 1
            else:
                self.poses.append((receipt_ns, stamp, pose_matrix(message.pose)))
        elif topic in ('/tf', '/tf_static'):
            for transform in message.transforms:
                edge = (transform.header.frame_id, transform.child_frame_id)
                self.transforms[edge].append((receipt_ns, stamp_ns(transform.header.stamp),
                                              pose_matrix(transform.transform), topic == '/tf_static'))
        elif topic == '/localization/status':
            if int(message.state) not in range(5):
                self.invalid_messages['unknown_localization_state'] += 1
            self.statuses.append((receipt_ns, int(message.state), message.state_label, message.reason))
        elif topic == '/localization/registration_status':
            self.registrations.append({'receipt_ns': receipt_ns, 'source_stamp_ns': stamp_ns(message.stamp),
                                       'accepted': bool(message.accepted), 'stage': message.stage,
                                       'reason': message.reason, 'fitness': float(message.fitness),
                                       'overlap': float(message.overlap),
                                       'correspondences': int(message.correspondence_count),
                                       'elapsed_ms': float(message.elapsed_ms)})
            row = self.registrations[-1]
            row['nonfinite_fields'] = {k: str(v) for k, v in row.items()
                                       if isinstance(v, float) and not math.isfinite(v)}
            if row['accepted'] and row['nonfinite_fields']:
                self.invalid_messages['accepted_nonfinite_registration'] += 1
        elif topic == '/cmd_vel_guarded':
            velocity = (message.linear.x, message.linear.y, message.angular.z)
            if not all(math.isfinite(v) for v in velocity):
                self.invalid_messages['nonfinite_guarded_velocity'] += 1
            self.velocities.append((receipt_ns, velocity))
        elif topic == '/route_follower/raw_path' and message.header.frame_id == 'map' and message.poses:
            self.route_points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            if self.goal is None:
                final = pose_matrix(message.poses[-1].pose)
                self.goal = [float(final[0, 3]), float(final[1, 3]),
                             math.atan2(final[1, 0], final[0, 0])]
        elif topic == '/goal_pose' and message.header.frame_id == 'map' and self.goal is None:
            final = pose_matrix(message.pose)
            self.goal = [float(final[0, 3]), float(final[1, 3]), math.atan2(final[1, 0], final[0, 0])]
        elif topic == '/route_follower/terminal_error_body':
            self.terminal_vectors.append([receipt_ns, message.vector.x, message.vector.y, message.vector.z])
        elif topic == '/local_costmap/costmap':
            cells = Counter(int(c) for c in message.data)
            self.costmaps.append({'receipt_ns': receipt_ns, 'lethal_cells': sum(n for k, n in cells.items() if k >= 99),
                                  'inflated_cells': sum(n for k, n in cells.items() if 0 < k < 99)})
        elif topic in ('/a2w_validation/events', '/collision_monitor_state'):
            self.events.append({'receipt_ns': receipt_ns, 'topic': topic, 'value': str(message)[:1500]})
        elif topic == '/rosout':
            if 'offline_match_stages ' in message.msg:
                fields = dict(re.findall(r'(\w+)=([^\s]+)', message.msg))
                row = {'receipt_ns': receipt_ns}
                for key, value in fields.items():
                    if key.endswith('_ms'):
                        try:
                            row[key] = float(value)
                        except ValueError:
                            pass
                    else:
                        row[key] = value
                self.stages.append(row)

    @staticmethod
    def _series_stats(values):
        return statistics((b-a)/1e6 for a, b in zip(values, values[1:]))

    def summary(self):
        topics = {}
        for topic, count in self.topic_counts.items():
            receipts = self.receipts[topic]
            header = self.headers.get(topic, [])
            topics[topic] = {'count': count, 'receipt_interval_ms': self._series_stats(receipts),
                             'header_interval_ms': self._series_stats([v[1] for v in header]),
                             'receipt_minus_header_ms': statistics((a-b)/1e6 for a, b in header),
                             'receipt_rate_hz': (len(receipts)-1)*1e9/(receipts[-1]-receipts[0])
                             if len(receipts)>1 and receipts[-1]>receipts[0] else None}
        missing = [t for t in CORE_TOPICS if not self.topic_counts[t]]
        stages = {key: statistics(row[key] for row in self.stages if key in row)
                  for key in {k for row in self.stages for k in row if k.endswith('_ms')}}
        normal = [v for v in self.registrations if v['stage'] == 'localization']
        accepted = [v for v in normal if v['accepted']]
        task_accepts = [r for r in self.stages if str(r.get('accepted')) in ('1', 'true')]
        task_updates = ((len(task_accepts)-1)*1e9/(task_accepts[-1]['receipt_ns']-task_accepts[0]['receipt_ns'])
                        if len(task_accepts)>1 and task_accepts[-1]['receipt_ns']>task_accepts[0]['receipt_ns'] else None)
        times = [v[0] for v in self.statuses]
        states = []
        durations = defaultdict(float)
        end = max((t[-1] for t in self.receipts.values() if t), default=0)
        for i, row in enumerate(self.statuses):
            nxt = self.statuses[i+1][0] if i+1<len(self.statuses) else end
            durations[row[2]] += max(0, nxt-row[0])/1e9
            if not states or states[-1]['state'] != row[2]:
                states.append({'receipt_ns': row[0], 'state': row[2], 'reason': row[3]})
        fault_nonzero = fault_commands = nonzero = invalid_velocity = 0
        known_moving_before_fault = False
        stop_eligible = False
        for receipt, velocity in self.velocities:
            if not all(math.isfinite(v) for v in velocity):
                invalid_velocity += 1
                continue
            moving = any(abs(v)>1e-6 for v in velocity)
            nonzero += moving
            index = bisect.bisect_right(times, receipt)-1
            state = self.statuses[index][1] if index >= 0 else None
            if state == 1:
                known_moving_before_fault = moving
            elif state not in (0, 2, 3, 4):
                known_moving_before_fault = False
            if state in (0, 2, 3, 4):
                fault_commands += 1
                fault_nonzero += moving
                stop_eligible |= known_moving_before_fault
        stop = False if fault_nonzero else (True if stop_eligible and not invalid_velocity
                and not self.invalid_messages['unknown_localization_state'] else None)
        raw_tf = self.transforms.get(('camera_init', 'body'), [])
        def received_pose(rows, receipt, stamp):
            matches = [r for r in rows if r[1] == stamp and r[0] <= receipt]
            return max(matches, key=lambda r: r[0])[2] if matches else None
        all_map_tf = self.transforms.get(('map', 'camera_init'), [])
        # Ignore future-stamped corrections beyond the navigation TF tolerance.
        # Receipt order alone cannot make a malformed future transform current.
        map_tf = sorted((r for r in all_map_tf if r[1] <= r[0]+100_000_000), key=lambda row: row[0])
        extrinsic = sorted(self.transforms.get(('body', 'base_link'), []), key=lambda row: row[0])
        mtimes = [r[0] for r in map_tf]
        etimes = [r[0] for r in extrinsic]
        residuals, rotations, tf_residuals, tf_rotations = [], [], [], []
        exact = tf_exact = no_map = 0
        deviations, terminal, base_matrices = [], None, []
        for receipt, stamp, body in self.poses:
            index = bisect.bisect_right(mtimes, receipt)-1
            raw = received_pose(self.odometry, receipt, stamp)
            if raw is not None:
                exact += 1
                if index >= 0:
                    d, angle = pose_difference(map_tf[index][2]@raw, body)
                    residuals.append(d); rotations.append(angle)
                else:
                    no_map += 1
            raw = received_pose(raw_tf, receipt, stamp)
            if raw is not None:
                tf_exact += 1
                if index >= 0:
                    d, angle = pose_difference(map_tf[index][2]@raw, body)
                    tf_residuals.append(d); tf_rotations.append(angle)
            eindex = bisect.bisect_right(etimes, receipt)-1
            if eindex >= 0:
                base = body@extrinsic[eindex][2]
                base_matrices.append(base)
                if self.goal is not None:
                    yaw = math.atan2(base[1, 0], base[0, 0])
                    dx, dy = self.goal[0]-base[0, 3], self.goal[1]-base[1, 3]
                    terminal = {'goal_map': self.goal, 'robot_frame': 'base_link',
                                'receipt_ns': receipt, 'source_stamp_ns': stamp,
                                'distance_m': math.hypot(dx, dy),
                                'error_body_x_m': math.cos(yaw)*dx+math.sin(yaw)*dy,
                                'error_body_y_m': -math.sin(yaw)*dx+math.cos(yaw)*dy,
                                'error_yaw_rad': math.atan2(math.sin(self.goal[2]-yaw), math.cos(self.goal[2]-yaw))}
                deviation = path_distance(base[:3, 3], self.route_points)
                if deviation is not None:
                    deviations.append(deviation)
        jumps = [pose_difference(a[2], b[2]) for a, b in zip(self.poses, self.poses[1:])]
        raw_rows = sorted(self.odometry, key=lambda row: row[1])
        odom_jumps = [pose_difference(a[2], b[2])[0] for a, b in zip(raw_rows, raw_rows[1:])]
        required_missing = [t for t in ('/lidar_points', '/lidar_imu', '/tf_static')
                            if not self.topic_counts[t]]
        for edge in [('map', 'camera_init'), ('camera_init', 'body'), ('body', 'base_link')]:
            rows = map_tf if edge == ('map', 'camera_init') else self.transforms.get(edge)
            if not rows:
                required_missing.append('TF:'+'=>'.join(edge))
        frontend_pairs = len({r[1] for r in self.odometry} &
                             {r[1] for r in self.headers.get('/cloud_registered_body', [])})
        if not frontend_pairs:
            required_missing.append('same_stamp_frontend_pair')
        if not normal:
            required_missing.append('normal_registration')
        complete = not missing and not required_missing and not self.invalid_messages
        missing_navigation = [t for t in ('/route_follower/raw_path', '/route_follower/path',
                              '/route_follower/terminal_error_body', '/cmd_vel_nav',
                              '/cmd_vel_guarded', '/local_costmap/costmap') if not self.topic_counts[t]]
        return {'schema': 'a2w_trial_metrics_v1', 'topics': topics,
                'missing_core_topics': missing, 'core_topics_present': not missing,
                'missing_required_data': required_missing, 'required_data_complete': complete,
                'capture_complete': complete,
                'same_stamp_frontend_pairs': frontend_pairs,
                'capture_complete_scope': 'localization replay inputs and contracts; not navigation or physical accuracy',
                'navigation_data_complete': not missing_navigation,
                'missing_navigation_data': missing_navigation,
                'invalid_messages': dict(self.invalid_messages),
                'normal_registration': {'count': len(normal), 'accepted': len(accepted),
                                        'count_semantics': 'stage=localization status messages, including no-fit input rejection; actual tasks are separate',
                                        'actual_tasks_from_stage_logs': len(self.stages),
                                        'consumer_accepts_from_stage_logs': len(task_accepts),
                                        'accepted_update_hz_receipt': task_updates,
                                        'accepted_fraction': len(accepted)/len(normal) if normal else None,
                                        'reasons': dict(Counter(v['reason'] for v in normal)),
                                        'worker_elapsed_ms': statistics(v['elapsed_ms'] for v in normal),
                                        'fitness': statistics(v['fitness'] for v in normal),
                                        'overlap': statistics(v['overlap'] for v in normal)},
                'match_stages_ms': stages,
                'localization': {'transitions': states, 'receipt_hold_durations_s': dict(durations),
                                 'duration_semantics': 'hold last received status, not independent health truth'},
                'guarded_velocity': {'count': len(self.velocities), 'nonzero_count': nonzero,
                                     'fault_command_count': fault_commands, 'fault_state_nonzero_count': fault_nonzero,
                                     'nonfinite_count': invalid_velocity, 'known_localized_motion_before_fault': stop_eligible,
                                     'fault_stop_observed': stop, 'stop_latency_verified': False},
                'tf_pose_consistency': {'exact_odom_pairs': exact, 'missing_exact_odom': len(self.poses)-exact,
                                        'exact_raw_tf_pairs': tf_exact,
                                        'raw_tf_translation_residual_m': statistics(tf_residuals),
                                        'raw_tf_rotation_residual_rad': statistics(tf_rotations),
                                        'future_map_tf_excluded': len(all_map_tf)-len(map_tf),
                                        'missing_prior_map_tf': no_map,
                                        'translation_residual_m': statistics(residuals),
                                        'rotation_residual_rad': statistics(rotations),
                                        'semantics': 'same-source odom and last-received correction; correction publication phase may differ'},
                'pose_translation_step_m': statistics(v[0] for v in jumps),
                'pose_rotation_step_rad': statistics(v[1] for v in jumps),
                'odom_translation_step_m': statistics(odom_jumps),
                'route_deviation_estimated_m': statistics(deviations),
                'estimated_terminal': terminal, 'terminal_error_messages': len(self.terminal_vectors),
                'body_base_extrinsic_available': bool(extrinsic),
                'avoidance_evidence': {'event_count': len(self.events), 'costmap_count': len(self.costmaps),
                                       'lethal_cells': statistics(v['lethal_cells'] for v in self.costmaps)},
                'true_endpoint_error_m': None, 'accuracy_validation': 'independent_ground_truth_required',
                'notes': ['GICP, worker and input-to-status timings are distinct; percentiles cannot be added.',
                          'Endpoint/route errors use localization estimates, not independently measured robot positions.',
                          'A moving-fault stop verdict needs nonzero motion and a recorded fault.',
                          'No ROS context, publisher or motion request is created by evaluation.']}
