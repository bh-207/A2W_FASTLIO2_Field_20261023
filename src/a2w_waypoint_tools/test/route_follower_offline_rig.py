"""Compile real RouteFollower methods with offline transport/clock boundaries.

No ROS imports/init, controller, node or hardware transport. Test utility only.
"""
import ast
import copy
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

from a2w_waypoint_tools import route_path
from a2w_waypoint_tools.route_path import build_path, distance_to_polyline, goal_error, ready_to_follow, remaining_path_distance, terminal_error_body
from a2w_waypoint_tools.route_storage import quaternion_yaw

ROOT = Path(__file__).resolve().parents[3]

def stamp(seconds):
    sec = math.floor(seconds)
    return NS(sec=sec, nanosec=round((seconds-sec)*1.e9))


class ClockTime:
    def __init__(self, seconds=0.):
        self.nanoseconds = round(seconds*1.e9)

    @classmethod
    def from_msg(cls, value):
        return cls(value.sec + value.nanosec*1.e-9)

    def to_msg(self):
        return stamp(self.nanoseconds*1.e-9)


class PoseStamped:
    def __init__(self):
        self.header = NS(frame_id='', stamp=stamp(0.))
        self.pose = NS(position=NS(x=0., y=0., z=0.),
                       orientation=NS(x=0., y=0., z=0., w=1.))


class PathMessage:
    def __init__(self):
        self.header = NS(frame_id='', stamp=stamp(0.))
        self.poses = []


class Vector3Stamped:
    def __init__(self):
        self.header = NS(frame_id='', stamp=stamp(0.))
        self.vector = NS(x=0., y=0., z=0.)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class Rig:
    def __init__(self):
        self.now = 1.
        self.pose = (0., 0., 0.)
        self.tf_age = 0.
        self.logs, self.tf_lookups, self.cancels, self.disables = [], [], [], []
        source = ROOT/'src/a2w_waypoint_tools/a2w_waypoint_tools/route_follower.py'
        tree = ast.parse(source.read_text())
        klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RouteFollower')
        space = dict(Node=object, math=math, json=json, sys=sys,
            time=NS(monotonic=lambda: self.now), Time=ClockTime,
            TransformException=RuntimeError, Path=PathMessage,
            PoseStamped=PoseStamped, Vector3Stamped=Vector3Stamped,
            LocalizationStatus=NS(LOCALIZED=1),
            SetBool=NS(Request=lambda **kw: NS(**kw)),
            build_path=build_path, distance_to_polyline=distance_to_polyline,
            goal_error=goal_error, quaternion_yaw=quaternion_yaw,
            ready_to_follow=ready_to_follow, remaining_path_distance=remaining_path_distance,
            terminal_error_body=terminal_error_body)
        space.update({name: getattr(route_path, name) for name in ('pose_discontinuity', 'rejoin_route')
                      if hasattr(route_path, name)})
        space['GetCostmap'] = NS(Request=lambda: NS())
        exec(compile(ast.Module(body=[klass], type_ignores=[]), str(source), 'exec'), space)
        self.node = n = object.__new__(space['RouteFollower'])
        n.pose_jump_translation_m, n.pose_jump_yaw_rad = .6, .2
        n.pose_jump_max_speed_mps, n.pose_jump_max_yaw_rate_rps = .8, .75
        n.last_nav_pose = None
        n.rejoin_segment_index = -1
        n.spacing_m, n.smooth_window, n.smooth_passes = .2, 5, 0
        n.start_tolerance = .5
        n.preview = Publisher()
        n.action = NS(server_is_ready=lambda: True)
        self.grid_requests = []
        n.grid_client = NS(service_is_ready=lambda: True, call_async=lambda req: NS(add_done_callback=lambda cb: self.grid_requests.append(cb)))
        n.valid_client = NS(service_is_ready=lambda: True)
        n.get_clock = lambda: NS(now=lambda: ClockTime(self.now))
        n.get_logger = lambda: NS(**{level: self.logs.append for level in ['info', 'warning', 'error']})
        n.frame, n.robot_frame = 'map', 'base_link'
        n.raw_points = [(0., 0., 0.), (5.4, 0., 0.)]
        n.points = build_path(n.raw_points)
        n.path = n._path(n.points)
        n.trace, n.trace_pub, n.terminal_error_pub = PathMessage(), Publisher(), Publisher()
        n.trace.header.frame_id = 'map'
        n.remaining_m, n.progress_s = None, 0.
        n.deviation_samples = 0
        n.max_raw_deviation_m = n.raw_deviation_sum_m = 0.
        n.max_path_deviation_m = n.path_deviation_sum_m = 0.
        n.done, n.phase, n.phase_since, n.started = False, 'RUNNING', 1., 0.
        n.wait_timeout, n.request_timeout, n.tf_max_age = 120., 5., 1.
        n.exit_code, n.cancel_requested, n.last_trace = 0, False, 0.
        n.localized, n.nav_ready = True, True
        n.safety = n.bridge = (True, 1.)
        n.busy = {'nav': False, 'follow': True}
        n.send_future = object()
        n.goal_handle = NS(accepted=True, cancel_goal_async=lambda: self.cancels.append(self.now))
        n.disable_client = NS(service_is_ready=lambda: True,
            call_async=lambda req: self.disables.append(req.data))
        n.buffer = NS(lookup_transform=self.lookup_transform)

    def lookup_transform(self, target, source, _time):
        self.tf_lookups.append((target, source))
        x, y, yaw = self.pose
        return NS(header=NS(frame_id='map', stamp=stamp(self.now-self.tf_age)),
                  child_frame_id='base_link', transform=NS(
                      translation=NS(x=x, y=y, z=0.),
                      rotation=NS(x=0., y=0., z=math.sin(yaw/2.), w=math.cos(yaw/2.))))

    def tick(self, now, pose):
        self.now, self.pose = now, pose
        self.node.safety = self.node.bridge = (True, now)
        self.node._tick()
