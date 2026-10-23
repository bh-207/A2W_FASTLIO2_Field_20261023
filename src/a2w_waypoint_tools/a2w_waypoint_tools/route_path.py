def terminal_error_body(pose, goal):
    """Goal translation in the current robot frame: +x forward, +y left."""
    dx, dy = goal[0]-pose[0], goal[1]-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c*dx+s*dy, -s*dx+c*dy, math.hypot(dx, dy)


def remaining_path_distance(pose, path, previous_progress_s=0.):
    """Remaining map-frame arc length from the nearest projected route segment."""
    if len(path) < 2:
        raise ValueError('路径至少需要两个点')
    total = progress = 0.
    nearest = math.inf
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length > 1.e-9:
            t = max(0., min(1., ((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(length*length)))
            distance = math.hypot(pose[0]-a[0]-t*dx, pose[1]-a[1]-t*dy)
            if distance < nearest:
                nearest, progress = distance, total+t*length
        total += length
    progress = min(total, max(previous_progress_s, progress))
    return max(0., total-progress), progress


"""A/B route preparation with fixed sampled anchors and bounded optional smoothing.

This is geometric processing, not a collision-aware optimizer or global planner.
"""
import csv
import math
from pathlib import Path

import yaml


def pose_discontinuity(previous, current, elapsed_s, *, translation_slack_m=.6,
                       yaw_slack_rad=.2, max_speed_mps=.8,
                       max_yaw_rate_rps=.75, max_interval_s=1.):
    """Flag a map/base pose step beyond motion plus bounded correction slack."""
    if (not all(math.isfinite(v) for v in (*previous, *current, elapsed_s)) or
            not 0. <= elapsed_s <= max_interval_s):
        return True
    translation = math.hypot(current[0]-previous[0], current[1]-previous[1])
    yaw = abs(math.atan2(math.sin(current[2]-previous[2]),
                         math.cos(current[2]-previous[2])))
    return (translation > translation_slack_m + max_speed_mps*elapsed_s or
            yaw > yaw_slack_rad + max_yaw_rate_rps*elapsed_s)


def rejoin_route(points, pose, segment_index, max_distance):
    """Explicitly rejoin one selected original segment, keeping its map goal.

    The connector and all remaining anchors still need live footprint checks.
    Never guess a global nearest segment or modify the caller's raw route.
    """
    if (isinstance(segment_index, bool) or not isinstance(segment_index, int) or
            not 0 <= segment_index < len(points)-1 or
            not math.isfinite(max_distance) or max_distance <= 0. or
            len(pose) != 3 or not all(math.isfinite(v) for v in pose)):
        raise ValueError('无效的显式重接段/位姿/距离')
    a, b = points[segment_index:segment_index+2]
    dx, dy = b[0]-a[0], b[1]-a[1]
    length_sq = dx*dx+dy*dy
    if length_sq <= 1.e-12:
        raise ValueError('选择的原始重接段没有平移长度')
    ratio = max(0., min(1., ((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length_sq))
    projected = (a[0]+ratio*dx, a[1]+ratio*dy, math.atan2(dy, dx))
    distance = math.hypot(pose[0]-projected[0], pose[1]-projected[1])
    if distance > max_distance:
        raise ValueError('距选择的原始重接段过远，请先确认定位并手控靠近')
    remaining = points[segment_index+1:]
    if distance <= 1.e-9:
        if ratio >= 1.-1.e-9 and len(remaining) > 1:
            remaining = remaining[1:]
        return [tuple(pose), *remaining]
    if ratio >= 1.-1.e-9:
        return [tuple(pose), *remaining]
    return [tuple(pose), projected, *remaining]


def load_route(filename, frame, robot_frame):
    path = Path(filename).expanduser()
    text = path.read_text(encoding='utf-8')
    if path.suffix.lower() in ('.yaml', '.yml'):
        doc = yaml.safe_load(text)
        if (not isinstance(doc, dict) or doc.get('schema') != 'a2w_waypoint_route_v1'
                or doc.get('frame_id') != frame or doc.get('robot_frame') != robot_frame):
            raise ValueError('路径文件 schema/frame/robot_frame 不匹配')
        points = [(float(p['position']['x']), float(p['position']['y']), float(p['yaw']))
                  for p in doc['waypoints']]
    elif path.suffix.lower() == '.csv':
        rows = list(csv.DictReader(text.splitlines()))
        if any(p['frame_id'] != frame for p in rows):
            raise ValueError('CSV frame_id 不匹配')
        points = [(float(p['x']), float(p['y']), float(p['yaw'])) for p in rows]
    else:
        # Legacy three-column files require the operator to confirm map/base_link.
        # Do not silently join GO/BACK segments or substitute a default goal.
        points = [tuple(float(v.strip()) for v in line.split(','))
                  for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#')]
    if len(points) < 2 or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('路径至少需要两个有效的 x,y,yaw 点')
    return points


def build_path(points, spacing=.2, window=5, passes=0):
    if (len(points) < 2 or not math.isfinite(spacing) or spacing <= 0 or
            not isinstance(window, int) or not 1 <= window <= 101 or
            not isinstance(passes, int) or not 0 <= passes <= 20 or
            any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points)):
        raise ValueError('无效路径或加密/平滑参数')
    lengths = [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])]
    if not any(length > 1e-6 for length in lengths):
        raise ValueError('路径没有有效平移长度')
    counts = [max(1, math.ceil(length / spacing)) for length in lengths]
    if sum(counts) > 100000:
        raise ValueError('路径点过多，请检查单位/采样间距')

    # Densify each original segment separately; its endpoint is never replaced.
    dense = [points[0][:2]]
    anchors = {0}
    for a, b, count in zip(points, points[1:], counts):
        for step in range(1, count):
            ratio = step / count
            dense.append((a[0] + ratio * (b[0]-a[0]),
                          a[1] + ratio * (b[1]-a[1])))
        dense.append(b[:2])
        anchors.add(len(dense)-1)

    if passes:
        reference = dense[:]
        window += 1 if window % 2 == 0 else 0
        radius = window // 2
        for _ in range(passes):
            padded = [dense[0]]*radius + dense + [dense[-1]]*radius
            smoothed = dense[:]
            for i in range(len(dense)):
                if i in anchors:
                    continue
                candidate = tuple(sum(p[j] for p in padded[i:i+window])/window
                                  for j in (0, 1))
                dx = candidate[0] - reference[i][0]
                dy = candidate[1] - reference[i][1]
                scale = min(1.0, .03 / math.hypot(dx, dy)) if dx or dy else 1.0
                smoothed[i] = (reference[i][0] + dx*scale,
                               reference[i][1] + dy*scale)
            dense = smoothed

    result = [(x, y, math.atan2(dense[i+1][1]-y, dense[i+1][0]-x))
              for i, (x, y) in enumerate(dense[:-1])]
    return result + [(dense[-1][0], dense[-1][1], points[-1][2])]


def distance_to_polyline(point, path):
    if not path:
        raise ValueError('路径为空')
    if len(path) == 1:
        return math.hypot(point[0]-path[0][0], point[1]-path[0][1])
    distances = []
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length_sq = dx*dx + dy*dy
        ratio = ((point[0]-a[0])*dx + (point[1]-a[1])*dy) / length_sq if length_sq else 0.
        ratio = max(0., min(1., ratio))
        distances.append(math.hypot(point[0]-a[0]-ratio*dx,
                                    point[1]-a[1]-ratio*dy))
    return min(distances)


def goal_error(pose, goal):
    dx, dy = pose[0]-goal[0], pose[1]-goal[1]
    dyaw = math.atan2(math.sin(pose[2]-goal[2]), math.cos(pose[2]-goal[2]))
    return dx, dy, math.hypot(dx, dy), dyaw


def ready_to_follow(*, localized, nav_ready, safety, bridge, now, timeout=1.):
    return bool(localized and nav_ready and math.isfinite(now) and all(
        flag is True and 0 <= now-stamp <= timeout for flag, stamp in (safety, bridge)))


def check_centerline(points, grid, frame):
    """Reject outside, unknown and inscribed/lethal CENTER cells, including gaps.

    Nav2 IsPathValid must additionally check its current configured footprint.
    This function does not replace that service or the live local costmap.
    """
    m = grid.metadata
    q = m.origin.orientation
    if (grid.header.frame_id != frame or m.resolution <= 0 or
            m.size_x*m.size_y != len(grid.data) or abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or
            abs(abs(q.w)-1.) > 1e-6):
        raise ValueError('global costmap frame/尺寸/origin 不受支持')
    for a, b in zip(points, points[1:]):
        steps = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/(m.resolution*.5)))
        for i in range(steps+1):
            x = a[0] + (b[0]-a[0])*i/steps
            y = a[1] + (b[1]-a[1])*i/steps
            mx = math.floor((x-m.origin.position.x)/m.resolution)
            my = math.floor((y-m.origin.position.y)/m.resolution)
            if not 0 <= mx < m.size_x or not 0 <= my < m.size_y:
                raise ValueError(f'路径超出 costmap：({x:.2f},{y:.2f})')
            if grid.data[my*m.size_x+mx] >= 253:
                raise ValueError(f'路径经过障碍/不可通行/未知单元：({x:.2f},{y:.2f})')
