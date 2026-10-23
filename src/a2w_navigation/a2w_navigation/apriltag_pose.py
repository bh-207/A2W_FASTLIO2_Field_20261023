"""Validated A2W AprilTag IPPE pose core, extracted unchanged from the existing v4 tool.
Camera/pose constants retain their original values; no motion or SDK controller is included.
"""
import math
import time
from collections import deque
import numpy as np

CALIB_WIDTH = 1920


CALIB_HEIGHT = 1080


K_RAW = np.array([
    [800.0149026,   0.0,          946.22247395],
    [0.0,           797.45244836, 587.90242696],
    [0.0,           0.0,            1.0],
], dtype=np.float64)


DIST = np.array([
    -2.71975620e-01,
     1.17399571e-01,
     2.22729018e-04,
     8.66553177e-04,
    -3.45216664e-02,
], dtype=np.float64)


K_NEW = np.array([
    [799.27410889, 0.0,          945.34630094],
    [0.0,          797.03704834, 587.59620493],
    [0.0,          0.0,            1.0],
], dtype=np.float64)


TAG_FAMILY = "tagStandard41h12"


DEFAULT_TAG_ID = 1


DEFAULT_TAG_SIZE_M = 0.100


DEFAULT_CAMERA_XYZ = np.array([0.3381, 0.0336, 0.0525], dtype=np.float64)


DEFAULT_CAMERA_RPY = np.array([0.0, 0.0, 0.0], dtype=np.float64)


OFFICIAL_A2_URDF_SOURCE = (
    "https://github.com/unitreerobotics/unitree_ros/"
    "blob/master/robots/a2_description/urdf/a2.urdf"
)


MIN_DECISION_MARGIN = 25.0


MIN_TAG_EDGE_PX = 22.0


MAX_REPROJ_PX = 3.0


MAX_FRAME_TRANSLATION_JUMP_M = 0.25


MAX_FRAME_ROTATION_JUMP_DEG = 15.0


CONTINUITY_RESET_S = 0.60


POSE_SMOOTH_WINDOW = 5


KX = 0.70


KY = 0.70


KYAW = 1.00


MAX_VX = 0.15


MAX_VY = 0.12


MAX_WZ = 0.25


POS_TOL_X = 0.03


POS_TOL_Y = 0.03


YAW_TOL_DEG = 2.0


ROTATE_FIRST_DEG = 25.0


TAG_LOST_STOP_S = 0.20


MIN_GOOD_FRAMES_BEFORE_MOTION = 5


ARRIVED_COUNT_REQUIRED = 10


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def rpy_to_R(rpy):
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr,  cr],
    ], dtype=np.float64)

    Ry = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp],
    ], dtype=np.float64)

    Rz = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1],
    ], dtype=np.float64)

    return Rz @ Ry @ Rx


def make_T(R=None, t=None):
    T = np.eye(4, dtype=np.float64)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def yaw_from_R(R):
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def rotation_distance_rad(Ra, Rb):
    R = np.asarray(Ra).T @ np.asarray(Rb)
    c = (np.trace(R) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    return math.acos(c)


def project_rotation(R):
    U, _, Vt = np.linalg.svd(np.asarray(R, dtype=np.float64))
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt
    return Rn


def average_transforms(T_list):
    if not T_list:
        raise ValueError("T_list is empty")

    ts = np.stack([T[:3, 3] for T in T_list], axis=0)
    Rs = np.stack([T[:3, :3] for T in T_list], axis=0)

    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = np.mean(ts, axis=0)
    T[:3, :3] = project_rotation(np.mean(Rs, axis=0))
    return T


def robust_average_transforms(T_list, trans_gate_m=0.05, rot_gate_rad=math.radians(5.0)):
    """
    两阶段鲁棒平均：
    1) 全部样本平均得到 provisional；
    2) 去掉与 provisional 相差过大的样本，再平均。
    """
    if len(T_list) < 3:
        return average_transforms(T_list), list(T_list)

    provisional = average_transforms(T_list)
    kept = []

    for T in T_list:
        dt = np.linalg.norm(T[:3, 3] - provisional[:3, 3])
        dr = rotation_distance_rad(T[:3, :3], provisional[:3, :3])
        if dt <= trans_gate_m and dr <= rot_gate_rad:
            kept.append(T)

    if len(kept) < max(3, int(0.7 * len(T_list))):
        raise RuntimeError(
            f"teach pose 不稳定：仅 {len(kept)}/{len(T_list)} 帧通过稳定性筛选"
        )

    return average_transforms(kept), kept


def build_base_from_camera_optical(camera_xyz, camera_rpy):
    """
    URDF camera_link:
      x前 y左 z上
    optical:
      x右 y下 z前
    """
    T_base_camera_link = make_T(
        rpy_to_R(camera_rpy),
        camera_xyz,
    )

    R_camera_link_optical = np.array([
        [ 0.0,  0.0,  1.0],
        [-1.0,  0.0,  0.0],
        [ 0.0, -1.0,  0.0],
    ], dtype=np.float64)

    return T_base_camera_link @ make_T(R_camera_link_optical, [0, 0, 0])


def make_tag_object_points(tag_size_m):
    """
    OpenCV SOLVEPNP_IPPE_SQUARE 要求的顺序：
      [-L/2, +L/2, 0]
      [+L/2, +L/2, 0]
      [+L/2, -L/2, 0]
      [-L/2, -L/2, 0]

    与 AprilTag homography 的理想角顺序：
      (-1,+1), (+1,+1), (+1,-1), (-1,-1)
    一致。
    """
    s = float(tag_size_m) * 0.5
    return np.array([
        [-s, +s, 0.0],
        [+s, +s, 0.0],
        [+s, -s, 0.0],
        [-s, -s, 0.0],
    ], dtype=np.float64)


def homography_image_points(H):
    """
    从 AprilTag detection.homography 生成与 IPPE_SQUARE 对应的 4 个像素角点。
    使用 homography 而不是依赖 corners[0] 的起始角定义。
    """
    H = np.asarray(H, dtype=np.float64).reshape(3, 3)
    ideal = np.array([
        [-1.0, +1.0, 1.0],
        [+1.0, +1.0, 1.0],
        [+1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0],
    ], dtype=np.float64).T

    p = H @ ideal
    p[:2, :] /= p[2:3, :]
    return p[:2, :].T.copy()


def quad_min_edge_px(image_points):
    pts = np.asarray(image_points, dtype=np.float64).reshape(4, 2)
    d = []
    for i in range(4):
        d.append(np.linalg.norm(pts[(i + 1) % 4] - pts[i]))
    return float(min(d))


def candidate_reprojection_error_px(obj, image_points, rvec, tvec, K, dist):
    import cv2

    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)

    errs = np.linalg.norm(proj - image_points, axis=1)
    return float(np.sqrt(np.mean(errs * errs)))


def solve_ippe_candidates(image_points, tag_size_m, K, dist):
    import cv2

    obj = make_tag_object_points(tag_size_m)
    img = np.asarray(image_points, dtype=np.float64).reshape(4, 2)

    result = cv2.solvePnPGeneric(
        obj,
        img,
        np.asarray(K, dtype=np.float64),
        np.asarray(dist, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if not result or not bool(result[0]):
        return []

    rvecs = result[1]
    tvecs = result[2]

    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)

        if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
            continue
        if tvec[2] <= 0.0:
            continue

        R, _ = cv2.Rodrigues(rvec)
        reproj = candidate_reprojection_error_px(
            obj, img, rvec, tvec.reshape(3, 1), K, dist
        )

        candidates.append({
            "rvec": rvec,
            "tvec": tvec,
            "T_camera_tag": make_T(R, tvec),
            "reproj_px": reproj,
        })

    candidates.sort(key=lambda c: c["reproj_px"])
    return candidates


def select_pose_candidate(
    candidates,
    prev_T_camera_tag=None,
    max_translation_jump_m=MAX_FRAME_TRANSLATION_JUMP_M,
    max_rotation_jump_rad=math.radians(MAX_FRAME_ROTATION_JUMP_DEG),
    max_reproj_px=MAX_REPROJ_PX,
):
    valid_reproj = [
        c for c in candidates
        if np.isfinite(c["reproj_px"]) and c["reproj_px"] <= max_reproj_px
    ]

    if not valid_reproj:
        return None, "reprojection_rejected"

    if prev_T_camera_tag is None:
        return min(valid_reproj, key=lambda c: c["reproj_px"]), "accepted"

    continuous = []
    for c in valid_reproj:
        T = c["T_camera_tag"]
        dt = float(np.linalg.norm(
            T[:3, 3] - prev_T_camera_tag[:3, 3]
        ))
        dr = rotation_distance_rad(
            prev_T_camera_tag[:3, :3],
            T[:3, :3],
        )
        c["translation_jump_m"] = dt
        c["rotation_jump_rad"] = dr

        if (
            dt <= max_translation_jump_m
            and dr <= max_rotation_jump_rad
        ):
            continuous.append(c)

    if not continuous:
        return None, "continuity_rejected"

    # 在满足连续性的分支中，优先最小重投影误差。
    return min(continuous, key=lambda c: c["reproj_px"]), "accepted"


class PoseTracker:
    def __init__(
        self,
        max_translation_jump_m=MAX_FRAME_TRANSLATION_JUMP_M,
        max_rotation_jump_rad=math.radians(MAX_FRAME_ROTATION_JUMP_DEG),
        max_reproj_px=MAX_REPROJ_PX,
        reset_after_s=CONTINUITY_RESET_S,
        smooth_window=POSE_SMOOTH_WINDOW,
    ):
        self.max_translation_jump_m = max_translation_jump_m
        self.max_rotation_jump_rad = max_rotation_jump_rad
        self.max_reproj_px = max_reproj_px
        self.reset_after_s = reset_after_s

        self.prev_T_camera_tag = None
        self.last_accept_time = None
        self.history = deque(maxlen=max(1, int(smooth_window)))

    def reset(self):
        self.prev_T_camera_tag = None
        self.last_accept_time = None
        self.history.clear()

    def update(self, candidates, now):
        if (
            self.last_accept_time is not None
            and now - self.last_accept_time > self.reset_after_s
        ):
            self.reset()

        selected, reason = select_pose_candidate(
            candidates,
            prev_T_camera_tag=self.prev_T_camera_tag,
            max_translation_jump_m=self.max_translation_jump_m,
            max_rotation_jump_rad=self.max_rotation_jump_rad,
            max_reproj_px=self.max_reproj_px,
        )

        if selected is None:
            return None, None, reason

        T = selected["T_camera_tag"]
        self.prev_T_camera_tag = T.copy()
        self.last_accept_time = now
        self.history.append(T.copy())

        smooth = average_transforms(list(self.history))
        return selected, smooth, "accepted"


class AprilTagIppeEstimator:
    def __init__(
        self,
        tag_id,
        tag_size_m,
        T_base_camera_optical,
        min_decision_margin=MIN_DECISION_MARGIN,
        min_edge_px=MIN_TAG_EDGE_PX,
        max_reproj_px=MAX_REPROJ_PX,
        max_translation_jump_m=MAX_FRAME_TRANSLATION_JUMP_M,
        max_rotation_jump_deg=MAX_FRAME_ROTATION_JUMP_DEG,
        continuity_reset_s=CONTINUITY_RESET_S,
        smooth_window=POSE_SMOOTH_WINDOW,
    ):
        from pupil_apriltags import Detector

        self.tag_id = int(tag_id)
        self.tag_size_m = float(tag_size_m)
        self.T_base_camera_optical = np.asarray(
            T_base_camera_optical, dtype=np.float64
        )

        self.min_decision_margin = float(min_decision_margin)
        self.min_edge_px = float(min_edge_px)

        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.tracker = PoseTracker(
            max_translation_jump_m=max_translation_jump_m,
            max_rotation_jump_rad=math.radians(max_rotation_jump_deg),
            max_reproj_px=max_reproj_px,
            reset_after_s=continuity_reset_s,
            smooth_window=smooth_window,
        )

    def reset(self):
        self.tracker.reset()

    def detect(self, frame, now=None):
        import cv2

        if now is None:
            now = time.monotonic()

        h, w = frame.shape[:2]
        if (w, h) != (CALIB_WIDTH, CALIB_HEIGHT):
            raise RuntimeError(
                f"相机分辨率 {w}x{h} 与标定分辨率 "
                f"{CALIB_WIDTH}x{CALIB_HEIGHT} 不一致，拒绝计算 pose。"
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=False,
        )

        target = None
        for d in detections:
            if d.tag_id != self.tag_id:
                continue
            if d.hamming != 0:
                continue
            if d.decision_margin < self.min_decision_margin:
                continue
            if target is None or d.decision_margin > target.decision_margin:
                target = d

        if target is None:
            return None, {
                "reason": "tag_not_found",
                "detection": None,
            }

        image_points = homography_image_points(target.homography)
        min_edge = quad_min_edge_px(image_points)

        if min_edge < self.min_edge_px:
            return None, {
                "reason": "tag_too_small",
                "detection": target,
                "min_edge_px": min_edge,
            }

        candidates = solve_ippe_candidates(
            image_points,
            self.tag_size_m,
            K_RAW,
            DIST,
        )

        selected, T_camera_tag_smooth, reason = self.tracker.update(
            candidates,
            now,
        )

        if selected is None:
            return None, {
                "reason": reason,
                "detection": target,
                "min_edge_px": min_edge,
                "candidate_count": len(candidates),
                "candidates": candidates,
            }

        T_base_tag = self.T_base_camera_optical @ T_camera_tag_smooth
        T_tag_base = np.linalg.inv(T_base_tag)

        return T_tag_base, {
            "reason": "accepted",
            "detection": target,
            "min_edge_px": min_edge,
            "candidate_count": len(candidates),
            "selected": selected,
            "T_camera_tag_smooth": T_camera_tag_smooth,
            "T_base_tag": T_base_tag,
        }
