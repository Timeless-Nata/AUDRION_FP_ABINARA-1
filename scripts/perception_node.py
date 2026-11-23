#!/usr/bin/env python3
"""Phase 4 perception: colour blobs in the camera image -> object positions in the world frame.

Inputs   /camera/image_raw (rgb8), /camera/camera_info, /camera/pose (optical frame in world),
         /hcsr04/range, /hcsr04/pose
Outputs  /perception/objects      autonomous_manipulator/DetectedObjectArray
         /perception/debug_image  sensor_msgs/Image (rgb8) with the detections drawn in
Service  /perception/dump_frame   std_srvs/Trigger: saves the last raw + annotated frame as PNG and
                                  prints the median HSV of every blob (use it to tune camera_config.yaml)

Localisation: the blob centroid is turned into a ray (pinhole model from camera_info), moved into the
world with the camera pose, and cut with the plane z = object_center_z. Detections that fall outside the
arena half-disc are dropped, which also removes colour false positives on the floor / background.
No cv_bridge: images are converted with numpy.
"""
import collections
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Range
from std_srvs.srv import Trigger, TriggerResponse

from autonomous_manipulator.msg import DetectedObject, DetectedObjectArray


# ----------------------------------------------------------------------------- pure helpers
def image_to_rgb(msg):
    """sensor_msgs/Image (rgb8 or bgr8) -> HxWx3 uint8 RGB array."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError("unsupported encoding %s" % msg.encoding)
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[:, :, ::-1].copy()


def rgb_to_image(rgb, stamp, frame_id):
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height, m.width = rgb.shape[:2]
    m.encoding = "rgb8"
    m.step = m.width * 3
    m.data = np.ascontiguousarray(rgb).tobytes()
    return m


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def detect_blobs(rgb, colors, kernel_px, min_area, max_area_frac=1.0):
    """Return [{color,u,v,area,w,h}] with (u,v) in continuous pixel coords (pixel i covers [i, i+1))."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    out = []
    max_area = max_area_frac * rgb.shape[0] * rgb.shape[1]  # rejects floor-sized regions
    for name, spec in colors.items():
        if "v_max" in spec:  # brightness-only rule, as used for black on the real robot
            mask = cv2.inRange(hsv[:, :, 2], 0, int(spec["v_max"]))
        else:
            mask = cv2.inRange(hsv, np.array(spec["lo"], np.uint8), np.array(spec["hi"], np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < min_area or area > max_area:
                continue
            out.append({"color": name, "u": float(cents[i][0]) + 0.5, "v": float(cents[i][1]) + 0.5, "area": area,
                        "w": int(stats[i, cv2.CC_STAT_WIDTH]), "h": int(stats[i, cv2.CC_STAT_HEIGHT]),
                        "x0": int(stats[i, cv2.CC_STAT_LEFT]), "y0": int(stats[i, cv2.CC_STAT_TOP])})
    return out


def backproject(u, v, K, R_wc, p_wc, plane_z):
    """Pixel -> world point on the plane z = plane_z. R_wc/p_wc: optical frame in world. None if no hit."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    d = R_wc @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    if d[2] >= -1e-9:  # ray does not go down toward the plane
        return None
    t = (plane_z - p_wc[2]) / d[2]
    return p_wc + t * d if t > 0 else None


def range_check(X, sensor_pos, sensor_R, measured, gate, radius, tol):
    """(checked, ok, expected_surface_range). The sensor looks along its +z axis."""
    axis = sensor_R[:, 2]
    v = X - sensor_pos
    t = float(v @ axis)
    if t <= 0.0 or np.linalg.norm(v - t * axis) > gate:
        return False, False, float("nan")
    expected = max(t - radius, 0.0)
    ok = math.isfinite(measured) and abs(measured - expected) <= tol
    return True, ok, expected


# ----------------------------------------------------------------------------- node
class Perception:
    def __init__(self):
        env = rospy.get_param("~environment", "simulation")
        cfg = rospy.get_param("~" + env)
        self.colors = cfg["colors"]
        self.min_area = int(cfg.get("min_blob_area_px", 150))
        self.kernel = int(cfg.get("merge_kernel_px", 7)) | 1
        self.max_frac = float(cfg.get("max_blob_area_frac", 0.15))
        self.plane_z = float(cfg.get("object_center_z", 0.0305))
        self.radius = float(cfg.get("object_radius_m", 0.03))
        arena = cfg.get("arena", {})
        self.arena_r = float(arena.get("radius_m", 0.37))
        self.arena_min_x = float(arena.get("min_x_m", -0.03))
        self.gate = float(cfg.get("range_gate_m", 0.04))
        self.tol = float(cfg.get("range_tol_m", 0.04))
        self.dump_dir = os.path.expanduser(rospy.get_param("~dump_dir", "~/.ros/perception_dumps"))

        self.lock = threading.Lock()
        self.K = None
        self.poses = collections.deque(maxlen=30)  # (stamp_sec, R, p)
        self.range_msg = None
        self.range_pose = None
        self.last = None  # (rgb, debug_rgb, blobs)

        self.pub = rospy.Publisher("/perception/objects", DetectedObjectArray, queue_size=5)
        self.dbg_pub = rospy.Publisher("/perception/debug_image", Image, queue_size=1)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber("/camera/pose", PoseStamped, self.on_cam_pose, queue_size=30)
        rospy.Subscriber("/hcsr04/range", Range, self.on_range, queue_size=1)
        rospy.Subscriber("/hcsr04/pose", PoseStamped, self.on_range_pose, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Service("/perception/dump_frame", Trigger, self.on_dump)
        rospy.loginfo("perception [%s]: colours %s, min area %d px, plane z %.4f m",
                      env, sorted(self.colors), self.min_area, self.plane_z)

    @staticmethod
    def _pose(msg):
        q, p = msg.pose.orientation, msg.pose.position
        return quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])

    def on_info(self, m):
        with self.lock:
            self.K = list(m.K)

    def on_cam_pose(self, m):
        R, p = self._pose(m)
        with self.lock:
            self.poses.append((m.header.stamp.to_sec(), R, p))

    def on_range(self, m):
        with self.lock:
            self.range_msg = m

    def on_range_pose(self, m):
        with self.lock:
            self.range_pose = self._pose(m)

    def _nearest_pose(self, t):
        best = min(self.poses, key=lambda e: abs(e[0] - t), default=None)
        return best if best is not None and abs(best[0] - t) < 0.1 else None

    def on_image(self, msg):
        with self.lock:
            K, pose = self.K, self._nearest_pose(msg.header.stamp.to_sec())
            rng, rpose = self.range_msg, self.range_pose
        if K is None or pose is None:
            rospy.logwarn_throttle(5.0, "perception: waiting for camera_info / camera pose")
            return
        rgb = image_to_rgb(msg)
        blobs = detect_blobs(rgb, self.colors, self.kernel, self.min_area, self.max_frac)
        _, R, p = pose
        measured = float(rng.range) if rng is not None else float("inf")

        out = DetectedObjectArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "world"
        out.range_measured = measured
        dbg = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        kept = []
        for b in blobs:
            X = backproject(b["u"], b["v"], K, R, p, self.plane_z)
            if X is None or math.hypot(X[0], X[1]) > self.arena_r or X[0] < self.arena_min_x:
                cv2.rectangle(dbg, (b["x0"], b["y0"]), (b["x0"] + b["w"], b["y0"] + b["h"]), (90, 90, 90), 1)
                continue  # off the table / not on the plane: reject
            o = DetectedObject()
            o.color = b["color"]
            o.position.x, o.position.y, o.position.z = float(X[0]), float(X[1]), float(X[2])
            o.pixel_u, o.pixel_v, o.area_px = b["u"], b["v"], float(b["area"])
            o.bbox_w_px, o.bbox_h_px = float(b["w"]), float(b["h"])
            o.range_checked, o.range_ok, o.range_expected = False, False, float("nan")
            if rpose is not None and rng is not None:
                o.range_checked, o.range_ok, o.range_expected = range_check(
                    X, rpose[1], rpose[0], measured, self.gate, self.radius, self.tol)
            out.objects.append(o)
            kept.append(b)
            cv2.rectangle(dbg, (b["x0"], b["y0"]), (b["x0"] + b["w"], b["y0"] + b["h"]), (0, 255, 0), 2)
            cv2.putText(dbg, "%s (%.3f, %.3f)" % (o.color, X[0], X[1]), (b["x0"], max(12, b["y0"] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        self.pub.publish(out)
        dbg_rgb = cv2.cvtColor(dbg, cv2.COLOR_BGR2RGB)
        self.dbg_pub.publish(rgb_to_image(dbg_rgb, msg.header.stamp, msg.header.frame_id))
        with self.lock:
            self.last = (rgb, dbg_rgb, kept)

    def on_dump(self, _req):
        with self.lock:
            last = self.last
        if last is None:
            return TriggerResponse(False, "no frame received yet")
        rgb, dbg, blobs = last
        os.makedirs(self.dump_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        raw_path = os.path.join(self.dump_dir, "frame_%s.png" % stamp)
        dbg_path = os.path.join(self.dump_dir, "frame_%s_debug.png" % stamp)
        cv2.imwrite(raw_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(dbg_path, cv2.cvtColor(dbg, cv2.COLOR_RGB2BGR))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        lines = []
        for b in blobs:
            patch = hsv[b["y0"]:b["y0"] + b["h"], b["x0"]:b["x0"] + b["w"]].reshape(-1, 3)
            med = np.median(patch, axis=0)
            lines.append("%s: median HSV (%d, %d, %d), bbox %dx%d" % (b["color"], med[0], med[1], med[2], b["w"], b["h"]))
        msg = "saved %s ; %s" % (raw_path, "; ".join(lines) if lines else "no blobs")
        rospy.loginfo(msg)
        return TriggerResponse(True, msg)


if __name__ == "__main__":
    rospy.init_node("perception")
    Perception()
    rospy.spin()
