"""Phase 3 kinematics, built directly from the MJCF (single source of truth, no hand-copied numbers).

Pure numpy, no ROS and no MuJoCo import, so it runs (and is unit-tested) anywhere.

Robot structure (read from arm.xml):
    joint_1  yaw about world z                      (base)
    joint_2, joint_3, joint_4  three PARALLEL pitch axes (planar 3R arm in the vertical plane
                                                      that contains the joint_1 axis)
    tool     +x of revolute_4 = gripper / camera / HC-SR04 heading; grasp_center site is on it.

Conventions
    * Frames are world frame (MJCF world). Angles in radians.
    * Tool "pitch" = elevation of the tool +x axis above the horizontal plane
      (0 = horizontal, -pi/2 = pointing straight down).
    * A 4-DOF arm cannot realise arbitrary orientations: tool yaw is slaved to the base azimuth and the
      tool roll is fixed. Orientation targets are therefore (yaw, pitch) with roll = 0; a full rotation
      matrix is projected onto that set and the unreachable part is reported.
"""
import math
import xml.etree.ElementTree as ET
import os

import numpy as np

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4"]

# Tool frame used by fk() / ik_pose() / the ROS interface (ROS convention): x = approach direction of the gripper,
# y = left, z = up when the gripper is horizontal, so the identity rotation means "horizontal gripper pointing
# along world +x" and a POSITIVE rotation about y pitches the gripper DOWN. Columns are the tool axes expressed
# in the revolute_4 body frame of the MJCF (x forward, y up, z = jaw-opening axis).
M_TOOL = np.array([[1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0],
                   [0.0, -1.0, 0.0]])


# --------------------------------------------------------------------------- SE(3) helpers
def quat_to_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n          # MuJoCo normalises MJCF quats too
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    """-> (w, x, y, z)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return tuple(q / np.linalg.norm(q))


def axis_angle_mat(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def rot_log(R):
    """Rotation matrix -> rotation vector (axis * angle)."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    if math.pi - ang < 1e-5:  # near pi: use the symmetric part
        A = (R + np.eye(3)) / 2.0
        k = np.sqrt(np.maximum(np.diag(A), 0))
        i = int(np.argmax(k))
        v = A[:, i] / max(k[i], 1e-12)
        v = v / np.linalg.norm(v)
        return v * ang
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    return w * ang


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------- model
class _Link:
    __slots__ = ("name", "pos", "R", "joint", "axis", "lo", "hi")

    def __init__(self, name, pos, R, joint=None, axis=None, lo=None, hi=None):
        self.name, self.pos, self.R = name, pos, R
        self.joint, self.axis, self.lo, self.hi = joint, axis, lo, hi


class ArmKinematics:
    """FK / Jacobian / joint limits / IK for the 4-DOF arm, parsed from the MJCF."""

    def __init__(self, mjcf_path, tool_site="grasp_center", base_body="base_link",
                 base_world_pos=None, safety_margin=0.03):
        self.mjcf_path = mjcf_path
        root = self._load_xml(mjcf_path)
        world = root.find("worldbody")
        base = None
        for b in world.iter("body"):
            if b.get("name") == base_body:
                base = b
                break
        if base is None:
            raise ValueError("body '%s' not found in %s" % (base_body, mjcf_path))
        angle_rad = (root.find("compiler") is None) or root.find("compiler").get("angle", "degree") == "radian"
        self.links = []
        self.sites = {}
        self.base_pos = self._vec(base.get("pos", "0 0 0"))
        self.base_R = self._quat(base.get("quat"))
        if base_world_pos is not None:
            self.base_pos = np.asarray(base_world_pos, float)
        node, depth_pos, depth_R = base, np.zeros(3), np.eye(3)
        chain_idx = -1
        # walk the (linear) kinematic chain of bodies that carry the arm joints
        body = base
        while body is not None:
            joint = None
            for j in body.findall("joint"):
                if j.get("type", "hinge") == "hinge" and j.get("name") in ARM_JOINTS:
                    joint = j
            if body is base:
                pos, R = np.zeros(3), np.eye(3)
            else:
                pos, R = self._vec(body.get("pos", "0 0 0")), self._quat(body.get("quat"))
            if joint is not None:
                rng = joint.get("range")
                lo, hi = (float(v) for v in rng.split()) if rng else (-math.pi, math.pi)
                if not angle_rad:
                    lo, hi = math.radians(lo), math.radians(hi)
                axis = self._vec(joint.get("axis", "0 0 1"))
                axis = axis / np.linalg.norm(axis)
                self.links.append(_Link(body.get("name"), pos, R, joint.get("name"), axis, lo, hi))
            else:
                self.links.append(_Link(body.get("name"), pos, R))
            for s in body.findall("site"):
                self.sites[s.get("name")] = (body.get("name"), self._vec(s.get("pos", "0 0 0")))
            nxt = None
            for child in body.findall("body"):
                if self._subtree_has_arm_joint(child):
                    nxt = child
                    break
            if nxt is None:
                # last arm body: also register sites/bodies hanging off it (tool, camera, sensor)
                self._register_tool_children(body)
            body = nxt
        self.joint_names = [l.joint for l in self.links if l.joint]
        if self.joint_names != ARM_JOINTS:
            raise ValueError("unexpected arm joints %s" % self.joint_names)
        self.n = len(self.joint_names)
        self.q_lo = np.array([l.lo for l in self.links if l.joint])
        self.q_hi = np.array([l.hi for l in self.links if l.joint])
        self.safety_margin = float(safety_margin)
        if tool_site not in self.sites:
            raise ValueError("site '%s' not found" % tool_site)
        self.tool_site = tool_site
        self.tool_body, self.tool_offset = self.sites[tool_site]
        self._derive_planar_geometry()
        self._parse_masses_and_actuators(root)

    # ----- xml helpers
    @staticmethod
    def _vec(s):
        return np.array([float(v) for v in s.split()])

    @staticmethod
    def _quat(s):
        if s is None:
            return np.eye(3)
        w, x, y, z = (float(v) for v in s.split())
        return quat_to_mat(w, x, y, z)

    @staticmethod
    def _load_xml(path):
        root = ET.parse(path).getroot()
        inc = root.find("include")
        if inc is not None and root.find("worldbody") is not None:
            # scene.xml: the robot lives in the included file
            inc_root = ET.parse(os.path.join(os.path.dirname(path), inc.get("file"))).getroot()
            return inc_root
        if root.find("worldbody") is None and inc is not None:
            return ET.parse(os.path.join(os.path.dirname(path), inc.get("file"))).getroot()
        return root

    def _subtree_has_arm_joint(self, body):
        return any(j.get("name") in ARM_JOINTS for j in body.iter("joint"))

    def _register_tool_children(self, body):
        # bodies mounted rigidly on the last link (esp32_cam, hcsr04, ...): keep their pose for FK of the sensors
        self.mounts = {}
        for child in body.findall("body"):
            if child.find("joint") is not None:
                continue
            pos = self._vec(child.get("pos", "0 0 0"))
            R = self._quat(child.get("quat"))
            self.mounts[child.get("name")] = (pos, R)
            for s in child.findall("site"):
                self.sites[s.get("name")] = (child.get("name"), self._vec(s.get("pos", "0 0 0")))
            for c in child.findall("camera"):
                p = self._vec(c.get("pos", "0 0 0"))
                Rc = self._quat(c.get("quat"))
                self.mounts[c.get("name")] = (pos + R @ p, R @ Rc)

    def _derive_planar_geometry(self):
        self._plane_model()

    # ----- gravity model (for feed-forward compensation of the position-controlled servos)
    def _parse_masses_and_actuators(self, root):
        """Point masses (arm link index, position in that link's frame, mass) from <inertial>/<geom mass>
        of every body in the arm subtree. Hinge joints that are not arm joints (fingers) are taken at 0."""
        self._mass_pts = []
        arm_bodies = {l.name: i for i, l in enumerate([l for l in self.links if l.joint])}
        # base_link (no arm joint) is static: its mass does not load any joint -> skipped.

        def walk(body, idx, T_pos, T_R):
            # T_pos / T_R: transform of this body's frame in the frame of arm link `idx`
            ine = body.find("inertial")
            if ine is not None and ine.get("mass") is not None:
                self._mass_pts.append((idx, T_pos + T_R @ self._vec(ine.get("pos", "0 0 0")), float(ine.get("mass"))))
            else:
                for g in body.findall("geom"):
                    if g.get("mass") is not None:
                        gp = self._vec(g.get("pos", "0 0 0")) if g.get("pos") else np.zeros(3)
                        self._mass_pts.append((idx, T_pos + T_R @ gp, float(g.get("mass"))))
            for ch in body.findall("body"):
                name = ch.get("name")
                if name in arm_bodies:
                    walk(ch, arm_bodies[name], np.zeros(3), np.eye(3))   # new arm link: its own frame is the reference
                else:
                    cp = self._vec(ch.get("pos", "0 0 0"))
                    cR = self._quat(ch.get("quat"))
                    walk(ch, idx, T_pos + T_R @ cp, T_R @ cR)

        first = [b for b in root.find("worldbody").iter("body") if b.get("name") == self.links[1].name][0]
        walk(first, 0, np.zeros(3), np.eye(3))
        # actuators: kp and force range per arm joint
        self.kp = np.full(self.n, np.nan)
        self.force_limit = np.full(self.n, np.nan)
        for a in root.iter("position"):
            if a.get("joint") in self.joint_names:
                i = self.joint_names.index(a.get("joint"))
                self.kp[i] = float(a.get("kp", "nan"))
                fr = a.get("forcerange")
                if fr:
                    self.force_limit[i] = max(abs(float(v)) for v in fr.split())

    def total_moving_mass(self):
        return float(sum(m for _, _, m in self._mass_pts))

    def gravity_torque(self, q, g=9.81, payload_kg=0.0):
        """Torque that gravity exerts about each arm joint axis [N m] (= -dU/dq).
        payload_kg: extra mass held at the tool site (e.g. a grasped dumbbell, 0.03 kg)."""
        q = np.asarray(q, float)[:self.n]
        chain = self._chain(q)
        tau = np.zeros(self.n)
        for idx, local, m in self._mass_pts:
            origin, R_after, _ax = chain[idx]
            c = origin + R_after @ local
            F = np.array([0.0, 0.0, -m * g])
            for j in range(idx + 1):
                oj, _Rj, axj = chain[j]
                tau[j] += axj @ np.cross(c - oj, F)
        if payload_kg:
            p_tool, _ = self.fk(q)
            F = np.array([0.0, 0.0, -payload_kg * g])
            for j in range(self.n):
                oj, _Rj, axj = chain[j]
                tau[j] += axj @ np.cross(p_tool - oj, F)
        return tau

    def gravity_compensation(self, q, payload_kg=0.0):
        """Offset to ADD to a desired joint angle so that the position servo (torque = kp (ctrl - q))
        holds q against gravity: ctrl = q - tau_grav / kp."""
        if np.any(np.isnan(self.kp)):
            return np.zeros(self.n)
        return -self.gravity_torque(q, payload_kg=payload_kg) / self.kp

    # ----- forward kinematics
    def _chain(self, q):
        """Per arm joint: (origin, R_after_joint, world_axis) of the body that carries it."""
        p = self.base_pos.copy()
        R = self.base_R.copy()
        out = []
        qi = 0
        for l in self.links:
            if l.name != self.links[0].name:
                p = p + R @ l.pos
                R = R @ l.R
            if l.joint:
                axis_w = R @ l.axis
                R = R @ axis_angle_mat(l.axis, q[qi])
                out.append((p.copy(), R.copy(), axis_w))
                qi += 1
        self._last_R = R
        self._last_p = p
        return out

    def fk_frame(self, q, frame="tool"):
        """-> (position(3), R(3x3)) in world.

        frame:  'tool'  grasp_center position, orientation in the ROS tool convention (see M_TOOL)
                site names ('grasp_center', 'esp32_cam_lens', 'hcsr04_sensor') and rigid mounts of revolute_4
                ('esp32_cam', 'hcsr04', 'esp32cam' = camera): orientation = the MJCF body / camera axes
                (identical to what MuJoCo reports, used to cross-check against the simulator).
        """
        q = np.asarray(q, float)[:self.n]
        self._chain(q)
        p4, R4 = self._last_p, self._last_R
        if frame == "tool":
            return p4 + R4 @ self.tool_offset, R4 @ M_TOOL
        if frame in self.sites:
            body, off = self.sites[frame]
            if body == self.links[-1].name:
                return p4 + R4 @ off, R4.copy()
            mp, mR = self.mounts[body]
            return p4 + R4 @ (mp + mR @ off), R4 @ mR
        if frame in self.mounts:
            mp, mR = self.mounts[frame]
            return p4 + R4 @ mp, R4 @ mR
        if frame == self.links[-1].name:
            return p4.copy(), R4.copy()
        raise KeyError(frame)

    def fk(self, q):
        """Tool (grasp_center) position and orientation (ROS tool convention, see M_TOOL) in world."""
        return self.fk_frame(q, "tool")

    def tool_pitch_yaw(self, q):
        """(yaw, pitch) of the tool +x axis. pitch = elevation above horizontal."""
        _, R = self.fk(q)
        x = R[:, 0]
        return math.atan2(x[1], x[0]), math.atan2(x[2], math.hypot(x[0], x[1]))

    # ----- Jacobian
    def jacobian(self, q):
        """Geometric Jacobian of the tool site: 6x4, rows = [v; w] in world frame."""
        q = np.asarray(q, float)[:self.n]
        chain = self._chain(q)
        p_t, _ = self.fk(q)
        J = np.zeros((6, self.n))
        for i, (origin, _R, axis_w) in enumerate(chain):
            # axis/origin are of the joint frame BEFORE its own rotation (axis is unchanged by it)
            J[:3, i] = np.cross(axis_w, p_t - origin)
            J[3:, i] = axis_w
        return J

    def jacobian_numeric(self, q, eps=1e-6):
        q = np.asarray(q, float)
        p0, R0 = self.fk(q)
        J = np.zeros((6, self.n))
        for i in range(self.n):
            dq = q.copy()
            dq[i] += eps
            p1, R1 = self.fk(dq)
            J[:3, i] = (p1 - p0) / eps
            J[3:, i] = rot_log(R1 @ R0.T) / eps
        return J

    def manipulability(self, q, position_only=True):
        J = self.jacobian(q)
        Jp = J[:3] if position_only else J
        return math.sqrt(max(np.linalg.det(Jp @ Jp.T), 0.0))

    # ----- limits
    def within_limits(self, q, margin=0.0):
        q = np.asarray(q, float)
        return bool(np.all(q >= self.q_lo + margin - 1e-9) and np.all(q <= self.q_hi - margin + 1e-9))

    def check_limits(self, q, margin=0.0):
        """-> list of human readable violations (empty = ok)."""
        q = np.asarray(q, float)
        bad = []
        for i, n in enumerate(self.joint_names):
            if q[i] < self.q_lo[i] + margin - 1e-9:
                bad.append("%s = %.3f < lower limit %.3f" % (n, q[i], self.q_lo[i] + margin))
            if q[i] > self.q_hi[i] - margin + 1e-9:
                bad.append("%s = %.3f > upper limit %.3f" % (n, q[i], self.q_hi[i] - margin))
        return bad

    def clamp(self, q, margin=0.0):
        return np.minimum(np.maximum(np.asarray(q, float), self.q_lo + margin), self.q_hi - margin)

    # ----- IK
    # Closed form for yaw + planar 3R, extracted from the MJCF chain at q = 0 (see _derive_planar_geometry).
    def _plane_model(self):
        """Constants of the planar model, all derived from the MJCF chain:
        S      shoulder (joint_2 origin) in the yaw frame: (radial, height)
        a,b,c  in-plane vectors joint2->joint3, joint3->joint4, joint4->tool (link frames at q = 0)
        lat    lateral (out-of-plane) coordinate of the tool in the yaw frame (the 3 mm grasp_center offset)
        sgn    +1/-1 : rotation of joint i by q_i is CCW (+) or CW (-) in the (radial, up) plane
        alpha0 angle of the tool +x axis in the (radial, up) plane at q = 0"""
        if hasattr(self, "_pm"):
            return self._pm
        q0 = np.zeros(self.n)
        ch = self._chain(q0)
        o = [c[0] - self.base_pos for c in ch]      # joint origins in world, base at origin (yaw 0 => world = yaw frame)
        tool, Rt = self.fk(q0)
        tool = tool - self.base_pos
        ax = [c[2] for c in ch]
        n_plane = np.array([0.0, -1.0, 0.0])         # CCW in (x, z) is rotation about -y
        sgn = [float(np.sign(ax[i] @ n_plane)) for i in (1, 2, 3)]
        for i in (1, 2, 3):
            if abs(abs(ax[i] @ n_plane) - 1) > 1e-6:
                raise ValueError("joints 2-4 are not parallel to the world y axis at q=0: planar IK invalid")
        if abs(o[0][0]) > 1e-6 or abs(o[0][1]) > 1e-6:
            raise ValueError("joint_1 axis is not through the base origin")
        def pl(v):
            return np.array([v[0], v[2]])
        S = pl(o[1])
        a = pl(o[2] - o[1])
        b = pl(o[3] - o[2])
        c = pl(tool - o[3])
        lat = float(tool[1])                          # y offset of the tool (constant, joints 2-4 don't move it)
        for i in (1, 2, 3):                           # all planar joints must sit in the plane y = const(=0)
            if abs(o[i][1]) > 1e-5:   # MJCF quats are not exactly unit; MuJoCo normalises them -> ~1e-7 m skew
                raise ValueError("joint origins are not in one vertical plane")
        alpha0 = math.atan2(Rt[2, 0], Rt[0, 0])
        self._pm = dict(S=S, a=a, b=b, c=c, lat=lat, sgn=sgn, alpha0=alpha0)
        return self._pm

    @staticmethod
    def _rot2(t):
        c, s = math.cos(t), math.sin(t)
        return np.array([[c, -s], [s, c]])

    def ik_analytic(self, p_des, pitch_des, q_seed=None, margin=None, return_all=False):
        """Closed-form IK for tool position + tool pitch. Returns (q, info) like ik_position_pitch.
        Up to two (elbow) branches are computed; the ones inside the joint limits (minus margin) are kept
        and the one closest to q_seed (default: zero) is returned."""
        margin = self.safety_margin if margin is None else margin
        pm = self._plane_model()
        p = np.asarray(p_des, float) - self.base_pos
        rho = math.hypot(p[0], p[1])
        fail = lambda why: (np.full(self.n, np.nan), dict(success=False, pos_err=float("inf"), pitch_err=float("inf"),
                                                           reason=why, solutions=[]))
        if abs(pm["lat"]) > rho:
            return fail("target too close to the base axis")
        # yaw: tool = Rz(q1) * (r, lat), r > 0
        q1 = math.atan2(p[1], p[0]) - math.asin(pm["lat"] / rho)
        r = math.sqrt(rho * rho - pm["lat"] ** 2)
        T = np.array([r, p[2]])
        Phi = pitch_des - pm["alpha0"]                # total rotation of link 4 in the plane
        W = T - self._rot2(Phi) @ pm["c"]             # wrist (joint_4) position
        D = W - pm["S"]
        a, b = pm["a"], pm["b"]
        la, lb, dd = np.linalg.norm(a), np.linalg.norm(b), np.linalg.norm(D)
        if dd > la + lb + 1e-9 or dd < abs(la - lb) - 1e-9:
            return fail("out of reach (wrist distance %.3f m, reachable %.3f..%.3f)" % (dd, abs(la - lb), la + lb))
        cb = (dd * dd - la * la - lb * lb) / (2 * la * lb)
        cb = max(-1.0, min(1.0, cb))
        beta = math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])
        sols = []
        for sgn_elbow in (1.0, -1.0):
            th3 = sgn_elbow * math.acos(cb) - beta
            v = a + self._rot2(th3) @ b
            th2 = math.atan2(D[1], D[0]) - math.atan2(v[1], v[0])
            th4 = Phi - th2 - th3
            q = np.array([q1, wrap(th2) * pm["sgn"][0], wrap(th3) * pm["sgn"][1], wrap(th4) * pm["sgn"][2]])
            q[1:] = [wrap(x) for x in q[1:]]
            sols.append(q)
        ok, bad = [], []
        for q in sols:
            (ok if self.within_limits(q, margin) else bad).append(q)
        if not ok:
            viol = "; ".join(self.check_limits(bad[0], margin)) if bad else "none"
            return fail("joint limits: " + viol)
        ref = np.zeros(self.n) if q_seed is None else np.asarray(q_seed, float)
        ok.sort(key=lambda q: float(np.linalg.norm(q - ref)))
        q = ok[0]
        p_fk, _ = self.fk(q)
        pitch_fk = self.tool_pitch_yaw(q)[1]
        info = dict(success=True, pos_err=float(np.linalg.norm(p_fk - np.asarray(p_des))),
                    pitch_err=float(wrap(pitch_des - pitch_fk)), reason="analytic", solutions=ok)
        return q, info

    # --- numeric (damped least squares) solver: polishes a solution / handles arbitrary seeds
    def _pitch_jac(self, q, eps=1e-6):
        q = np.asarray(q, float)
        _, p0 = self.tool_pitch_yaw(q)
        row = np.zeros(self.n)
        for i in range(self.n):
            dq = q.copy()
            dq[i] += eps
            row[i] = wrap(self.tool_pitch_yaw(dq)[1] - p0) / eps
        return row[None, :]

    def ik_numeric(self, p_des, pitch_des, q_seed, margin=None, tol_pos=2e-4, tol_pitch=1e-3, max_iter=100):
        """Damped-least-squares IK started from q_seed, joint limits enforced by clamping with the
        clamped joints frozen for that step. Returns (q, info)."""
        margin = self.safety_margin if margin is None else margin
        p_des = np.asarray(p_des, float)
        q = self.clamp(q_seed, margin)
        lam = 0.02
        info = dict(success=False, pos_err=float("inf"), pitch_err=float("inf"), reason="max_iter")
        for it in range(max_iter):
            p, _ = self.fk(q)
            e = np.concatenate([p_des - p, [wrap(pitch_des - self.tool_pitch_yaw(q)[1])]])
            info.update(pos_err=float(np.linalg.norm(e[:3])), pitch_err=float(e[3]), iters=it)
            if info["pos_err"] < tol_pos and abs(e[3]) < tol_pitch:
                info.update(success=True, reason="converged")
                return q, info
            J4 = np.vstack([self.jacobian(q)[:3], self._pitch_jac(q)])
            free = np.ones(self.n, bool)
            for _ in range(2):
                Jf = J4[:, free]
                step = np.zeros(self.n)
                step[free] = Jf.T @ np.linalg.solve(Jf @ Jf.T + lam ** 2 * np.eye(4), e)
                step = np.clip(step, -0.3, 0.3)
                qn = q + step
                qc = self.clamp(qn, margin)
                hit = np.abs(qc - qn) > 1e-12
                if not hit.any() or not (free & ~hit).any():
                    break
                free &= ~hit
            q = qc
        info["reason"] = "unreachable_or_limits (pos_err %.1f mm, pitch_err %.1f deg)" % (
            info["pos_err"] * 1000, math.degrees(abs(info["pitch_err"])))
        return q, info

    def ik_position_pitch(self, p_des, pitch_des, q_seed=None, margin=None, polish=True, **kw):
        """Tool position + pitch. Analytic solution (elbow branch closest to q_seed) polished by DLS.
        Returns (q, info): info['success'], 'pos_err' [m], 'pitch_err' [rad], 'reason'."""
        q, info = self.ik_analytic(p_des, pitch_des, q_seed=q_seed, margin=margin)
        if info["success"] and polish and (info["pos_err"] > 1e-4 or abs(info["pitch_err"]) > 1e-3):
            q2, i2 = self.ik_numeric(p_des, pitch_des, q, margin=margin)
            if i2["success"]:
                return q2, i2
        return q, info

    def ik_position(self, p_des, q_seed=None, pitch_candidates=None, prefer_pitch=0.0, margin=None):
        """Position-only IK: the one spare DOF is the tool pitch. Sweeps pitches (closest to prefer_pitch
        first) and returns the first reachable solution; info['pitch'] holds the pitch used."""
        if pitch_candidates is None:
            pitch_candidates = np.radians(np.arange(-90, 91, 2.5))
        last = None
        for pc in sorted(pitch_candidates, key=lambda a: abs(a - prefer_pitch)):
            q, info = self.ik_position_pitch(p_des, pc, q_seed=q_seed, margin=margin)
            info["pitch"] = float(pc)
            if info["success"]:
                return q, info
            last = (q, info)
        last[1]["reason"] = "no tool pitch makes the point reachable (" + last[1]["reason"] + ")"
        return last

    def ik_pose(self, p_des, R_des, q_seed=None, margin=None):
        """Position + orientation. Only (yaw, pitch) of the tool x axis can be controlled (roll is fixed by
        the arm, yaw = base azimuth). info['orient_err'] [rad] = rotation error of what is NOT achievable."""
        x = np.asarray(R_des, float)[:, 0]
        pitch = math.atan2(x[2], math.hypot(x[0], x[1]))
        q, info = self.ik_position_pitch(p_des, pitch, q_seed=q_seed, margin=margin)
        if info["success"]:
            _, R = self.fk(q)
            info["orient_err"] = float(np.linalg.norm(rot_log(R_des @ R.T)))
        else:
            info["orient_err"] = float("inf")
        return q, info

    # ----- workspace helper
    def reachable(self, p_des, pitch=None, q_seed=None, margin=None):
        if pitch is None:
            q, info = self.ik_position(p_des, q_seed=q_seed, margin=margin)
        else:
            q, info = self.ik_position_pitch(p_des, pitch, q_seed=q_seed, margin=margin)
        return bool(info["success"]), q, info
