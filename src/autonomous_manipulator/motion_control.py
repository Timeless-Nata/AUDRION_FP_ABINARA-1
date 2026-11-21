"""Phase 3 motion controller core (no ROS imports: the ROS wrapper is scripts/motion_control_node.py).

plan_*  : IK + trajectory generation + validation, never moves anything
execute : streams the trajectory as position targets, adds gravity feed-forward (the simulated servos are
          P(D) position actuators and sag under gravity), a small integral trim, and waits until the
          measured joints have settled on the goal.
"""
import math
import threading

import numpy as np

from .kinematics import ArmKinematics, mat_to_quat, quat_to_mat, rot_log
from .trajectory import cartesian_line, check_trajectory, joint_trajectory

DEFAULTS = dict(
    vmax=1.5,                # rad/s  per joint (MG996R is capable of ~5; stay gentle, the arm carries payload)
    amax=6.0,                # rad/s^2
    tool_speed=0.08,         # m/s for Cartesian lines
    control_rate=100.0,      # Hz
    min_time=0.3,            # s
    safety_margin=0.03,      # rad kept away from the MJCF joint limits
    min_tool_z=0.012,        # m, grasp_center must stay above this
    min_link_z=0.020,        # m, elbow / wrist origin must stay above this
    gravity_comp=True,
    payload_kg=0.0,
    integral_gain=1.5,       # 1/s   integral trim on the commanded position (0 = off)
    integral_limit=0.08,     # rad   anti-windup
    integral_deadband=0.0015,  # rad  no integration below this error (servo friction dead-band)
    settle_tol=0.004,        # rad   all joints within this of the goal ...
    settle_time=0.25,        # s     ... for this long
    settle_timeout=3.0,      # s
    pos_tol=0.004,           # m     final tool position error to report success
    pitch_tol=math.radians(3.0),
)


class MotionController:
    def __init__(self, kin: ArmKinematics, **cfg):
        self.kin = kin
        self.cfg = dict(DEFAULTS)
        self.cfg.update({k: v for k, v in cfg.items() if k in DEFAULTS})
        kin.safety_margin = self.cfg["safety_margin"]
        self.vmax = np.full(kin.n, float(self.cfg["vmax"]))
        self.amax = np.full(kin.n, float(self.cfg["amax"]))
        self._abort = threading.Event()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ planning
    def _path_height_ok(self, traj):
        ts, Q, _ = traj.discretise(0.05)
        for q in Q:
            p, _ = self.kin.fk(q)
            if p[2] < self.cfg["min_tool_z"]:
                return False, "tool would go below z=%.3f m (%.3f)" % (self.cfg["min_tool_z"], p[2])
            chain = self.kin._chain(q)
            for name, idx in (("elbow", 2), ("wrist", 3)):
                if chain[idx][0][2] < self.cfg["min_link_z"]:
                    return False, "%s would go below z=%.3f m" % (name, self.cfg["min_link_z"])
        return True, ""

    def plan_joints(self, q_now, q_goal, duration=None):
        q_goal = np.asarray(q_goal, float)
        bad = self.kin.check_limits(q_goal, self.cfg["safety_margin"])
        if bad:
            return None, dict(ok=False, reason="goal violates joint limits: " + "; ".join(bad))
        traj = joint_trajectory(q_now, q_goal, self.vmax, self.amax, duration, self.cfg["min_time"])
        ok, why = self._path_height_ok(traj)
        if not ok:
            return None, dict(ok=False, reason=why)
        p, _ = self.kin.fk(q_goal)
        pitch = self.kin.tool_pitch_yaw(q_goal)[1]
        return traj, dict(ok=True, reason="ok", q_goal=q_goal, pos_err=0.0, pitch_err=0.0, orient_err=0.0,
                          goal_pos=p, goal_pitch=pitch)

    def plan_pose(self, q_now, position, orientation=None, prefer_pitch=0.0, linear=False, duration=None):
        """position: world xyz. orientation: quaternion (w,x,y,z) or None (position-only, pitch chosen from
        prefer_pitch). Only tool pitch can be commanded (yaw follows the base, roll is fixed)."""
        q_now = np.asarray(q_now, float)
        position = np.asarray(position, float)
        if orientation is not None:
            R = quat_to_mat(*orientation)
            q_goal, inf = self.kin.ik_pose(position, R, q_seed=q_now)
        else:
            q_goal, inf = self.kin.ik_position(position, q_seed=q_now, prefer_pitch=prefer_pitch)
        if not inf["success"]:
            return None, dict(ok=False, reason="IK failed: " + inf["reason"])
        pitch = self.kin.tool_pitch_yaw(q_goal)[1]
        if linear:
            traj, linf = cartesian_line(self.kin, q_now, position, pitch, self.vmax, self.amax,
                                        tool_speed=self.cfg["tool_speed"], duration=duration,
                                        margin=self.cfg["safety_margin"], min_time=self.cfg["min_time"])
            if traj is None:
                return None, dict(ok=False, reason=linf["reason"])
        else:
            traj = joint_trajectory(q_now, q_goal, self.vmax, self.amax, duration, self.cfg["min_time"])
        ok, why = self._path_height_ok(traj)
        if not ok:
            return None, dict(ok=False, reason=why)
        chk = check_trajectory(self.kin, traj, 0.01, self.vmax, self.amax, self.cfg["safety_margin"] - 1e-6)
        if not chk["ok"]:
            return None, dict(ok=False, reason="trajectory check failed: " + "; ".join(chk["issues"]))
        return traj, dict(ok=True, reason="ok", q_goal=q_goal, pos_err=inf["pos_err"], pitch_err=inf["pitch_err"],
                          orient_err=inf.get("orient_err", 0.0), goal_pos=position, goal_pitch=pitch)

    # ------------------------------------------------------------------ execution
    def abort(self):
        self._abort.set()

    def feedforward(self, q_ref):
        if not self.cfg["gravity_comp"]:
            return np.zeros(self.kin.n)
        return self.kin.gravity_compensation(q_ref, payload_kg=self.cfg["payload_kg"])

    def execute(self, traj, get_q, send, now, sleep):
        """Run `traj`. get_q() -> measured arm joints (4,), send(q_cmd), now() -> s, sleep(s).
        Returns dict(ok, reason, q_final, joint_err, duration)."""
        if not self._lock.acquire(blocking=False):
            return dict(ok=False, reason="another motion is running")
        self._abort.clear()
        try:
            dt = 1.0 / self.cfg["control_rate"]
            trim = np.zeros(self.kin.n)
            t0 = now()
            last = t0
            q_goal = traj.q_end
            settled_since = None
            while True:
                t = now() - t0
                d = max(now() - last, 1e-4)
                last = now()
                if self._abort.is_set():
                    send(self.kin.clamp(get_q(), self.cfg["safety_margin"]) + trim * 0)
                    return dict(ok=False, reason="aborted", q_final=get_q(), joint_err=np.nan, duration=t)
                q_ref, _qd = traj.sample(min(t, traj.duration))
                q_meas = np.asarray(get_q(), float)
                err = q_ref - q_meas
                if self.cfg["integral_gain"] > 0:
                    act = np.abs(err) > self.cfg["integral_deadband"]
                    trim = np.clip(trim + self.cfg["integral_gain"] * err * d * act, -self.cfg["integral_limit"],
                                   self.cfg["integral_limit"])
                cmd = self.kin.clamp(q_ref + self.feedforward(q_ref) + trim, 0.0)
                send(cmd)
                if t >= traj.duration:
                    if np.max(np.abs(q_goal - q_meas)) < self.cfg["settle_tol"]:
                        settled_since = settled_since if settled_since is not None else t
                        if t - settled_since >= self.cfg["settle_time"]:
                            break
                    else:
                        settled_since = None
                    if t - traj.duration > self.cfg["settle_timeout"]:
                        return dict(ok=False, reason="did not settle (joint error %.4f rad)" %
                                    np.max(np.abs(q_goal - q_meas)), q_final=q_meas,
                                    joint_err=float(np.max(np.abs(q_goal - q_meas))), duration=t)
                sleep(dt)
            q_f = np.asarray(get_q(), float)
            return dict(ok=True, reason="ok", q_final=q_f, joint_err=float(np.max(np.abs(q_goal - q_f))), duration=t)
        finally:
            self._lock.release()

    # ------------------------------------------------------------------ verification
    def pose_error(self, q_meas, goal_pos, goal_pitch):
        p, _ = self.kin.fk(q_meas)
        pitch = self.kin.tool_pitch_yaw(q_meas)[1]
        return float(np.linalg.norm(p - np.asarray(goal_pos))), float(
            (goal_pitch - pitch + math.pi) % (2 * math.pi) - math.pi)
