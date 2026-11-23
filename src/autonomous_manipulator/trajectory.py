"""Phase 3 trajectory generation (pure numpy).

* quintic (minimum-jerk) time scaling: zero velocity and acceleration at both ends
* joint-space trajectories with per-joint velocity / acceleration limits (duration chosen automatically)
* Cartesian straight-line trajectories: the tool moves on a line (position linear in s, pitch linear in s) and
  IK is solved at every sample, always selecting the elbow branch closest to the previous sample, so the joint
  path is continuous. The plan is rejected (with the failing fraction of the path) if any sample is unreachable,
  violates a joint limit, or needs a joint jump above max_joint_step.
"""
import math

import numpy as np

QUINTIC_VMAX = 15.0 / 8.0 / 1.0      # peak of s'(t) * T  for s(t)=10u^3-15u^4+6u^5 -> 1.875
QUINTIC_AMAX = 10.0 / math.sqrt(3.0)  # peak of s''(t) * T^2 -> 5.7735


def quintic(u):
    """Minimum-jerk blend s(u), ds/du, d2s/du2 for u in [0, 1]."""
    u = np.clip(u, 0.0, 1.0)
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    ds = 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4
    dds = 60 * u - 180 * u ** 2 + 120 * u ** 3
    return s, ds, dds


class Trajectory:
    """Sampled-on-demand trajectory q(t). `fn(t)` -> (q, qd)."""

    def __init__(self, duration, fn, q_start, q_end, info=None):
        self.duration = float(duration)
        self._fn = fn
        self.q_start = np.asarray(q_start, float)
        self.q_end = np.asarray(q_end, float)
        self.info = info or {}

    def sample(self, t):
        t = min(max(t, 0.0), self.duration)
        return self._fn(t)

    def discretise(self, dt):
        n = max(2, int(math.ceil(self.duration / dt)) + 1)
        ts = np.linspace(0.0, self.duration, n)
        Q = np.zeros((n, len(self.q_start)))
        QD = np.zeros_like(Q)
        for i, t in enumerate(ts):
            Q[i], QD[i] = self.sample(t)
        return ts, Q, QD


def min_duration(delta, vmax, amax):
    """Shortest quintic duration that respects per-joint |v| <= vmax and |a| <= amax."""
    delta = np.abs(np.asarray(delta, float))
    vmax = np.broadcast_to(np.asarray(vmax, float), delta.shape)
    amax = np.broadcast_to(np.asarray(amax, float), delta.shape)
    t_v = QUINTIC_VMAX * delta / vmax
    t_a = np.sqrt(QUINTIC_AMAX * delta / amax)
    return float(max(np.max(t_v), np.max(t_a), 0.0))


def joint_trajectory(q0, q1, vmax, amax, duration=None, min_time=0.2):
    """Point-to-point joint-space move. duration=None -> fastest allowed by the limits (at least min_time).
    A user-given duration that would exceed the limits is raised to the minimum feasible one."""
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = q1 - q0
    t_min = min_duration(d, vmax, amax)
    T = max(t_min, min_time) if duration is None or duration <= 0 else max(float(duration), t_min)

    def fn(t):
        s, ds, _ = quintic(t / T)
        return q0 + d * s, d * ds / T

    return Trajectory(T, fn, q0, q1, info=dict(kind="joint", t_min=t_min))


def cartesian_line(kin, q0, p1, pitch1, vmax, amax, tool_speed=0.08, duration=None, step=0.005,
                   max_joint_step=0.35, margin=None, min_time=0.3):
    """Straight line in tool space from the pose at q0 to (p1, pitch1).

    Returns (Trajectory | None, info). info: ok (bool), reason, fail_fraction, n_samples, path_len.
    The joint path is precomputed at `step` metres resolution and interpolated linearly in between
    (the step is small, so the tool stays on the line to well below a millimetre).
    """
    q0 = np.asarray(q0, float)
    p0, _ = kin.fk(q0)
    pitch0 = kin.tool_pitch_yaw(q0)[1]
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    dpitch = ((pitch1 - pitch0) + math.pi) % (2 * math.pi) - math.pi
    # sample count: by length and by pitch change (1 deg ~ 1 cm of tool motion)
    n = max(2, int(math.ceil(max(L / step, abs(dpitch) / math.radians(2.0)))) + 1)
    qs = np.zeros((n, kin.n))
    qs[0] = q0
    prev = q0
    for i in range(1, n):
        u = i / (n - 1)
        pi = p0 + (p1 - p0) * u
        pitchi = pitch0 + dpitch * u
        q, inf = kin.ik_position_pitch(pi, pitchi, q_seed=prev, margin=margin)
        if not inf["success"]:
            return None, dict(ok=False, reason="line blocked at %.0f%% of the path: %s" % (100 * u, inf["reason"]),
                              fail_fraction=u, n_samples=n, path_len=L)
        jump = float(np.max(np.abs(q - prev)))
        if jump > max_joint_step:
            return None, dict(ok=False, reason="line needs a %.2f rad joint jump at %.0f%% (elbow/limit flip)" % (jump, 100 * u),
                              fail_fraction=u, n_samples=n, path_len=L)
        qs[i] = q
        prev = q
    # time: by tool speed, then stretched to honour joint velocity/acceleration limits
    T_tool = QUINTIC_VMAX * L / max(tool_speed, 1e-6) if L > 1e-6 else 0.0
    # worst per-joint total excursion along the path
    seg = np.abs(np.diff(qs, axis=0))
    exc = seg.sum(axis=0)
    t_lim = min_duration(exc, vmax, amax)
    T = max(T_tool, t_lim, min_time) if duration is None or duration <= 0 else max(float(duration), t_lim)
    u_grid = np.linspace(0.0, 1.0, n)

    def fn(t):
        s, ds, _ = quintic(t / T)
        q = np.array([np.interp(s, u_grid, qs[:, j]) for j in range(kin.n)])
        # velocity: dq/ds from the neighbouring samples
        h = 1.0 / (n - 1)
        s2 = min(1.0, s + h * 0.5)
        s1 = max(0.0, s - h * 0.5)
        qa = np.array([np.interp(s2, u_grid, qs[:, j]) for j in range(kin.n)])
        qb = np.array([np.interp(s1, u_grid, qs[:, j]) for j in range(kin.n)])
        dq_ds = (qa - qb) / max(s2 - s1, 1e-9)
        return q, dq_ds * ds / T

    return Trajectory(T, fn, qs[0], qs[-1], info=dict(kind="line", path_len=L)), dict(
        ok=True, reason="ok", fail_fraction=1.0, n_samples=n, path_len=L)


def check_trajectory(kin, traj, dt, vmax, amax, margin=0.0):
    """Offline validation of a plan: limits, velocity, acceleration. -> dict(ok, issues, peak_v, peak_a)."""
    ts, Q, QD = traj.discretise(dt)
    issues = []
    for k_, t in enumerate(ts):
        v = kin.check_limits(Q[k_], margin)
        if v:
            issues.append("t=%.2f %s" % (t, v[0]))
            break
    peak_v = np.max(np.abs(QD), axis=0)
    acc = np.diff(QD, axis=0) / np.diff(ts)[:, None]
    peak_a = np.max(np.abs(acc), axis=0) if len(acc) else np.zeros(kin.n)
    vmax = np.broadcast_to(np.asarray(vmax, float), peak_v.shape)
    amax = np.broadcast_to(np.asarray(amax, float), peak_a.shape)
    if np.any(peak_v > vmax * 1.05):
        issues.append("velocity limit exceeded: %s" % np.round(peak_v, 2))
    if np.any(peak_a > amax * 1.25):
        issues.append("acceleration limit exceeded: %s" % np.round(peak_a, 2))
    return dict(ok=not issues, issues=issues, peak_v=peak_v, peak_a=peak_a)
