#!/usr/bin/env python3
"""Phase 3 offline test (no ROS, no MuJoCo needed):   python3 scripts/test_kinematics.py [--mjcf mjcf/scene.xml]

Checks the kinematic model that is parsed from the MJCF: structure, home pose, Jacobian, gravity model, IK
round trip, joint-limit handling, unreachable targets, trajectories, and whether every dumbbell of the scene is
reachable by a horizontal side grasp. Exit code 0 = all passed.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from autonomous_manipulator.kinematics import ArmKinematics, quat_to_mat  # noqa: E402
from autonomous_manipulator.trajectory import cartesian_line, check_trajectory, joint_trajectory  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


def scene_objects(path):
    root = ET.parse(path).getroot()
    out = {}
    for b in root.find("worldbody").findall("body"):
        if b.get("name", "").startswith("db_"):
            out[b.get("name")] = np.array([float(v) for v in b.get("pos").split()])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=os.path.join(HERE, "..", "mjcf", "scene.xml"))
    a = ap.parse_args()
    k = ArmKinematics(a.mjcf)
    rng = np.random.RandomState(0)
    home = np.array([0.0, 0.8901, 0.4538, -0.4363])

    print("1. model parsed from the MJCF")
    check("joints", k.joint_names == ["joint_1", "joint_2", "joint_3", "joint_4"])
    check("limits +-1.571", np.allclose(k.q_lo, -1.571) and np.allclose(k.q_hi, 1.571))
    check("moving mass ~0.45 kg", abs(k.total_moving_mass() - 0.451) < 0.01, "(%.3f kg)" % k.total_moving_mass())
    check("servo kp / force range read", np.allclose(k.kp, 4.65) and np.allclose(k.force_limit, 0.93))

    print("2. forward kinematics")
    p, R = k.fk(home)
    yaw, pitch = k.tool_pitch_yaw(home)
    check("home keyframe: tool horizontal", abs(pitch) < 1e-3 and abs(yaw) < 1e-3, "(pitch %.4f deg)" % math.degrees(pitch))
    check("home keyframe: tool ahead of base, above floor", 0.15 < p[0] < 0.25 and 0.08 < p[2] < 0.16, "(%s)" % np.round(p, 4))
    pa, _ = k.fk(np.array([0.5, 0.0, 0.0, 0.0]))
    pb, _ = k.fk(np.array([0.0, 0.0, 0.0, 0.0]))
    check("joint_1 rotates about world z", abs(pa[2] - pb[2]) < 1e-9 and abs(np.hypot(pa[0], pa[1]) - np.hypot(pb[0], pb[1])) < 3e-3)
    check("joint_1 positive = counter-clockwise (+y)", pa[1] > 0)

    print("3. Jacobian")
    w = max(np.abs(k.jacobian(q) - k.jacobian_numeric(q)).max() for q in rng.uniform(k.q_lo, k.q_hi, (100, 4)))
    check("analytic == numeric", w < 1e-5, "(max diff %.1e)" % w)

    print("4. gravity model")
    def U(q):
        ch = k._chain(q)
        return sum(m * 9.81 * (ch[i][0] + ch[i][1] @ loc)[2] for i, loc, m in k._mass_pts)
    w = 0.0
    for q in rng.uniform(k.q_lo, k.q_hi, (40, 4)):
        g = np.array([-(U(q + e) - U(q - e)) / 2e-6 for e in np.eye(4) * 1e-6])
        w = max(w, np.abs(g - k.gravity_torque(q)).max())
    check("torque == -dU/dq", w < 1e-6, "(max diff %.1e N m)" % w)
    mx = np.zeros(4)
    for q2 in np.linspace(-1.5, 1.5, 21):
        for q3 in np.linspace(-1.5, 1.5, 21):
            for q4 in np.linspace(-1.5, 1.5, 21):
                mx = np.maximum(mx, np.abs(k.gravity_torque(np.array([0, q2, q3, q4]))))
    check("worst-case holding torque below servo force limit", np.all(mx < k.force_limit),
          "(max %s of %.2f N m; margin x%.2f)" % (np.round(mx[1:], 3), k.force_limit[0], k.force_limit[0] / mx.max()))

    print("5. inverse kinematics")
    ok = tot = 0
    worst = 0.0
    for q in rng.uniform(k.q_lo + 0.05, k.q_hi - 0.05, (1500, 4)):
        p, R = k.fk(q)
        if p[:2] @ np.array([math.cos(q[0]), math.sin(q[0])]) < 0.05 or R[:, 0] @ np.array([math.cos(q[0]), math.sin(q[0]), 0]) < 0.0:
            continue  # tool behind the base axis or tilted past vertical toward the base: pitch is ambiguous there, never needed
        tot += 1
        qs, inf = k.ik_position_pitch(p, k.tool_pitch_yaw(q)[1], q_seed=q + 0.05)
        if inf["success"]:
            ok += 1
            worst = max(worst, np.linalg.norm(k.fk(qs)[0] - p))
    check("round trip FK->IK->FK", ok == tot and worst < 5e-4, "(%d/%d solved, worst %.3f mm)" % (ok, tot, worst * 1000))
    q, inf = k.ik_position_pitch([0.28, 0.1, 0.05], math.radians(-20))
    check("IK result inside limits (with margin)", inf["success"] and k.within_limits(q, k.safety_margin))
    q, inf = k.ik_position_pitch([0.7, 0.0, 0.05], 0.0)
    check("unreachable target rejected", not inf["success"], "(%s)" % inf["reason"])
    q, inf = k.ik_position_pitch([0.25, 0.0, 0.03], math.radians(-80))
    check("pose outside joint limits rejected", not inf["success"], "(%s)" % inf["reason"][:60])
    Rd = quat_to_mat(*(math.cos(0.25), 0, math.sin(0.25), 0))   # rotation +0.5 rad about y = gripper pitched DOWN 28.6 deg, no roll
    q, inf = k.ik_pose(np.array([0.27, 0.0, 0.06]), Rd, q_seed=home)
    check("pose IK (position + orientation)", inf["success"] and inf["orient_err"] < 0.02 and abs(k.tool_pitch_yaw(q)[1] + 0.5) < 1e-3,
          "(residual %.4f rad = yaw offset from the 3 mm grasp-centre lateral offset; pitch %.1f deg)" % (inf["orient_err"], math.degrees(k.tool_pitch_yaw(q)[1])))
    pk, Rk = k.fk(home)
    check("tool frame: identity at home (x fwd, y left, z up)", np.allclose(Rk, np.eye(3), atol=2e-3), "(max dev %.1e)" % np.abs(Rk - np.eye(3)).max())

    print("6. trajectories")
    vmax, amax = np.full(4, 1.5), np.full(4, 6.0)
    tr = joint_trajectory(home, np.array([0.8, 0.3, 0.9, -0.2]), vmax, amax)
    c = check_trajectory(k, tr, 0.005, vmax, amax)
    check("joint trajectory respects v/a limits", c["ok"], "(peak v %s, T=%.2f s)" % (np.round(c["peak_v"], 2), tr.duration))
    check("starts/ends at rest", np.allclose(tr.sample(0)[1], 0) and np.allclose(tr.sample(tr.duration)[1], 0, atol=1e-9))
    tgt = np.array([0.28, -0.08, 0.0305])
    p0, _ = k.fk(home)
    tj, inf = cartesian_line(k, home, tgt, 0.0, vmax, amax)
    ts, Q, _ = tj.discretise(0.01)
    d = tgt - p0
    dev = max(np.linalg.norm(p0 + np.clip((k.fk(q)[0] - p0) @ d / (d @ d), 0, 1) * d - k.fk(q)[0]) for q in Q)
    check("Cartesian line stays on the straight line", dev < 5e-4, "(max deviation %.3f mm)" % (dev * 1000))
    _, inf = cartesian_line(k, home, np.array([0.6, 0, 0.05]), 0.0, vmax, amax)
    check("blocked line rejected", not inf["ok"], "(%s)" % inf["reason"][:70])

    print("7. dumbbells of the scene (side grasp, tool horizontal, z = body centre)")
    objs = scene_objects(a.mjcf)
    for n, pos in sorted(objs.items()):
        r = math.hypot(pos[0], pos[1])
        ps = [d for d in range(-90, 91) if k.ik_position_pitch(pos, math.radians(d))[1]["success"]]
        pre = pos - 0.06 * np.array([pos[0], pos[1], 0]) / r          # 6 cm before the object along the radial line
        ok_h = k.ik_position_pitch(pos, 0.0)[1]["success"] and k.ik_position_pitch(pre, 0.0)[1]["success"]
        check("%s r=%.3f m reachable at pitch 0 (+ 6 cm pre-grasp)" % (n, r), ok_h,
              "(pitch envelope %d..%d deg)" % (min(ps), max(ps)) if ps else "")
    print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: %s" % ", ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
