#!/usr/bin/env python3
"""Offline MuJoCo probe (no ROS): put the arm in a given pose, run mj_forward and print
the grasp_center position and ALL contacts involving the robot (with signed distance).
Compare against:  arm_core_cli config/robot_model.txt fk q1 q2 q3 q4   and   ... collide q1 q2 q3 q4 0
usage: python3 mj_probe.py <path/to/mjcf/scene.xml> q1 q2 q3 q4 [finger_active=-0.7]
"""
import sys, mujoco, numpy as np
m = mujoco.MjModel.from_xml_path(sys.argv[1]); d = mujoco.MjData(m)
q = [float(x) for x in sys.argv[2:6]]; fa = float(sys.argv[6]) if len(sys.argv) > 6 else -0.7
for n, v in zip(["joint_1", "joint_2", "joint_3", "joint_4", "finger_active"], q + [fa]):
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "finger_passive")]] = -fa
mujoco.mj_forward(m, d)
gc = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_center")]
print("grasp_center  %.5f %.5f %.5f" % tuple(gc))
for b in ["revolute_1", "revolute_2", "revolute_3", "revolute_4"]:
    print("%-11s xpos %.5f %.5f %.5f" % ((b,) + tuple(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)])))
bn = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
gn = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "geom#%d" % g
print("contacts (dist<0 = penetration, margin=%s):" % set(np.round(m.geom_margin, 4)))
for i in range(d.ncon):
    c = d.contact[i]
    print("  %s/%s <-> %s/%s  dist %+.2f mm" % (bn(c.geom1), gn(c.geom1), bn(c.geom2), gn(c.geom2), c.dist * 1000))
