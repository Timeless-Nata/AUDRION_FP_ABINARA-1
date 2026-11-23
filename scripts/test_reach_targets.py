#!/usr/bin/env python3
"""Phase 3 simulation test. Run with simulation.launch running (sim + motion_control):
        rosrun autonomous_manipulator test_reach_targets.py

A. FK vs MuJoCo ground truth: the arm is moved to several joint configurations and the FK of the camera and
   of the HC-SR04 site is compared with /camera/pose and /hcsr04/pose, which MuJoCo itself publishes.
B. Target reaching: each dumbbell position (parsed from scene.xml) is approached horizontally: pre-grasp pose
   6 cm in front (joint move) and then a straight-line move onto the object centre. The tool error is measured
   from the measured joints.
C. Orientation target (pitch), dry-run, and rejection of unreachable / below-floor / limit-violating targets.
Contacts that the simulation reports during the motions are listed; contact with a dumbbell is a warning,
anything else (floor, plate, self) is a failure.
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from autonomous_manipulator.kinematics import ARM_JOINTS, ArmKinematics
from autonomous_manipulator.srv import MoveJoints, MoveToPose, MoveToPoseRequest

FAILS, WARNS = [], []


def check(name, ok, detail=""):
    print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pos-tol", type=float, default=0.004, help="accepted final tool error [m]")
    ap.add_argument("--fk-tol", type=float, default=0.001, help="accepted FK vs MuJoCo position difference [m]")
    ap.add_argument("--no-contacts", action="store_true")
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_reach_targets")
    mjcf = rospy.get_param("/motion_control/mjcf_path", None)
    if not mjcf:
        print("FAIL: /motion_control/mjcf_path not set - is motion_control running?")
        return 1
    kin = ArmKinematics(mjcf)
    state = {}
    poses = {}
    rospy.Subscriber("/arm/joint_states", JointState, lambda m: state.update(zip(m.name, m.position)))
    rospy.Subscriber("/camera/pose", PoseStamped, lambda m: poses.__setitem__("cam", m))
    rospy.Subscriber("/hcsr04/pose", PoseStamped, lambda m: poses.__setitem__("hc", m))
    for s in ("/motion/move_to_pose", "/motion/move_joints", "/motion/go_home"):
        rospy.wait_for_service(s, timeout=15)
    move_pose = rospy.ServiceProxy("/motion/move_to_pose", MoveToPose)
    move_joints = rospy.ServiceProxy("/motion/move_joints", MoveJoints)
    go_home = rospy.ServiceProxy("/motion/go_home", Trigger)
    report = None
    if not args.no_contacts:
        try:
            rospy.wait_for_service("/sim/contact_report", timeout=5)
            report = rospy.ServiceProxy("/sim/contact_report", Trigger)
            report()  # clear
        except rospy.ROSException:
            print("note: /sim/contact_report not available, contacts not checked")

    def q_now():
        return np.array([state[j] for j in ARM_JOINTS])

    def pose_req(p, pitch=0.0, linear=False, dry=False, mode=MoveToPoseRequest.MODE_POSITION, quat=None):
        r = MoveToPoseRequest()
        r.target.header.frame_id = "world"
        r.target.pose.position.x, r.target.pose.position.y, r.target.pose.position.z = [float(v) for v in p]
        if quat is not None:
            r.target.pose.orientation.w, r.target.pose.orientation.x, r.target.pose.orientation.y, r.target.pose.orientation.z = quat
        r.mode, r.prefer_pitch, r.linear, r.dry_run = mode, pitch, linear, dry
        return r

    def contacts(label):
        if report is None:
            return
        for c in json.loads(report().message)["contacts"]:
            if c["min_dist"] > -0.0005:
                continue
            other = (c["body1"] + " " + c["body2"])
            tag = "warn" if "db_" in other else "FAIL"
            print("    contact [%s] %s/%s <-> %s/%s depth %.1f mm  (%s)" % (
                tag, c["body1"], c["geom1"], c["body2"], c["geom2"], -c["min_dist"] * 1000, label))
            if tag == "FAIL":
                FAILS.append("contact during " + label)

    t0 = rospy.get_time()
    while len(state) < 5 and rospy.get_time() - t0 < 10:
        rospy.sleep(0.1)
    if len(state) < 5:
        print("FAIL: no joint states")
        return 1
    print("home")
    check("go_home", go_home().success)
    contacts("home")

    print("A. FK vs MuJoCo (camera pose and HC-SR04 site pose published by the simulation)")
    for q in ([0.0, 0.89, 0.45, -0.44], [0.5, 0.5, 0.8, -0.6], [-0.6, 0.3, 1.0, -0.9], [0.3, -0.2, 0.6, 0.1]):
        r = move_joints(list(ARM_JOINTS), q, 0.0, False)
        if not r.success:
            check("move_joints %s" % q, False, r.message)
            continue
        rospy.sleep(0.6)
        qm = q_now()
        for key, frame, optical in (("cam", "esp32cam", True), ("hc", "hcsr04_sensor", False)):
            p_fk, R_fk = kin.fk_frame(qm, frame)
            if optical:
                R_fk = R_fk @ np.diag([1.0, -1.0, -1.0])
            m = poses[key].pose
            p_sim = np.array([m.position.x, m.position.y, m.position.z])
            from autonomous_manipulator.kinematics import quat_to_mat, rot_log
            R_sim = quat_to_mat(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
            dp = np.linalg.norm(p_fk - p_sim)
            da = np.linalg.norm(rot_log(R_sim.T @ R_fk))
            check("%s pose at q=%s" % (frame, np.round(q, 2)), dp < args.fk_tol and da < math.radians(0.3),
                  "(position diff %.3f mm, angle diff %.3f deg)" % (dp * 1000, math.degrees(da)))
    contacts("FK poses")

    print("B. reaching the dumbbells (horizontal side approach)")
    root = ET.parse(os.path.join(os.path.dirname(mjcf), "scene.xml")).getroot()
    objs = {b.get("name"): np.array([float(v) for v in b.get("pos").split()])
            for b in root.find("worldbody").findall("body") if b.get("name", "").startswith("db_")}
    for n, pos in sorted(objs.items()):
        radial = np.array([pos[0], pos[1], 0.0]) / math.hypot(pos[0], pos[1])
        pre = pos - 0.06 * radial
        pre[2] = pos[2] + 0.02
        check("go_home before %s" % n, go_home().success)
        r = move_pose(pose_req(pre, linear=False))
        check("%s pre-grasp" % n, r.success and r.pos_error < args.pos_tol,
              "(%s, err %.2f mm, %.2f s)" % (r.message, r.pos_error * 1000, r.duration))
        r = move_pose(pose_req(pos, linear=True))
        check("%s centre, straight line" % n, r.success and r.pos_error < args.pos_tol,
              "(%s, err %.2f mm, pitch err %.2f deg, %.2f s)" % (r.message, r.pos_error * 1000, math.degrees(r.pitch_error), r.duration))
        contacts(n)

    print("C. orientation, dry run, rejections")
    q = (math.cos(0.25), 0.0, math.sin(0.25), 0.0)   # +0.5 rad about y = gripper pitched down 28.6 deg
    r = move_pose(pose_req([0.27, 0.0, 0.07], mode=MoveToPoseRequest.MODE_POSE, quat=q))
    check("MODE_POSE pitch -28.6 deg", r.success and r.pos_error < args.pos_tol and abs(r.pitch_error) < math.radians(3),
          "(%s, pos %.2f mm, pitch err %.2f deg, orient err %.3f rad)" % (r.message, r.pos_error * 1000, math.degrees(r.pitch_error), r.orient_error))
    r = move_pose(pose_req([0.28, 0.05, 0.0305], dry=True))
    check("dry run does not move", r.success and np.allclose(q_now(), q_now()) and r.duration > 0)
    for name, tgt in (("out of reach", [0.7, 0, 0.05]), ("below the floor", [0.25, 0, 0.0]), ("behind the base", [-0.2, 0, 0.1])):
        r = move_pose(pose_req(tgt))
        check("reject %s" % name, not r.success, "(%s)" % r.message[:80])
    contacts("section C")
    go_home()
    print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: %s" % "; ".join(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
