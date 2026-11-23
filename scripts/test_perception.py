#!/usr/bin/env python3
"""Phase 4 check: are simulated objects identified (colour) and localised (position) correctly?

Run while simulation.launch is running:   rosrun autonomous_manipulator test_perception.py
Ground truth = the dumbbell bodies (db_<colour>_<n>) in the MJCF scene (their initial pose, so run it
before the arm pushes anything; /sim/reset restores it). Detections from /perception/objects are
collected for --duration seconds and matched to the nearest ground-truth object of the SAME colour.

  PASS  every detection matches a ground-truth object (no false positives / wrong colours) AND the mean
        position error of every object that was seen is <= --tol, AND at least one object was seen.
  Objects that were never in the camera's view are only listed (the camera sees a small part of the arena).

--sweep pans joint_1 over its range during the test so that more of the arena passes through the view;
--pose J1 J2 J3 J4 first moves the arm to that pose (e.g. to tilt the camera toward the floor).
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import rospkg
import rospy
from sensor_msgs.msg import JointState

from autonomous_manipulator.msg import DetectedObjectArray


def load_truth(scene):
    truth = []
    for b in ET.parse(scene).getroot().iter("body"):
        name = b.get("name", "")
        if name.startswith("db_") and b.get("pos"):
            x, y, z = (float(v) for v in b.get("pos").split())
            truth.append((name, name.split("_")[1], x, y))
    return truth


def main():
    default_scene = rospkg.RosPack().get_path("autonomous_manipulator") + "/mjcf/scene.xml"
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=default_scene)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--tol", type=float, default=0.02, help="max mean position error per object [m]")
    ap.add_argument("--match-radius", type=float, default=0.06, help="detection further than this from every "
                    "same-colour object is a false positive [m]")
    ap.add_argument("--sweep", action="store_true", help="pan joint_1 from -1.2 to +1.2 rad during the test")
    ap.add_argument("--pose", type=float, nargs=4, metavar=("J1", "J2", "J3", "J4"))
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_perception")

    truth = load_truth(args.scene)
    if not truth:
        print("FAIL: no db_* bodies with a pos attribute found in %s" % args.scene)
        return 1
    errs = defaultdict(list)
    false_pos, frames, n_det = [], [0], [0]
    rng_checked, rng_ok = [0], [0]

    def cb(m):
        frames[0] += 1
        for o in m.objects:
            n_det[0] += 1
            same = [(math.hypot(o.position.x - x, o.position.y - y), n) for n, c, x, y in truth if c == o.color]
            d, n = min(same, default=(float("inf"), None))
            if d <= args.match_radius:
                errs[n].append(d)
            else:
                false_pos.append((o.color, o.position.x, o.position.y, d))
            if o.range_checked:
                rng_checked[0] += 1
                rng_ok[0] += int(o.range_ok)

    pub = rospy.Publisher("/arm/joint_command", JointState, queue_size=1)
    rospy.Subscriber("/perception/objects", DetectedObjectArray, cb, queue_size=20)
    rospy.sleep(1.0)

    names = ["joint_1", "joint_2", "joint_3", "joint_4"]
    if args.pose:
        pub.publish(JointState(name=names, position=list(args.pose)))
        rospy.sleep(3.0)
    t0 = rospy.Time.now()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < args.duration:
        if args.sweep:
            frac = (rospy.Time.now() - t0).to_sec() / args.duration
            pub.publish(JointState(name=["joint_1"], position=[-1.2 + 2.4 * frac]))
        rate.sleep()

    if frames[0] == 0:
        print("FAIL: no /perception/objects messages (is the camera enabled and perception_node running?)")
        return 1
    print("frames %d, detections %d" % (frames[0], n_det[0]))
    ok = True
    for name, colour, x, y in truth:
        e = errs.get(name)
        if e:
            mean = sum(e) / len(e)
            flag = "PASS" if mean <= args.tol else "FAIL"
            ok &= mean <= args.tol
            print("  %-12s truth (%.3f, %.3f)  seen %3d x  mean err %.1f mm  max %.1f mm  %s" %
                  (name, x, y, len(e), 1000 * mean, 1000 * max(e), flag))
        else:
            print("  %-12s truth (%.3f, %.3f)  not seen" % (name, x, y))
    for c, x, y, d in false_pos[:10]:
        print("  FALSE POSITIVE: %s at (%.3f, %.3f), nearest same-colour object %.0f mm away" % (c, x, y, 1000 * d))
    if false_pos:
        print("  %d false positive detections in total" % len(false_pos))
    ok &= not false_pos
    ok &= bool(errs)
    if rng_checked[0]:
        print("range cross-check: %d / %d detections consistent with the HC-SR04" % (rng_ok[0], rng_checked[0]))
    else:
        print("range cross-check: sensor axis never pointed at a detected object (not an error)")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
