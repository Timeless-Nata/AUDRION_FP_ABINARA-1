#!/usr/bin/env python3
"""Phase 2 check: does each simulated joint follow its command, without unwanted contacts?

Run while simulation.launch is running:   rosrun autonomous_manipulator test_joint_control.py
For every actuated joint: step it by +/-0.3 rad (away from its limit), wait, compare the
measured position with the target, then restore. The gripper test also checks that
finger_passive mirrors finger_active (gear constraint: q_passive = -q_active).
Note: simulated servos are P(D) position actuators, so a small steady-state error under
gravity is expected; --tol sets the accepted error (rad).

Contact check: the simulation node logs every contact that involves a robot geom after EVERY
physics step and hands the log over via /sim/contact_report. A joint test FAILS if, during its
step + restore motion, any robot-robot or robot-environment contact (resting base<->plate
excluded) penetrates deeper than --pen-tol metres. The home pose is checked the same way.
Limitation: MuJoCo never creates contacts between a body and its direct parent, so overlap
between neighbouring links cannot be detected here.
"""
import argparse
import json
import sys

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

ARM = ["joint_1", "joint_2", "joint_3", "joint_4"]
GRIP = "finger_active"
LIMITS = {"joint_1": (-1.571, 1.571), "joint_2": (-1.571, 1.571), "joint_3": (-1.571, 1.571),
          "joint_4": (-1.571, 1.571), "finger_active": (-0.873, 0.227)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol", type=float, default=0.1, help="joint position error tolerance [rad]")
    ap.add_argument("--settle", type=float, default=3.0)
    ap.add_argument("--pen-tol", type=float, default=0.0005,
                    help="max accepted contact penetration [m]; deeper contacts FAIL")
    ap.add_argument("--no-contacts", action="store_true", help="skip the contact check")
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_joint_control")
    state = {}

    def cb(m):
        state.update(dict(zip(m.name, m.position)))
    rospy.Subscriber("/arm/joint_states", JointState, cb)
    pub = rospy.Publisher("/arm/joint_command", JointState, queue_size=1)

    report = None
    if not args.no_contacts:
        try:
            rospy.wait_for_service("/sim/contact_report", timeout=10)
            report = rospy.ServiceProxy("/sim/contact_report", Trigger)
        except rospy.ROSException:
            print("FAIL: /sim/contact_report not available (rebuild the node, or use --no-contacts)")
            return 1

    def get_contacts():
        """Contacts logged since the previous call (the call also clears the log)."""
        if report is None:
            return []
        return json.loads(report().message)["contacts"]

    def check_contacts(label):
        bad = [c for c in get_contacts() if c["min_dist"] < -args.pen_tol]
        for c in sorted(bad, key=lambda c: c["min_dist"]):
            print("    CONTACT [%s] %s/%s  <->  %s/%s  depth %.2f mm  (%d steps)  [%s]" %
                  (c["kind"], c["body1"], c["geom1"], c["body2"], c["geom2"],
                   -c["min_dist"] * 1000.0, c["count"], label))
        return not bad

    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and not all(k in state for k in ARM + [GRIP, "finger_passive"]):
        if (rospy.Time.now() - t0).to_sec() > 10:
            print("FAIL: no /arm/joint_states with all joints within 10 s")
            return 1
        rospy.sleep(0.1)
    rospy.sleep(0.5)
    names = ARM + [GRIP]
    base = {n: state[n] for n in names}

    def send(targets):
        m = JointState(name=list(targets), position=[targets[k] for k in targets])
        for _ in range(5):
            pub.publish(m)
            rospy.sleep(0.05)
        rospy.sleep(args.settle)

    ok_all = True
    if report is not None:
        get_contacts()                      # discard everything logged before this point
        rospy.sleep(1.0)
        ok = check_contacts("home pose")
        print("%-14s %s" % ("home pose", "PASS (no contacts)" if ok else "FAIL"))
        ok_all &= ok

    for n in names:
        lo, hi = LIMITS[n]
        step = 0.3 if base[n] + 0.3 <= hi - 0.05 else -0.3
        if n == 'joint_3':
            step = -0.3   # +q3 swings the forearm into base/rev1 (collision checker: free only for q3 <= ~0.54)
        tgt = dict(base)
        tgt[n] = base[n] + step
        get_contacts()                      # clear: log covers step + restore of this joint
        send(tgt)
        meas = state[n]
        snap = [state[k] for k in ARM] + [state[GRIP]]   # full pose at the target (for mj_probe.py)
        err = abs(meas - tgt[n])
        others = max(abs(state[k] - base[k]) for k in names if k != n)
        ok = err <= args.tol and others <= args.tol
        if n == GRIP:
            gerr = abs(state["finger_passive"] + state[GRIP])
            ok = ok and gerr <= 0.03
            print("  gear check |q_passive + q_active| = %.4f" % gerr)
        pos_ok = ok
        send(base)
        contact_ok = check_contacts(n)
        ok = ok and contact_ok
        print("%-14s target %+.3f measured %+.3f err %.3f other-joint drift %.3f  position %s  contacts %s  %s" %
              (n, tgt[n], meas, err, others,
               "ok" if pos_ok else "BAD", "ok" if contact_ok else "BAD", "PASS" if ok else "FAIL"))
        if not contact_ok:
            print("    pose at target -> python3 tools/mj_probe.py mjcf/scene.xml " + " ".join("%.4f" % v for v in snap))
        ok_all &= ok
    print("RESULT:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
