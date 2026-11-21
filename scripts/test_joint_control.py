#!/usr/bin/env python3
"""Phase 2 check: does each simulated joint follow its command?

Run while simulation.launch is running:   rosrun autonomous_manipulator test_joint_control.py
For every actuated joint: step it by +/-0.3 rad (away from its limit), wait, compare the
measured position with the target, then restore. The gripper test also checks that
finger_passive mirrors finger_active (gear constraint: q_passive = -q_active).
Note: simulated servos are P(D) position actuators, so a small steady-state error under
gravity is expected; --tol sets the accepted error (rad).
"""
import argparse
import sys

import rospy
from sensor_msgs.msg import JointState

ARM = ["joint_1", "joint_2", "joint_3", "joint_4"]
GRIP = "finger_active"
LIMITS = {"joint_1": (-1.571, 1.571), "joint_2": (-1.571, 1.571), "joint_3": (-1.571, 1.571),
          "joint_4": (-1.571, 1.571), "finger_active": (-0.873, 0.227)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol", type=float, default=0.1)
    ap.add_argument("--settle", type=float, default=3.0)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_joint_control")
    state = {}

    def cb(m):
        state.update(dict(zip(m.name, m.position)))
    rospy.Subscriber("/arm/joint_states", JointState, cb)
    pub = rospy.Publisher("/arm/joint_command", JointState, queue_size=1)
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
    for n in names:
        lo, hi = LIMITS[n]
        step = 0.3 if base[n] + 0.3 <= hi - 0.05 else -0.3
        tgt = dict(base)
        tgt[n] = base[n] + step
        send(tgt)
        err = abs(state[n] - tgt[n])
        others = max(abs(state[k] - base[k]) for k in names if k != n)
        ok = err <= args.tol and others <= args.tol
        if n == GRIP:
            gerr = abs(state["finger_passive"] + state[GRIP])
            ok = ok and gerr <= 0.03
            print("  gear check |q_passive + q_active| = %.4f" % gerr)
        print("%-14s target %+.3f measured %+.3f err %.3f other-joint drift %.3f  %s" %
              (n, tgt[n], state[n], err, others, "PASS" if ok else "FAIL"))
        ok_all &= ok
        send(base)
    print("RESULT:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
