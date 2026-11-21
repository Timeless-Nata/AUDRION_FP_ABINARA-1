#!/usr/bin/env python3
"""Phase 3 ROS node: kinematics + motion control for the 4-DOF arm.

Subscribes  /arm/joint_states (sensor_msgs/JointState)              measured joints from the simulation
Publishes   /arm/joint_command (sensor_msgs/JointState)             ONLY the 4 arm joints (gripper untouched)
            /motion/tool_pose (geometry_msgs/PoseStamped, world)    FK of the measured joints, 20 Hz
Services    /motion/move_to_pose  (MoveToPose)   Cartesian target (pose or position), joint-space or straight line
            /motion/move_joints   (MoveJoints)   joint-space target
            /motion/go_home       (std_srvs/Trigger)
            /motion/stop          (std_srvs/Trigger)  aborts the running motion; holds the current position
Every service supports dry_run (plan + validate only). All blocking calls return after the arm has settled.
"""
import json
import math
import os
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

from autonomous_manipulator.kinematics import ARM_JOINTS, ArmKinematics, mat_to_quat
from autonomous_manipulator.motion_control import DEFAULTS, MotionController
from autonomous_manipulator.srv import (MoveJoints, MoveJointsResponse, MoveToPose, MoveToPoseRequest,
                                        MoveToPoseResponse)


class MotionNode:
    def __init__(self):
        p = rospy.get_param
        mjcf = p("~mjcf_path")
        self.kin = ArmKinematics(mjcf, tool_site=p("~tool_site", "grasp_center"))
        cfg = {k: p("~" + k) for k in DEFAULTS if rospy.has_param("~" + k)}
        if rospy.has_param("~pitch_tol_deg"):
            cfg["pitch_tol"] = math.radians(p("~pitch_tol_deg"))
        self.mc = MotionController(self.kin, **cfg)
        self.home_q = np.array(p("~home_q", [0.0, 0.8901, 0.4538, -0.4363]), float)
        self.q = None
        self.q_lock = threading.Lock()
        self.cmd_pub = rospy.Publisher(p("~joint_command_topic", "/arm/joint_command"), JointState, queue_size=1)
        self.pose_pub = rospy.Publisher("/motion/tool_pose", PoseStamped, queue_size=1)
        rospy.Subscriber(p("~joint_states_topic", "/arm/joint_states"), JointState, self.on_joint_state, queue_size=1)
        rospy.Service("/motion/move_to_pose", MoveToPose, self.srv_move_to_pose)
        rospy.Service("/motion/move_joints", MoveJoints, self.srv_move_joints)
        rospy.Service("/motion/go_home", Trigger, self.srv_home)
        rospy.Service("/motion/stop", Trigger, self.srv_stop)
        rospy.Timer(rospy.Duration(0.05), self.publish_pose)
        rospy.loginfo("motion_control_node ready: model %s, tool site '%s', joint limits %s..%s" % (
            os.path.basename(mjcf), self.kin.tool_site, np.round(self.kin.q_lo, 3), np.round(self.kin.q_hi, 3)))

    # ------------------------------------------------------------------ io
    def on_joint_state(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM_JOINTS):
            with self.q_lock:
                self.q = np.array([d[j] for j in ARM_JOINTS])

    def get_q(self):
        with self.q_lock:
            return None if self.q is None else self.q.copy()

    def wait_for_state(self, timeout=3.0):
        t0 = rospy.get_time()
        while self.get_q() is None and rospy.get_time() - t0 < timeout and not rospy.is_shutdown():
            rospy.sleep(0.05)
        return self.get_q()

    def send(self, q):
        m = JointState()
        m.header.stamp = rospy.Time.now()
        m.name = list(ARM_JOINTS)
        m.position = [float(v) for v in q]
        self.cmd_pub.publish(m)

    def publish_pose(self, _evt):
        q = self.get_q()
        if q is None:
            return
        p, R = self.kin.fk(q)
        w, x, y, z = mat_to_quat(R)
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = w, x, y, z
        self.pose_pub.publish(m)

    def run(self, traj, info):
        res = self.mc.execute(traj, self.get_q, self.send, rospy.get_time, rospy.sleep)
        if not res["ok"]:
            return res, float("nan"), float("nan")
        e, pe = self.mc.pose_error(res["q_final"], info["goal_pos"], info["goal_pitch"])
        if e > self.mc.cfg["pos_tol"] or abs(pe) > self.mc.cfg["pitch_tol"]:
            res = dict(res, ok=False, reason="finished %.1f mm / %.1f deg off the target (tolerance %.1f mm / %.1f deg)" % (
                e * 1000, math.degrees(abs(pe)), self.mc.cfg["pos_tol"] * 1000, math.degrees(self.mc.cfg["pitch_tol"])))
        return res, e, pe

    # ------------------------------------------------------------------ services
    def srv_move_to_pose(self, req):
        r = MoveToPoseResponse()
        q0 = self.wait_for_state()
        if q0 is None:
            r.message = "no /arm/joint_states received"
            return r
        if req.target.header.frame_id not in ("", "world"):
            r.message = "unsupported frame '%s' (use world)" % req.target.header.frame_id
            return r
        pos = np.array([req.target.pose.position.x, req.target.pose.position.y, req.target.pose.position.z])
        o = req.target.pose.orientation
        orient = None
        if req.mode == MoveToPoseRequest.MODE_POSE:
            if abs(o.w) + abs(o.x) + abs(o.y) + abs(o.z) < 1e-9:
                r.message = "MODE_POSE needs a valid orientation quaternion"
                return r
            orient = (o.w, o.x, o.y, o.z)
        traj, info = self.mc.plan_pose(q0, pos, orient, prefer_pitch=float(req.prefer_pitch), linear=req.linear,
                                       duration=float(req.duration))
        if traj is None:
            r.message = info["reason"]
            return r
        r.q_goal = [float(v) for v in info["q_goal"]]
        r.duration = float(traj.duration)
        r.pos_error, r.pitch_error, r.orient_error = float(info["pos_err"]), float(info["pitch_err"]), float(info["orient_err"])
        if req.dry_run:
            r.success, r.message = True, "dry run: reachable, planned %.2f s" % traj.duration
            return r
        res, e, pe = self.run(traj, info)
        r.success, r.message = bool(res["ok"]), res["reason"]
        if res["ok"] or not math.isnan(e):
            r.pos_error, r.pitch_error = float(e), float(pe)
        return r

    def srv_move_joints(self, req):
        r = MoveJointsResponse()
        q0 = self.wait_for_state()
        if q0 is None:
            r.message = "no /arm/joint_states received"
            return r
        if len(req.name) != len(req.position):
            r.message = "name/position size mismatch"
            return r
        goal = q0.copy()
        for n, v in zip(req.name, req.position):
            if n not in ARM_JOINTS:
                r.message = "unknown joint '%s'" % n
                return r
            goal[ARM_JOINTS.index(n)] = v
        traj, info = self.mc.plan_joints(q0, goal, float(req.duration))
        if traj is None:
            r.message = info["reason"]
            return r
        r.q_goal = [float(v) for v in goal]
        r.duration = float(traj.duration)
        r.tool_position = [float(v) for v in info["goal_pos"]]
        r.tool_pitch = float(info["goal_pitch"])
        if req.dry_run:
            r.success, r.message = True, "dry run ok"
            return r
        res = self.mc.execute(traj, self.get_q, self.send, rospy.get_time, rospy.sleep)
        r.success, r.message = bool(res["ok"]), res["reason"]
        return r

    def srv_home(self, _req):
        q0 = self.wait_for_state()
        if q0 is None:
            return TriggerResponse(False, "no /arm/joint_states received")
        traj, info = self.mc.plan_joints(q0, self.home_q)
        if traj is None:
            return TriggerResponse(False, info["reason"])
        res = self.mc.execute(traj, self.get_q, self.send, rospy.get_time, rospy.sleep)
        return TriggerResponse(bool(res["ok"]), res["reason"])

    def srv_stop(self, _req):
        self.mc.abort()
        return TriggerResponse(True, "stop requested")



if __name__ == "__main__":
    rospy.init_node("motion_control")
    MotionNode()
    rospy.spin()
