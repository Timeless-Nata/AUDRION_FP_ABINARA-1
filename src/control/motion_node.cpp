// motion_node.cpp - ROS wrapper of the arm core (Phase 3b). The planning and execution code is the pure C++
// library arm_core; this file only converts ROS messages and bridges RobotInterface / Clock to ROS topics.
//
//   subscribes  /arm/joint_states   (sensor_msgs/JointState)
//   publishes   /arm/joint_command  (sensor_msgs/JointState, names joint_1..joint_4)
//   services    /motion/move_to_pose (MoveToPose)   /motion/move_joints (MoveJoints)   [same definitions as Phase 3 (Python)]
//
// Hardware later: only RosRobot (below) is replaced by a serial backend; nothing else changes.
#include <cmath>
#include <map>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "autonomous_manipulator/MoveJoints.h"
#include "autonomous_manipulator/MoveToPose.h"
#include "autonomous_manipulator/executor.hpp"
#include "autonomous_manipulator/motion_planner.hpp"

using namespace arm;

namespace {
const char* kJointNames[kArmDof] = {"joint_1", "joint_2", "joint_3", "joint_4"};

class RosRobot : public RobotInterface, public Clock {
 public:
  RosRobot(ros::NodeHandle& nh, const std::string& state_topic, const std::string& cmd_topic) {
    sub_ = nh.subscribe(state_topic, 20, &RosRobot::onState, this);
    pub_ = nh.advertise<sensor_msgs::JointState>(cmd_topic, 1);
  }
  bool readJoints(JointVec& q) override {
    std::lock_guard<std::mutex> l(m_);
    if (!have_) return false;
    q = q_;
    return true;
  }
  void sendJoints(const JointVec& q) override {
    sensor_msgs::JointState m;
    for (int i = 0; i < kArmDof; ++i) {
      m.name.push_back(kJointNames[i]);
      m.position.push_back(q[i]);
    }
    pub_.publish(m);
  }
  double now() override { return ros::WallTime::now().toSec(); }
  void sleep(double s) override { ros::WallDuration(s).sleep(); }
  bool waitForState(double timeout) {
    const double t0 = now();
    while (ros::ok() && now() - t0 < timeout) {
      {
        std::lock_guard<std::mutex> l(m_);
        if (have_) return true;
      }
      sleep(0.05);
    }
    return false;
  }

 private:
  void onState(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> l(m_);
    int found = 0;
    for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k)
      for (int i = 0; i < kArmDof; ++i)
        if (msg->name[k] == kJointNames[i]) {
          q_[i] = msg->position[k];
          ++found;
        }
    if (found == kArmDof) have_ = true;
  }
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::mutex m_;
  JointVec q_{};
  bool have_ = false;
};

// elevation of the tool x axis from a ROS quaternion (x = approach, positive rotation about y pitches DOWN)
double pitchFromQuaternion(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-9) return 0.0;
  x /= n; y /= n; z /= n; w /= n;
  return std::asin(std::max(-1.0, std::min(1.0, 2 * (x * z - w * y))));
}

class MotionNode {
 public:
  MotionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : model_(RobotModel::loadFromFile(param<std::string>(pnh, "robot_model_path", "config/robot_model.txt"))),
        kin_(model_),
        chk_(model_, CollisionOptions::withMargin(param(pnh, "collision_margin", 0.002))),
        ik_(kin_, &chk_),
        robot_(nh, param<std::string>(pnh, "joint_states_topic", "/arm/joint_states"),
               param<std::string>(pnh, "joint_command_topic", "/arm/joint_command")) {
    cfg_.limits.vmax = param(pnh, "vmax", 1.0);
    cfg_.limits.amax = param(pnh, "amax", 4.0);
    cfg_.tool_speed = param(pnh, "tool_speed", 0.08);
    cfg_.joint_margin = param(pnh, "safety_margin", 0.03);
    cfg_.min_tool_z = param(pnh, "min_tool_z", 0.012);
    cfg_.finger_angle = param(pnh, "finger_angle", -0.7);
    cfg_.ik.finger_angle = cfg_.finger_angle;
    exec_.rate_hz = param(pnh, "control_rate", 100.0);
    exec_.integral_gain = param(pnh, "integral_gain", 0.0);  // simulation only; keep 0 on hardware
    exec_.settle_tol = param(pnh, "settle_tol", 0.004);
    exec_.settle_time = param(pnh, "settle_time", 0.25);
    exec_.settle_timeout = param(pnh, "settle_timeout", 3.0);
    std::vector<double> home;
    pnh.getParam("home_q", home);
    planner_.reset(new MotionPlanner(kin_, chk_, ik_, cfg_));
    if (home.size() == kArmDof) {
      JointVec h;
      for (int i = 0; i < kArmDof; ++i) h[i] = home[i];
      planner_->setViaPoints({h});  // fallback route through the home pose
    }
    s1_ = nh.advertiseService("/motion/move_to_pose", &MotionNode::onMoveToPose, this);
    s2_ = nh.advertiseService("/motion/move_joints", &MotionNode::onMoveJoints, this);
    ROS_INFO("motion_node ready: %zu collision pairs, vmax %.2f rad/s, amax %.2f rad/s^2", chk_.pairCount(), cfg_.limits.vmax,
             cfg_.limits.amax);
  }
  RosRobot& robot() { return robot_; }

 private:
  template <class T>
  static T param(ros::NodeHandle& n, const std::string& key, const T& def) {
    T v = def;
    n.param(key, v, def);
    return v;
  }

  bool run(const PlanResult& plan, bool dry_run, std::string& msg) {
    if (!plan.ok) {
      msg = plan.message;
      return false;
    }
    if (dry_run) {
      msg = "ok (dry run)";
      return true;
    }
    const ExecResult e = executeTrajectory(plan.traj, robot_, robot_, exec_);
    msg = e.message;
    return e.ok;
  }

  bool onMoveToPose(autonomous_manipulator::MoveToPose::Request& req, autonomous_manipulator::MoveToPose::Response& res) {
    std::unique_lock<std::mutex> lock(busy_, std::try_to_lock);
    if (!lock.owns_lock()) {
      res.message = "busy: another motion is running";
      return true;
    }
    JointVec q0;
    if (!robot_.readJoints(q0)) {
      res.message = "no /arm/joint_states yet";
      return true;
    }
    const auto& p = req.target.pose.position;
    const auto& o = req.target.pose.orientation;
    const bool pose_mode = req.mode == autonomous_manipulator::MoveToPoseRequest::MODE_POSE;
    const double pitch = pose_mode ? pitchFromQuaternion(o.x, o.y, o.z, o.w) : 0.0;
    const PlanResult plan = planner_->planPose(q0, Vec3{p.x, p.y, p.z}, pitch, pose_mode ? PoseMode::Pose : PoseMode::Position,
                                               req.prefer_pitch, req.linear, req.duration);
    res.success = run(plan, req.dry_run, res.message);
    res.duration = plan.ok ? plan.traj.duration() : 0.0;
    for (double v : plan.q_goal) res.q_goal.push_back(v);
    if (plan.ok && res.success && !req.dry_run) {
      JointVec q;
      if (robot_.readJoints(q)) {
        const Pose T = kin_.toolPose(q);
        res.pos_error = std::sqrt(std::pow(T.p.x - p.x, 2) + std::pow(T.p.y - p.y, 2) + std::pow(T.p.z - p.z, 2));
        res.pitch_error = (pose_mode ? pitch : kin_.toolPitch(plan.q_goal)) - Kinematics::pitchOf(T);
      }
    } else {
      res.pos_error = plan.pos_error;
      res.pitch_error = plan.pitch_error;
    }
    res.orient_error = std::fabs(res.pitch_error);  // only the pitch is controllable and compared (roll / yaw are not computed)
    return true;
  }

  bool onMoveJoints(autonomous_manipulator::MoveJoints::Request& req, autonomous_manipulator::MoveJoints::Response& res) {
    std::unique_lock<std::mutex> lock(busy_, std::try_to_lock);
    if (!lock.owns_lock()) {
      res.message = "busy: another motion is running";
      return true;
    }
    JointVec q0;
    if (!robot_.readJoints(q0)) {
      res.message = "no /arm/joint_states yet";
      return true;
    }
    if (req.name.size() != req.position.size()) {
      res.message = "name and position must have the same length";
      return true;
    }
    JointVec goal = q0;
    for (size_t k = 0; k < req.name.size(); ++k) {
      bool known = false;
      for (int i = 0; i < kArmDof; ++i)
        if (req.name[k] == kJointNames[i]) {
          goal[i] = req.position[k];
          known = true;
        }
      if (!known) {
        res.message = "unknown joint '" + req.name[k] + "' (use joint_1..joint_4)";
        return true;
      }
    }
    const PlanResult plan = planner_->planJoints(q0, goal, req.duration);
    res.success = run(plan, req.dry_run, res.message);
    res.duration = plan.ok ? plan.traj.duration() : 0.0;
    for (double v : goal) res.q_goal.push_back(v);
    const Pose T = kin_.toolPose(goal);
    res.tool_position = {static_cast<float>(T.p.x), static_cast<float>(T.p.y), static_cast<float>(T.p.z)};
    res.tool_pitch = Kinematics::pitchOf(T);
    return true;
  }

  RobotModel model_;
  Kinematics kin_;
  CollisionChecker chk_;
  InverseKinematics ik_;
  RosRobot robot_;
  PlannerConfig cfg_;
  ExecOptions exec_;
  std::unique_ptr<MotionPlanner> planner_;
  std::mutex busy_;
  ros::ServiceServer s1_, s2_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_node");
  ros::NodeHandle nh, pnh("~");
  try {
    MotionNode node(nh, pnh);
    if (!node.robot().waitForState(10.0)) ROS_WARN("no /arm/joint_states within 10 s (is the simulation running?)");
    ros::AsyncSpinner spinner(3);  // the service callbacks block while a motion runs; the state callback needs its own thread
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("motion_node: %s", e.what());
    return 1;
  }
  return 0;
}
