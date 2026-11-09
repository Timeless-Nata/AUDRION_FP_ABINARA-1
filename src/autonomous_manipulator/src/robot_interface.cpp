#include "autonomous_manipulator/robot_interface.hpp"

#include <cmath>
#include <std_srvs/SetBool.h>

namespace am {

RobotInterface::RobotInterface(ros::NodeHandle& nh) : nh_(nh) {
  cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/arm/joint_command", 5);
  js_sub_ = nh_.subscribe("/joint_states", 20, &RobotInterface::onJoints, this);
  range_sub_ = nh_.subscribe("/ultrasonic/range", 20, &RobotInterface::onRange, this);
  weld_cli_ = nh_.serviceClient<std_srvs::SetBool>("/sim/grasp_weld");
}

void RobotInterface::onJoints(const sensor_msgs::JointStateConstPtr& m) {
  if (m->position.size() < 6 || m->velocity.size() < 6 || m->effort.size() < 6) return;
  std::lock_guard<std::mutex> l(mtx_);
  for (int i = 0; i < 4; ++i) { q_[i] = m->position[i]; qd_[i] = m->velocity[i]; eff_[i] = m->effort[i]; }
  fq_ = {m->position[4], m->position[5]};
  fqd_ = {m->velocity[4], m->velocity[5]};
  have_js_ = true;
}
void RobotInterface::onRange(const sensor_msgs::RangeConstPtr& m) {
  std::lock_guard<std::mutex> l(mtx_);
  range_ = std::isfinite(m->range) ? m->range : -1.0;
}

bool RobotInterface::waitForState(double wall_timeout_s) {
  ros::WallTime t0 = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - t0).toSec() < wall_timeout_s) {
    {
      std::lock_guard<std::mutex> l(mtx_);
      if (have_js_ && ros::Time::now().toSec() > 0.05) {
        if (!have_cmd_) { arm_cmd_ = q_; have_cmd_ = true; }
        return true;
      }
    }
    ros::WallDuration(0.05).sleep();
  }
  return false;
}

JointVec RobotInterface::armQ() const { std::lock_guard<std::mutex> l(mtx_); return q_; }
JointVec RobotInterface::armQd() const { std::lock_guard<std::mutex> l(mtx_); return qd_; }
JointVec RobotInterface::armEffort() const { std::lock_guard<std::mutex> l(mtx_); return eff_; }
std::array<double, 2> RobotInterface::fingerQ() const { std::lock_guard<std::mutex> l(mtx_); return fq_; }
std::array<double, 2> RobotInterface::fingerQd() const { std::lock_guard<std::mutex> l(mtx_); return fqd_; }
bool RobotInterface::range(double& r) const {
  std::lock_guard<std::mutex> l(mtx_);
  r = range_;
  return range_ >= 0;
}
void RobotInterface::setFingerCommand(double a) { std::lock_guard<std::mutex> l(mtx_); finger_cmd_ = a; }
void RobotInterface::setArmCommand(const JointVec& q) { std::lock_guard<std::mutex> l(mtx_); arm_cmd_ = q; have_cmd_ = true; }

void RobotInterface::publishCommand() {
  std_msgs::Float64MultiArray m;
  {
    std::lock_guard<std::mutex> l(mtx_);
    for (int i = 0; i < 4; ++i) m.data.push_back(arm_cmd_[i]);
    m.data.push_back(finger_cmd_);
    m.data.push_back(finger_cmd_);
  }
  cmd_pub_.publish(m);
}

bool RobotInterface::execute(const JointTrajectory& tr) {
  if (tr.empty()) return true;
  ros::Rate rate(100.0);
  ros::Time t0 = ros::Time::now();
  while (ros::ok()) {
    double t = (ros::Time::now() - t0).toSec();
    setArmCommand(tr.sample(t));
    publishCommand();
    if (t >= tr.duration()) break;
    rate.sleep();
  }
  setArmCommand(tr.q.back());
  publishCommand();
  return ros::ok();
}

bool RobotInterface::hold(double seconds) {
  ros::Rate rate(100.0);
  ros::Time t0 = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - t0).toSec() < seconds) {
    publishCommand();
    rate.sleep();
  }
  return ros::ok();
}

bool RobotInterface::waitArmSettled(double tol, double timeout) {
  ros::Rate rate(100.0);
  ros::Time t0 = ros::Time::now();
  int good = 0;
  while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout) {
    publishCommand();
    JointVec q = armQ(), qd = armQd(), c;
    { std::lock_guard<std::mutex> l(mtx_); c = arm_cmd_; }
    bool ok = true;
    for (int i = 0; i < 4; ++i) ok = ok && std::fabs(q[i] - c[i]) < tol && std::fabs(qd[i]) < 0.01;
    good = ok ? good + 1 : 0;
    if (good >= 20) return true;
    rate.sleep();
  }
  return false;
}

JointVec RobotInterface::meanEffort(double seconds) {
  ros::Rate rate(100.0);
  ros::Time t0 = ros::Time::now();
  JointVec acc{};
  int n = 0;
  while (ros::ok() && (ros::Time::now() - t0).toSec() < seconds) {
    publishCommand();
    JointVec e = armEffort();
    for (int i = 0; i < 4; ++i) acc[i] += e[i];
    ++n;
    rate.sleep();
  }
  if (n > 0) for (auto& a : acc) a /= n;
  return acc;
}

bool RobotInterface::setWeld(bool on, std::string& msg) {
  std_srvs::SetBool s;
  s.request.data = on;
  if (!weld_cli_.call(s)) { msg = "service /sim/grasp_weld unavailable"; return false; }
  msg = s.response.message;
  return s.response.success;
}

}  // namespace am
