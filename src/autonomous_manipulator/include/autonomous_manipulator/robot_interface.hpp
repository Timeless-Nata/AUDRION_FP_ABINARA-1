// ROS-side interface of the mission node to the simulated robot: measured state in, position commands out.
#pragma once
#include <array>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64MultiArray.h>

#include "autonomous_manipulator/trajectory_planner.hpp"

namespace am {

class RobotInterface {
 public:
  explicit RobotInterface(ros::NodeHandle& nh);
  bool waitForState(double wall_timeout_s);       // joint states + valid simulation clock
  JointVec armQ() const;
  JointVec armQd() const;
  JointVec armEffort() const;
  std::array<double, 2> fingerQ() const;
  std::array<double, 2> fingerQd() const;
  bool range(double& r) const;                    // false if no echo
  void setFingerCommand(double angle);
  double fingerCommand() const { return finger_cmd_; }
  void setArmCommand(const JointVec& q);
  // Streams a trajectory at 100 Hz of simulation time and then holds its end point. Returns false if ROS shuts down.
  bool execute(const JointTrajectory& tr);
  bool hold(double seconds);                      // keep publishing the current command
  bool waitArmSettled(double tol_rad, double timeout_s);
  JointVec meanEffort(double seconds);            // average arm effort while holding
  bool setWeld(bool on, std::string& msg);

 private:
  void publishCommand();
  void onJoints(const sensor_msgs::JointStateConstPtr& m);
  void onRange(const sensor_msgs::RangeConstPtr& m);

  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber js_sub_, range_sub_;
  ros::ServiceClient weld_cli_;
  mutable std::mutex mtx_;
  JointVec q_{}, qd_{}, eff_{}, arm_cmd_{};
  std::array<double, 2> fq_{}, fqd_{};
  double finger_cmd_ = 0.9, range_ = -1;
  bool have_js_ = false, have_cmd_ = false;
};

}  // namespace am
