// kinematics.hpp - forward kinematics, Jacobian and joint limits of the 4-DOF arm.
// Works on RobotModel only (no MuJoCo / ROS). Frames: world = the simulation/robot base frame.
//
// Tool frame (ROS convention, identity = gripper horizontal pointing along world +x):
//   x = approach direction, y = left, z = up.   `pitch` = elevation of the tool x axis above the horizontal
//   plane (0 = horizontal, negative = pointing down).
#pragma once
#include <array>
#include <string>
#include <vector>

#include "autonomous_manipulator/robot_model.hpp"

namespace arm {

using JointVec = std::array<double, kArmDof>;
using DofVec = std::array<double, kModelDof>;  // joint_1..4 + finger_active

struct Jacobian {
  double J[6][kArmDof];  // rows 0-2: linear velocity of the tool point per rad/s, rows 3-5: angular velocity (world axes)
};

class Kinematics {
 public:
  explicit Kinematics(const RobotModel& model);
  const RobotModel& model() const { return *model_; }

  // World pose of every body for the given dof values (index = RobotModel::bodies).
  std::vector<Pose> bodyPoses(const DofVec& dofs) const;
  Pose toolPose(const JointVec& q) const;
  // Any named frame of the model (tool, hcsr04_sensor, cam_esp32cam, ...); finger_active only matters for finger frames.
  Pose framePose(const std::string& name, const JointVec& q, double finger_active = 0.0) const;

  Jacobian toolJacobian(const JointVec& q) const;
  static double pitchOf(const Pose& tool);                  // elevation of the tool x axis
  JointVec pitchGradient(const JointVec& q) const;          // d(pitch)/dq
  double toolPitch(const JointVec& q) const { return pitchOf(toolPose(q)); }

  const JointVec& lower() const { return lo_; }
  const JointVec& upper() const { return hi_; }
  bool withinLimits(const JointVec& q, double margin = 0.0) const;
  JointVec clampToLimits(const JointVec& q, double margin = 0.0) const;
  const std::array<int, kArmDof>& armBodies() const { return arm_; }

 private:
  const RobotModel* model_;
  std::array<int, kArmDof> arm_;
  int tool_frame_;
  JointVec lo_{}, hi_{};
};

}  // namespace arm
