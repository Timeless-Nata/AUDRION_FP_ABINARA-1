// motion_planner.hpp - collision-aware motion planning for the 4-DOF arm (pure C++17, no ROS / MuJoCo).
//
// Turns "go to this joint configuration / tool pose / along this line" into a time-parameterised Trajectory, or
// explains why it cannot: unreachable, links would overlap at the goal, or on the way (joint-space path check).
#pragma once
#include <string>
#include <vector>

#include "autonomous_manipulator/collision_checker.hpp"
#include "autonomous_manipulator/inverse_kinematics.hpp"
#include "autonomous_manipulator/trajectory.hpp"

namespace arm {

struct PlannerConfig {
  JointLimits limits;                 // velocity / acceleration used for timing
  double joint_margin = 0.03;         // [rad] kept away from the joint limits at goals and on lines
  double path_step_rad = 0.02;        // collision sampling step of joint-space segments
  double line_step = 0.004;           // [m] spacing of IK samples on Cartesian lines
  double max_joint_jump = 0.12;       // [rad] largest joint change between consecutive line samples (branch flip guard)
  double tool_speed = 0.08;           // [m/s] speed used to time Cartesian lines
  double min_tool_z = 0.012;          // [m] the tool never goes below this on a line
  double finger_angle = -0.7;         // finger_active angle assumed for collision checks (open gripper)
  IKOptions ik;
};

enum class PoseMode { Pose, Position };  // Pose: position + pitch; Position: pitch chosen near prefer_pitch

struct PlanResult {
  bool ok = false;
  std::string message;
  Trajectory traj;
  JointVec q_goal{};
  double pos_error = 0;       // [m] IK residual at the goal
  double pitch_error = 0;     // [rad]
  double failed_fraction = 0; // lines: fraction of the path where planning failed (0..1)
};

class MotionPlanner {
 public:
  MotionPlanner(const Kinematics& kin, const CollisionChecker& chk, const InverseKinematics& ik,
                const PlannerConfig& cfg = PlannerConfig())
      : kin_(kin), chk_(chk), ik_(ik), cfg_(cfg) {}

  // Intermediate configurations tried (in order) when the direct joint-space path from a to b collides.
  void setViaPoints(std::vector<JointVec> via) { via_ = std::move(via); }

  PlanResult planJoints(const JointVec& q0, const JointVec& q_goal, double duration = 0.0) const;
  PlanResult planPose(const JointVec& q0, Vec3 p, double pitch, PoseMode mode, double prefer_pitch, bool linear,
                      double duration = 0.0) const;
  // Straight tool line from the current pose to p with the pitch interpolated linearly to `pitch`.
  PlanResult planLine(const JointVec& q0, Vec3 p, double pitch, double duration = 0.0) const;

  const PlannerConfig& config() const { return cfg_; }

 private:
  bool validGoal(const JointVec& q, std::string& why) const;
  bool pathFree(const JointVec& a, const JointVec& b, std::string& why) const;
  // joint-space path a -> b, possibly through one via point; returns the waypoint list or empty
  std::vector<JointVec> route(const JointVec& a, const JointVec& b, std::string& why) const;

  const Kinematics& kin_;
  const CollisionChecker& chk_;
  const InverseKinematics& ik_;
  PlannerConfig cfg_;
  std::vector<JointVec> via_;
};

}  // namespace arm
