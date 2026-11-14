// Kinematic model of the robot, transcribed from
// arm_gripper_dumbbells_with_sensors.xml.
// Keep these transforms synchronized with the MJCF. mujoco_sim_node verifies
// the analytic poses against MuJoCo at start-up.
#pragma once

#include <array>
#include <string>

#include "autonomous_manipulator/math_utils.hpp"

namespace am {

using JointVec = std::array<double, 4>;  // {hip, shoulder, arm, hand}

namespace robot {
// MJCF body/joint transforms:
// base_link -> base_fixed -> revolute_1(hip) -> revolute_2(shoulder)
// -> revolute_3(arm) -> base_hand(hand) -> ee site.
constexpr double kHipZ = 0.094;  // revolute_1 body pos
constexpr double kShoulderX = -0.016, kShoulderZ = 0.055;  // revolute_2 body pos
constexpr double kElbowX = 0.12471, kElbowY = 0.082043;  // revolute_3 body pos
constexpr double kHandX = 0.0088641, kHandY = -0.21104;  // base_hand body pos
constexpr double kEeX = 0.11;  // site ee pos in base_hand

// Camera origin relative to base_hand:
// color_sensor body pos + color_cam pos.
constexpr double kCamX = 0.0305 + 0.0006;
constexpr double kCamY = 0.0145;
constexpr double kCamZ = 0.0;

// Ultrasonic site position relative to base_hand.
constexpr double kUsX = 0.052;
constexpr double kUsZ = 0.034;

// Compatibility values for code that still references the former planar IK
// constants. These are geometric segment lengths, not a complete IK solution
// for the current offset 3R chain. The IK solver must use the exact transforms.
constexpr double kHandPivotX = 0.0;
constexpr double kLateral = kElbowY + kHandY;
constexpr double kL1 = 0.149269;  // hypot(kElbowX, kElbowY), rounded
constexpr double kL2 = 0.211226;  // hypot(kHandX, kHandY), rounded
constexpr double kTool = kEeX;

// Joint ranges from the current MJCF.
constexpr std::array<double, 4> kLower = {-1.571, -1.571, -1.571, -1.571};
constexpr std::array<double, 4> kUpper = { 1.571,  1.571,  1.571,  1.571};
const char* const kJointNames[4] = {"hip", "shoulder", "arm", "hand"};
}  // namespace robot

// Forward kinematics.
Pose handPose(const JointVec& q);             // body base_hand frame
Pose forwardKinematics(const JointVec& q);    // site ee frame
Pose cameraPose(const JointVec& q);           // MuJoCo camera frame
Pose ultrasonicPose(const JointVec& q);       // ultrasonic site frame/origin
Vec3 ultrasonicDirection(const JointVec& q);

// Inverse kinematics.
struct IKOptions {
  double pitch_tol = 0.02;
  double pitch_step = 0.005;
  double joint_margin = 0.02;
  double pos_tol = 1e-4;
};

struct IKResult {
  bool ok = false;
  JointVec q{};
  double pos_err = 0;
  double orient_err = 0;
  double pitch = 0;
  std::string reason;
};

IKResult solveIK(const Vec3& target, double pitch, const IKOptions& opt = IKOptions());
bool withinLimits(const JointVec& q, double margin, std::string* which = nullptr);
double orientationError(const Pose& ee, double desired_pitch);
Vec3 eeOnPlane(double hip, double X, double z);

}  // namespace am
