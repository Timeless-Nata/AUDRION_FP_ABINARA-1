// Kinematic model of the robot, transcribed from arm_gripper_dumbbells.xml.
// Every constant below is annotated with the MJCF element it comes from.
// mujoco_sim_node re-verifies this model against MuJoCo at start-up (FK self-check).
#pragma once
#include <array>
#include <string>

#include "autonomous_manipulator/math_utils.hpp"

namespace am {

using JointVec = std::array<double, 4>;  // {hip, shoulder, arm, hand}

namespace robot {
// <body name="rotator" pos="0 0 0.095">, joint hip axis +Z
constexpr double kHipZ = 0.095;
// <body name="upper_arm" pos="0.02 0 0.053">, joint shoulder axis +Y
constexpr double kShoulderX = 0.02, kShoulderZ = 0.053;
// <body name="lower_arm" pos="0.136294 0.032 0">, joint arm axis +Y, range 0..3
constexpr double kElbowX = 0.136294, kElbowY = 0.032;
// <body name="base_hand" pos="0.18 -0.036 0">, joint hand axis +Y at pos="0.0075 0 0"
constexpr double kHandX = 0.18, kHandY = -0.036, kHandPivotX = 0.0075;
// <site name="ee" pos="0.11 0 0"> in base_hand
constexpr double kEeX = 0.11;
// <body name="color_sensor" pos="0.0305 0.0145 0"> + <camera name="color_cam" pos="0.0006 0 0" xyaxes="0 -1 0 0 0 1" fovy="20">
constexpr double kCamX = 0.0305 + 0.0006, kCamY = 0.0145, kCamZ = 0.0;
// <body name="ultrasonic" pos="0.052 0 0.034"> with <site name="ultrasonic_site" zaxis="1 0 0">
constexpr double kUsX = 0.052, kUsZ = 0.034;
// Net lateral (hand/arm-plane Y) offset of ee relative to the hip-shoulder plane: 0.032 - 0.036
constexpr double kLateral = kElbowY + kHandY;  // = -0.004
// Planar link lengths used by the analytic IK
constexpr double kL1 = kElbowX;                    // shoulder -> elbow
constexpr double kL2 = kHandX + kHandPivotX;       // elbow -> wrist pivot
constexpr double kTool = kEeX - kHandPivotX;       // wrist pivot -> ee site
// Joint limits from the MJCF (hip, shoulder, arm, hand)
constexpr std::array<double, 4> kLower = {-1.5708, -1.5708, 0.0, -1.5708};
constexpr std::array<double, 4> kUpper = {1.5708, 1.5708, 3.0, 1.5708};
const char* const kJointNames[4] = {"hip", "shoulder", "arm", "hand"};
}  // namespace robot

// ---- Forward kinematics -------------------------------------------------
Pose handPose(const JointVec& q);        // frame of body base_hand
Pose forwardKinematics(const JointVec& q);  // frame at site "ee" (x = approach axis)
Pose cameraPose(const JointVec& q);      // MuJoCo camera frame: x right, y up, looks along -z
Pose ultrasonicPose(const JointVec& q);  // origin of ultrasonic site; ray along column 0 of hand frame
Vec3 ultrasonicDirection(const JointVec& q);

// ---- Inverse kinematics -------------------------------------------------
struct IKOptions {
  double pitch_tol = 0.02;     // allowed deviation from the requested tool pitch [rad]
  double pitch_step = 0.005;   // search step inside the tolerance
  double joint_margin = 0.02;  // keep this far from joint limits [rad]
  double pos_tol = 1e-4;       // accepted FK residual [m]
};

struct IKResult {
  bool ok = false;
  JointVec q{};
  double pos_err = 0;     // |FK(q) - target| [m]
  double orient_err = 0;  // angle between achieved and requested tool axis (+roll error) [rad]
  double pitch = 0;       // pitch actually used
  std::string reason;
};

// target: desired ee position (world). pitch: tool pitch, positive = pointing DOWN, 0 = level.
// The tool yaw is determined by the hip (base rotation); roll is fixed by the mechanism (jaw plane level).
IKResult solveIK(const Vec3& target, double pitch, const IKOptions& opt = IKOptions());
bool withinLimits(const JointVec& q, double margin, std::string* which = nullptr);
double orientationError(const Pose& ee, double desired_pitch);
// Position of the ee given the hip angle, arm-plane coordinate X (distance along the hip direction) and z.
Vec3 eeOnPlane(double hip, double X, double z);

}  // namespace am
