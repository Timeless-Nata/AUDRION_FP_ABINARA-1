#include "autonomous_manipulator/robot_model.hpp"

namespace am {
using namespace robot;

namespace {

// The revolute_2 body has euler="1.5708 0 0" in the MJCF.
// Express Rx(+pi/2) using the rotation helpers already provided by math_utils.
Mat3 shoulderMountRotation() {
  constexpr double half_pi = 1.57079632679489661923;
  return rotZ(-half_pi) * rotY(half_pi) * rotZ(half_pi);
}

// MuJoCo camera xyaxes="0 -1 0  0 0 1".
// Its local basis columns are x=(0,-1,0), y=(0,0,1), z=(-1,0,0).
Mat3 cameraLocalRotation() {
  Mat3 r;
  r.m[0][0] = 0.0;  r.m[0][1] = 0.0;  r.m[0][2] = -1.0;
  r.m[1][0] = -1.0; r.m[1][1] = 0.0;  r.m[1][2] = 0.0;
  r.m[2][0] = 0.0;  r.m[2][1] = 1.0;  r.m[2][2] = 0.0;
  return r;
}

}  // namespace

Pose handPose(const JointVec& q) {
  // Match the MJCF hierarchy exactly. All translations are expressed in the
  // parent body's frame, and each joint rotation is applied in its own frame.
  const Mat3 R_hip = rotZ(q[0]);
  const Vec3 p_hip{0.0, 0.0, kHipZ};

  const Vec3 p_shoulder =
      p_hip + R_hip * Vec3{kShoulderX, 0.0, kShoulderZ};
  const Mat3 R_shoulder =
      R_hip * shoulderMountRotation() * rotZ(q[1]);

  const Vec3 p_arm =
      p_shoulder + R_shoulder * Vec3{kElbowX, kElbowY, 0.0};
  // MJCF arm joint axis is (0, 0, -1).
  const Mat3 R_arm = R_shoulder * rotZ(-q[2]);

  const Vec3 p_hand =
      p_arm + R_arm * Vec3{kHandX, kHandY, 0.0};
  // MJCF hand joint axis is (0, 0, +1), at the base_hand origin.
  const Mat3 R_hand = R_arm * rotZ(q[3]);

  Pose out;
  out.p = p_hand;
  out.R = R_hand;
  return out;
}

Pose forwardKinematics(const JointVec& q) {
  const Pose h = handPose(q);
  Pose ee;
  ee.p = h.p + h.R * Vec3{kEeX, 0.0, 0.0};
  ee.R = h.R;
  return ee;
}

Pose cameraPose(const JointVec& q) {
  const Pose h = handPose(q);
  Pose cam;
  cam.p = h.p + h.R * Vec3{kCamX, kCamY, kCamZ};
  cam.R = h.R * cameraLocalRotation();
  return cam;
}

Pose ultrasonicPose(const JointVec& q) {
  const Pose h = handPose(q);
  Pose us;
  us.p = h.p + h.R * Vec3{kUsX, 0.0, kUsZ};
  us.R = h.R;
  return us;
}

Vec3 ultrasonicDirection(const JointVec& q) {
  // ultrasonic_site uses zaxis="1 0 0", so its local +Z points along
  // the base_hand frame's +X axis.
  return handPose(q).R.col(0);
}

// Kept for compatibility with the existing IK implementation.
Vec3 eeOnPlane(double hip, double X, double z) {
  return {X * std::cos(hip) - kLateral * std::sin(hip),
          X * std::sin(hip) + kLateral * std::cos(hip),
          z};
}

}  // namespace am
