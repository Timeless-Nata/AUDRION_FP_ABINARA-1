#include "autonomous_manipulator/robot_model.hpp"

namespace am {
using namespace robot;

Pose handPose(const JointVec& q) {
  const Mat3 Rh = rotZ(q[0]);
  Vec3 p1 = Vec3{0, 0, kHipZ} + Rh * Vec3{kShoulderX, 0, kShoulderZ};
  Mat3 R1 = Rh * rotY(q[1]);
  Vec3 p2 = p1 + R1 * Vec3{kElbowX, kElbowY, 0};
  Mat3 R2 = R1 * rotY(q[2]);
  Vec3 p3 = p2 + R2 * Vec3{kHandX, kHandY, 0};
  Mat3 R3 = R2 * rotY(q[3]);
  const Vec3 piv{kHandPivotX, 0, 0};
  Pose out;
  out.p = p3 + R2 * piv - R3 * piv;  // hinge "hand" is located at pos=(0.0075,0,0) of base_hand
  out.R = R3;
  return out;
}

Pose forwardKinematics(const JointVec& q) {
  Pose h = handPose(q);
  Pose e;
  e.R = h.R;
  e.p = h.p + h.R * Vec3{kEeX, 0, 0};
  return e;
}

Pose cameraPose(const JointVec& q) {
  Pose h = handPose(q);
  Pose c;
  c.p = h.p + h.R * Vec3{kCamX, kCamY, kCamZ};
  // xyaxes="0 -1 0  0 0 1": x_cam = -y_hand, y_cam = +z_hand, z_cam = x_cam x y_cam = -x_hand
  Vec3 xc = -1.0 * h.R.col(1), yc = h.R.col(2), zc = -1.0 * h.R.col(0);
  for (int i = 0; i < 3; ++i) {
    c.R.m[0][i] = i == 0 ? xc.x : (i == 1 ? yc.x : zc.x);
    c.R.m[1][i] = i == 0 ? xc.y : (i == 1 ? yc.y : zc.y);
    c.R.m[2][i] = i == 0 ? xc.z : (i == 1 ? yc.z : zc.z);
  }
  return c;
}

Pose ultrasonicPose(const JointVec& q) {
  Pose h = handPose(q);
  Pose u;
  u.p = h.p + h.R * Vec3{kUsX, 0, kUsZ};
  u.R = h.R;
  return u;
}

Vec3 ultrasonicDirection(const JointVec& q) { return handPose(q).R.col(0); }

Vec3 eeOnPlane(double hip, double X, double z) {
  // Rz(hip) * (X, kLateral)
  return {X * std::cos(hip) - kLateral * std::sin(hip), X * std::sin(hip) + kLateral * std::cos(hip), z};
}

}  // namespace am
