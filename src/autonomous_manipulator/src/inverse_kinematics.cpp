#include <cmath>

#include "autonomous_manipulator/robot_model.hpp"

namespace am {
using namespace robot;

bool withinLimits(const JointVec& q, double margin, std::string* which) {
  for (int i = 0; i < 4; ++i) {
    if (!(q[i] >= kLower[i] + margin - 1e-9 && q[i] <= kUpper[i] - margin + 1e-9)) {
      if (which) *which = kJointNames[i];
      return false;
    }
  }
  return true;
}

double orientationError(const Pose& ee, double desired_pitch) {
  Vec3 a = ee.R.col(0);
  double psi = std::atan2(a.y, a.x);
  Vec3 d{std::cos(desired_pitch) * std::cos(psi), std::cos(desired_pitch) * std::sin(psi), -std::sin(desired_pitch)};
  double axis_err = std::acos(clampd(dot(a, d), -1.0, 1.0));
  double roll_err = std::asin(clampd(ee.R.col(1).z, -1.0, 1.0));  // jaw plane must stay level (hand-Y horizontal)
  return std::sqrt(axis_err * axis_err + roll_err * roll_err);
}

namespace {
// Closed-form solution. Returns false with a reason when no solution exists.
bool analytic(const Vec3& t, double pitch, JointVec& q, std::string& why) {
  const double r = std::hypot(t.x, t.y);
  if (r < std::fabs(kLateral) + 1e-6) {
    why = "target too close to the hip axis";
    return false;
  }
  const double th = std::atan2(t.y, t.x);
  // r*sin(th - hip) = kLateral  ->  hip = th - asin(kLateral / r)
  const double hip = th - std::asin(kLateral / r);
  const double X = r * std::cos(th - hip);  // coordinate along the arm plane
  // Planar chain in (X,z): shoulder pivot S, p = S + L1*u(s) + (L2)*u(s+a) + Tool*u(s+a+w), u(t)=(cos t,-sin t)
  const double cx = X - kTool * std::cos(pitch) - kShoulderX;
  const double cz = t.z + kTool * std::sin(pitch) - (kHipZ + kShoulderZ);
  const double D2 = cx * cx + cz * cz;
  const double ca = (D2 - kL1 * kL1 - kL2 * kL2) / (2 * kL1 * kL2);
  if (ca > 1.0 || ca < -1.0) {
    why = "target outside the arm's reach for this tool pitch";
    return false;
  }
  const double a = std::acos(ca);  // joint 'arm' range is [0,3] => only the +acos branch exists
  const double s = std::atan2(-kL2 * std::sin(a), kL1 + kL2 * std::cos(a)) - std::atan2(cz, cx);
  const double w = pitch - s - a;
  q = {hip, s, a, w};
  return true;
}
}  // namespace

IKResult solveIK(const Vec3& target, double pitch, const IKOptions& opt) {
  IKResult res;
  res.reason = "no solution";
  std::string last_reason;
  const int n = static_cast<int>(std::floor(opt.pitch_tol / opt.pitch_step + 1e-9));
  for (int k = 0; k <= 2 * n; ++k) {
    // offsets 0, +1, -1, +2, -2 ... (times pitch_step)
    int idx = (k + 1) / 2;
    double off = ((k % 2) ? 1.0 : -1.0) * idx * opt.pitch_step;
    if (k == 0) off = 0.0;
    JointVec q;
    std::string why;
    if (!analytic(target, pitch + off, q, why)) {
      last_reason = why;
      continue;
    }
    std::string which;
    if (!withinLimits(q, opt.joint_margin, &which)) {
      last_reason = "joint limit reached: " + which;
      continue;
    }
    Pose fk = forwardKinematics(q);
    double perr = norm(fk.p - target);
    if (perr > opt.pos_tol) {
      last_reason = "FK residual too large";
      continue;
    }
    double oerr = orientationError(fk, pitch);
    if (oerr > opt.pitch_tol + 0.02) {  // 0.02 rad allowance for hip-yaw vs. radial direction
      last_reason = "orientation tolerance exceeded";
      continue;
    }
    res.ok = true;
    res.q = q;
    res.pos_err = perr;
    res.orient_err = oerr;
    res.pitch = pitch + off;
    res.reason.clear();
    return res;
  }
  if (!last_reason.empty()) res.reason = last_reason;
  return res;
}

}  // namespace am
