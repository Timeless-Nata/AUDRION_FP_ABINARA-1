#include "autonomous_manipulator/view_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace arm {

bool CameraModel::project(const Pose& cam, Vec3 p, double& u, double& v) const {
  const Vec3 d = p - cam.p;
  const Vec3 f = forward(cam), r = right(cam), up{cam.R(0, 1), cam.R(1, 1), cam.R(2, 1)};
  const double z = dot(d, f);
  if (z <= 1e-6) return false;
  const double fpx = (height / 2.0) / std::tan(fovy_deg * kPi / 360.0);
  u = width / 2.0 + fpx * dot(d, r) / z;
  v = height / 2.0 - fpx * dot(d, up) / z;
  return true;
}

// rotation of v about the unit axis by angle a (Rodrigues)
static Vec3 rotAxis(Vec3 v, Vec3 k, double a) {
  return std::cos(a) * v + std::sin(a) * Vec3{k.y * v.z - k.z * v.y, k.z * v.x - k.x * v.z, k.x * v.y - k.y * v.x} +
         ((1 - std::cos(a)) * dot(k, v)) * k;
}

Pose ViewPlanner::cameraPose(const JointVec& q) const {
  Pose c = kin_.framePose(p_.camera_frame, q, p_.finger_angle);
  if (p_.camera_tilt_deg == 0.0) return c;
  // the joint axes are perpendicular to the arm plane: tilting the camera about that axis changes only its pitch
  const Vec3 f0{-c.R(0, 2), -c.R(1, 2), -c.R(2, 2)};
  const double fh = std::max(1e-9, std::hypot(f0.x, f0.y));
  const Vec3 left{-f0.y / fh, f0.x / fh, 0.0};  // horizontal, perpendicular to the arm plane
  const double a = -p_.camera_tilt_deg * kPi / 180.0;  // rotating about `left` by +a pitches DOWN by a
  Mat3 R;
  for (int col = 0; col < 3; ++col) {
    const Vec3 v = rotAxis({c.R(0, col), c.R(1, col), c.R(2, col)}, left, a);
    R(0, col) = v.x;
    R(1, col) = v.y;
    R(2, col) = v.z;
  }
  c.R = R;
  return c;
}

double ViewPlanner::cameraPitch(const JointVec& q) const {
  const Pose c = cameraPose(q);
  return std::asin(std::max(-1.0, std::min(1.0, -c.R(2, 2))));
}

bool ViewPlanner::plan(Vec3 target, double standoff, double cam_pitch, double lateral, const JointVec& seed, ViewPose& out,
                       std::string* why) const {
  auto fail = [&](const std::string& m) {
    if (why) *why = m;
    return false;
  };
  auto rotz = [](Vec3 v, double a) { return Vec3{v.x * std::cos(a) - v.y * std::sin(a), v.x * std::sin(a) + v.y * std::cos(a), v.z}; };

  const double az_t = std::atan2(target.y, target.x);
  const Vec3 aim = target + lateral * Vec3{-std::sin(az_t), std::cos(az_t), 0.0};
  const double r = std::hypot(aim.x, aim.y);

  // The camera is rigidly mounted on the tool and sits a fixed distance `l` to the side of the arm plane (the plane at
  // azimuth q1). The optical axis lies parallel to that plane, so it passes through the aim point only when the aim is
  // exactly `l` to the side of the plane: q1 = azimuth(aim) - asin(l / r). Everything else is a planar problem.
  const JointVec ref{0.0, 0.89, 0.35, -0.54};
  const Pose cam_ref = cameraPose(ref), tool_ref = kin_.toolPose(ref);
  const Vec3 o_ref = cam_ref.p - tool_ref.p;  // world offset camera - tool at q1 = 0
  const double l = o_ref.y;                   // sideways offset (the arm plane at q1 = 0 is the x-z plane)
  if (r < std::fabs(l) + 0.02) return fail("target too close to the base axis");
  const double phi = std::atan2(aim.y, aim.x) - std::asin(l / r);

  const Vec3 h{std::cos(phi), std::sin(phi), 0.0};
  const Vec3 axis = std::cos(cam_pitch) * h + Vec3{0, 0, std::sin(cam_pitch)};
  const Vec3 cam_goal = aim - standoff * axis;
  if (cam_goal.z < 0.05) return fail("camera would be below 5 cm");

  const double delta = cameraPitch(ref) - kin_.toolPitch(ref);  // pitch_cam = pitch_tool + delta
  const double tool_pitch = cam_pitch - delta;

  IKOptions opt;
  opt.finger_angle = p_.finger_angle;
  opt.joint_margin = p_.joint_margin;
  Vec3 off = rotz(o_ref, phi);  // first guess of camera - tool; refined from the IK result (depends on the tool orientation only)
  IKSolution sol;
  Pose cam;
  double err = 1e9;
  std::string last_why;
  for (int it = 0; it < 6; ++it) {
    const Vec3 tool_goal = cam_goal - off;
    bool have = false;
    for (const IKSolution& s : ik_.solveAll(tool_goal, tool_pitch, seed, opt)) {
      if (s.pos_err > 1e-3 || s.pitch_err > 0.02) continue;
      if (!s.collision_free) { last_why = "every IK solution makes links overlap: " + s.first_pair; continue; }
      sol = s;
      have = true;
      break;
    }
    if (!have) return fail(last_why.empty() ? "camera pose not reachable within the joint limits" : last_why);
    cam = cameraPose(sol.q);
    err = norm(cam_goal - cam.p);
    if (err < 5e-4) break;
    off = cam.p - kin_.toolPose(sol.q).p;
  }
  const double pitch_err = std::fabs(cameraPitch(sol.q) - cam_pitch) * 180.0 / kPi;
  if (err > p_.max_pos_error) {
    char b[96];
    std::snprintf(b, sizeof b, "camera position error %.1f mm", err * 1000);
    return fail(b);
  }
  if (pitch_err > p_.max_pitch_error_deg) return fail("camera pitch not reachable");
  out.q = sol.q;
  out.cam = cam;
  out.aim = aim;
  out.standoff = standoff;
  out.cam_pitch = cam_pitch;
  out.lateral = lateral;
  if (why) why->clear();
  return true;
}

}  // namespace arm
