#include "autonomous_manipulator/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace arm {

namespace {
std::string fmt(const char* f, double a = 0, double b = 0, double c = 0) {
  char buf[256];
  std::snprintf(buf, sizeof buf, f, a, b, c);
  return buf;
}
double maxDiff(const JointVec& a, const JointVec& b) {
  double d = 0;
  for (int i = 0; i < kArmDof; ++i) d = std::max(d, std::fabs(a[i] - b[i]));
  return d;
}
std::string describe(const CollisionReport& r) {
  if (r.contacts.empty()) return "collision";
  return r.contacts.front().a + " <-> " + r.contacts.front().b + fmt(" (%.1f mm)", r.contacts.front().depth * 1000);
}
}  // namespace

bool MotionPlanner::validGoal(const JointVec& q, std::string& why) const {
  if (!kin_.withinLimits(q, cfg_.joint_margin)) {
    why = "goal violates the joint limits (margin " + fmt("%.3f rad)", cfg_.joint_margin);
    return false;
  }
  const CollisionReport r = chk_.check(q, cfg_.finger_angle);
  if (r.in_collision) {
    why = "goal pose makes links overlap: " + describe(r);
    return false;
  }
  return true;
}

bool MotionPlanner::pathFree(const JointVec& a, const JointVec& b, std::string& why) const {
  CollisionReport hit;
  if (chk_.pathCollisionFree(a, b, cfg_.finger_angle, cfg_.path_step_rad, &hit)) return true;
  why = "joint-space path collides: " + describe(hit);
  return false;
}

std::vector<JointVec> MotionPlanner::route(const JointVec& a, const JointVec& b, std::string& why) const {
  std::string w;
  if (pathFree(a, b, w)) return {a, b};
  std::string first = w;
  for (const JointVec& v : via_) {
    std::string w1, w2;
    if (validGoal(v, w1) && pathFree(a, v, w1) && pathFree(v, b, w2)) return {a, v, b};
  }
  why = first + (via_.empty() ? " (no via points configured)" : " (also through every via point)");
  return {};
}

PlanResult MotionPlanner::planJoints(const JointVec& q0, const JointVec& q_goal, double duration) const {
  PlanResult r;
  r.q_goal = q_goal;
  std::string why;
  if (!validGoal(q_goal, why)) {
    r.message = why;
    return r;
  }
  const std::vector<JointVec> path = route(q0, q_goal, why);
  if (path.empty()) {
    r.message = why;
    return r;
  }
  r.traj = path.size() == 2 ? Trajectory::pointToPoint(q0, q_goal, cfg_.limits, duration)
                            : Trajectory::fromPath(path, cfg_.limits, duration);
  r.ok = true;
  r.message = "ok";
  return r;
}

PlanResult MotionPlanner::planPose(const JointVec& q0, Vec3 p, double pitch, PoseMode mode, double prefer_pitch,
                                   bool linear, double duration) const {
  if (linear) {
    const double target_pitch = mode == PoseMode::Pose ? pitch : kin_.toolPitch(q0);
    return planLine(q0, p, mode == PoseMode::Pose ? target_pitch : prefer_pitch, duration);
  }
  PlanResult r;
  std::vector<double> pitches;
  if (mode == PoseMode::Pose) {
    pitches = {pitch};
  } else {  // prefer_pitch first, then widening alternatives (horizontal-ish before steep)
    pitches.push_back(prefer_pitch);
    for (int k = 1; k <= 12; ++k)
      for (int s : {1, -1}) pitches.push_back(prefer_pitch + s * k * 5.0 * kPi / 180.0);
  }
  std::string last = "target not reachable within the joint limits";
  for (double pt : pitches) {
    const std::vector<IKSolution> sols = ik_.solveAll(p, pt, q0, cfg_.ik);
    for (const IKSolution& s : sols) {
      if (s.pos_err > 2 * cfg_.ik.pos_tol * 10 || (mode == PoseMode::Pose && s.pitch_err > 0.02)) continue;
      if (!s.collision_free) {
        last = "every IK solution makes links overlap: " + s.first_pair;
        continue;
      }
      if (!kin_.withinLimits(s.q, cfg_.joint_margin)) {
        last = "IK solution too close to the joint limits";
        continue;
      }
      std::string why;
      const std::vector<JointVec> path = route(q0, s.q, why);
      if (path.empty()) {
        last = why;
        continue;
      }
      r.q_goal = s.q;
      r.pos_error = s.pos_err;
      r.pitch_error = s.pitch_err;
      r.traj = path.size() == 2 ? Trajectory::pointToPoint(q0, s.q, cfg_.limits, duration)
                                : Trajectory::fromPath(path, cfg_.limits, duration);
      r.ok = true;
      r.message = "ok";
      return r;
    }
  }
  r.message = last;
  return r;
}

PlanResult MotionPlanner::planLine(const JointVec& q0, Vec3 p1, double pitch1, double duration) const {
  PlanResult r;
  const Pose T0 = kin_.toolPose(q0);
  const Vec3 p0 = T0.p;
  const double pitch0 = Kinematics::pitchOf(T0);
  const double len = norm(p1 - p0);
  const int n = std::max(1, static_cast<int>(std::ceil(len / cfg_.line_step)));

  std::vector<JointVec> path{q0};
  JointVec prev = q0;
  IKOptions local = cfg_.ik;
  local.seed_grid = false;  // continue the current branch: refine from the previous sample only
  for (int i = 1; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    const Vec3 p = p0 + f * (p1 - p0);
    const double pt = pitch0 + f * (pitch1 - pitch0);
    r.failed_fraction = f;
    if (p.z < cfg_.min_tool_z) {
      r.message = fmt("line goes below min_tool_z at %.0f %% of the path", 100 * f);
      return r;
    }
    JointVec next{};
    bool found = false;
    std::string reason = "unreachable";
    const IKOptions* const opts[2] = {&local, &cfg_.ik};
    for (const IKOptions* o : opts) {  // 1) local continuation, 2) full multi-seed solve
      for (const IKSolution& s : ik_.solveAll(p, pt, prev, *o)) {
        if (s.pos_err > 1e-3 || s.pitch_err > 0.02) continue;
        if (!s.collision_free) {
          reason = "links overlap: " + s.first_pair;
          continue;
        }
        if (!kin_.withinLimits(s.q, cfg_.joint_margin)) {
          reason = "joint limit";
          continue;
        }
        if (maxDiff(s.q, prev) > cfg_.max_joint_jump) {
          reason = "joint jump (elbow flip / singularity)";
          continue;
        }
        next = s.q;
        found = true;
        break;
      }
      if (found) break;
    }
    if (!found) {
      r.message = fmt("line fails at %.0f %% of the path: ", 100 * f) + reason;
      return r;
    }
    path.push_back(next);
    prev = next;
  }
  r.q_goal = path.back();
  r.pos_error = norm(kin_.toolPose(r.q_goal).p - p1);
  r.pitch_error = pitch1 - kin_.toolPitch(r.q_goal);
  r.failed_fraction = 0.0;
  r.traj = Trajectory::fromPath(path, cfg_.limits, duration, len / cfg_.tool_speed);
  r.ok = true;
  r.message = "ok";
  return r;
}

}  // namespace arm
