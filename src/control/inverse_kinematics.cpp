#include "autonomous_manipulator/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace arm {

bool InverseKinematics::refine(JointVec& q, Vec3 p, double pitch, const IKOptions& o) const {
  const double w = o.pitch_weight;
  for (int it = 0; it <= o.max_iter; ++it) {
    const Pose T = kin_.toolPose(q);
    const Vec3 ep = p - T.p;
    const double eph = pitch - Kinematics::pitchOf(T);
    if (norm(ep) < o.pos_tol && std::fabs(eph) < o.pitch_tol) return true;
    if (it == o.max_iter) break;

    const Jacobian J = kin_.toolJacobian(q);
    const JointVec g = kin_.pitchGradient(q);
    double Jm[4][kArmDof];
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < kArmDof; ++c) Jm[r][c] = J.J[r][c];
    for (int c = 0; c < kArmDof; ++c) Jm[3][c] = w * g[c];
    const std::vector<double> e = {ep.x, ep.y, ep.z, w * eph};

    std::vector<double> M(16), y;
    for (int r = 0; r < 4; ++r)
      for (int s = 0; s < 4; ++s) {
        double a = 0;
        for (int c = 0; c < kArmDof; ++c) a += Jm[r][c] * Jm[s][c];
        M[4 * r + s] = a + (r == s ? o.damping * o.damping : 0.0);
      }
    if (!solveLinear(M, e, 4, y)) return false;
    JointVec dq{};
    double mx = 0;
    for (int c = 0; c < kArmDof; ++c) {
      for (int r = 0; r < 4; ++r) dq[c] += Jm[r][c] * y[r];
      mx = std::fmax(mx, std::fabs(dq[c]));
    }
    const double s = mx > o.max_step ? o.max_step / mx : 1.0;
    for (int c = 0; c < kArmDof; ++c) q[c] += s * dq[c];
    q = kin_.clampToLimits(q, o.joint_margin);
  }
  return false;
}

std::vector<IKSolution> InverseKinematics::solveAll(Vec3 p, double pitch, const JointVec& seed, const IKOptions& o) const {
  std::vector<JointVec> seeds;
  seeds.push_back(kin_.clampToLimits(seed, o.joint_margin));
  if (o.seed_grid) {
    const double yaw = clampd(std::atan2(p.y, p.x), kin_.lower()[0] + o.joint_margin, kin_.upper()[0] - o.joint_margin);
    const double q2s[] = {-1.0, -0.3, 0.4, 1.1};
    const double q3s[] = {-1.2, -0.4, 0.4, 1.2};
    const double q4s[] = {-1.0, 0.0, 1.0};
    for (double a : q2s)
      for (double b : q3s)
        for (double c : q4s) seeds.push_back(kin_.clampToLimits(JointVec{yaw, a, b, c}, o.joint_margin));
  }

  std::vector<IKSolution> sols;
  for (JointVec q : seeds) {
    if (!refine(q, p, pitch, o)) continue;
    bool dup = false;
    for (const IKSolution& s : sols) {
      double d = 0;
      for (int i = 0; i < kArmDof; ++i) d = std::fmax(d, std::fabs(s.q[i] - q[i]));
      if (d < 1e-3) dup = true;
    }
    if (dup) continue;
    IKSolution s;
    s.q = q;
    const Pose T = kin_.toolPose(q);
    s.pos_err = norm(p - T.p);
    s.pitch_err = std::fabs(pitch - Kinematics::pitchOf(T));
    if (checker_) {
      const CollisionReport r = checker_->check(q, o.finger_angle);
      s.collision_free = !r.in_collision;
      if (r.in_collision) {
        s.depth = r.max_depth;
        s.first_pair = r.contacts.front().a + " <-> " + r.contacts.front().b;
      }
    }
    sols.push_back(s);
  }
  auto dist = [&](const IKSolution& s) {
    double d = 0;
    for (int i = 0; i < kArmDof; ++i) d += (s.q[i] - seed[i]) * (s.q[i] - seed[i]);
    return d;
  };
  std::sort(sols.begin(), sols.end(), [&](const IKSolution& a, const IKSolution& b) {
    if (a.collision_free != b.collision_free) return a.collision_free;
    return dist(a) < dist(b);
  });
  return sols;
}

IKStatus InverseKinematics::solve(Vec3 p, double pitch, const JointVec& seed, const IKOptions& o, IKSolution& best) const {
  const std::vector<IKSolution> s = solveAll(p, pitch, seed, o);
  if (s.empty()) return IKStatus::NoSolution;
  best = s.front();
  return best.collision_free ? IKStatus::Ok : IKStatus::AllInCollision;
}

IKStatus InverseKinematics::solvePosition(Vec3 p, const std::vector<double>& pitches, const JointVec& seed, const IKOptions& o,
                                          IKSolution& best, double* pitch_used) const {
  IKStatus worst = IKStatus::NoSolution;
  bool haveColliding = false;
  IKSolution colliding;
  double collidingPitch = 0;
  for (double pc : pitches) {
    IKSolution s;
    const IKStatus st = solve(p, pc, seed, o, s);
    if (st == IKStatus::Ok) {
      best = s;
      if (pitch_used) *pitch_used = pc;
      return st;
    }
    if (st == IKStatus::AllInCollision && !haveColliding) {
      haveColliding = true;
      colliding = s;
      collidingPitch = pc;
      worst = st;
    }
  }
  if (haveColliding) {
    best = colliding;
    if (pitch_used) *pitch_used = collidingPitch;
  }
  return worst;
}

}  // namespace arm
