#include "autonomous_manipulator/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace am {

JointVec JointTrajectory::sample(double t) const {
  if (q.empty()) return JointVec{};
  if (t <= 0) return q.front();
  double f = t / dt;
  size_t i = static_cast<size_t>(f);
  if (i + 1 >= q.size()) return q.back();
  double a = f - static_cast<double>(i);
  JointVec out;
  for (int k = 0; k < 4; ++k) out[k] = (1 - a) * q[i][k] + a * q[i + 1][k];
  return out;
}

double TrajectoryPlanner::quintic(double tau) {
  tau = clampd(tau, 0.0, 1.0);
  return tau * tau * tau * (10 + tau * (-15 + 6 * tau));
}

double TrajectoryPlanner::jointMoveDuration(const JointVec& q0, const JointVec& q1, const JointVec& vmax,
                                            double min_duration) {
  double d = min_duration;
  for (int k = 0; k < 4; ++k) d = std::max(d, 1.875 * std::fabs(q1[k] - q0[k]) / vmax[k]);  // 1.875 = peak of s'(tau)
  return d;
}

JointTrajectory TrajectoryPlanner::jointSpace(const JointVec& q0, const JointVec& q1, double duration, double dt) {
  JointTrajectory tr;
  tr.dt = dt;
  int n = std::max(2, static_cast<int>(std::ceil(duration / dt)) + 1);
  tr.q.resize(n);
  for (int i = 0; i < n; ++i) {
    double s = quintic(static_cast<double>(i) / (n - 1));
    for (int k = 0; k < 4; ++k) tr.q[i][k] = q0[k] + (q1[k] - q0[k]) * s;
  }
  return tr;
}

bool TrajectoryPlanner::cartesianLine(const Vec3& p0, const Vec3& p1, double pitch0, double pitch1, double duration,
                                      double dt, const IKOptions& ik, const JointVec* q_start, JointTrajectory& out,
                                      std::string& err, double start_tol, double max_jump) {
  out = JointTrajectory();
  out.dt = dt;
  int n = std::max(2, static_cast<int>(std::ceil(duration / dt)) + 1);
  out.q.reserve(n);
  for (int i = 0; i < n; ++i) {
    double s = quintic(static_cast<double>(i) / (n - 1));
    Vec3 p = p0 + s * (p1 - p0);
    double pitch = pitch0 + (pitch1 - pitch0) * s;
    IKResult r = solveIK(p, pitch, ik);
    if (!r.ok) {
      std::ostringstream os;
      os << "IK failed at sample " << i << "/" << n << " (" << p.x << "," << p.y << "," << p.z << "): " << r.reason;
      err = os.str();
      out = JointTrajectory();
      return false;
    }
    if (i == 0 && q_start) {
      for (int k = 0; k < 4; ++k)
        if (std::fabs(r.q[k] - (*q_start)[k]) > start_tol) {
          std::ostringstream os;
          os << "line start does not match the measured arm state (joint " << robot::kJointNames[k] << " differs by "
             << (r.q[k] - (*q_start)[k]) << " rad)";
          err = os.str();
          out = JointTrajectory();
          return false;
        }
    }
    if (i > 0) {
      for (int k = 0; k < 4; ++k)
        if (std::fabs(r.q[k] - out.q.back()[k]) > max_jump) {
          err = "IK branch jump / singularity along the Cartesian line";
          out = JointTrajectory();
          return false;
        }
    }
    out.q.push_back(r.q);
  }
  return true;
}

}  // namespace am
