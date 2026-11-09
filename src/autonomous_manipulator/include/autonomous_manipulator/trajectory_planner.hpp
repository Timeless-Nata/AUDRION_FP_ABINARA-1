// Smooth trajectory generation (quintic time scaling) in joint space and along Cartesian lines.
#pragma once
#include <string>
#include <vector>

#include "autonomous_manipulator/robot_model.hpp"

namespace am {

struct JointTrajectory {
  double dt = 0.01;
  std::vector<JointVec> q;
  bool empty() const { return q.empty(); }
  double duration() const { return q.empty() ? 0.0 : (q.size() - 1) * dt; }
  JointVec sample(double t) const;  // linear interpolation between stored samples, clamped at the ends
};

class TrajectoryPlanner {
 public:
  // s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5 (zero velocity and acceleration at both ends)
  static double quintic(double tau);
  static double jointMoveDuration(const JointVec& q0, const JointVec& q1, const JointVec& vmax, double min_duration);
  static JointTrajectory jointSpace(const JointVec& q0, const JointVec& q1, double duration, double dt);
  // Straight Cartesian line p0->p1; the tool pitch is blended pitch0->pitch1 with the same time scaling.
  // q_start (optional) is the measured arm state: the first IK sample must be within start_tol of it.
  static bool cartesianLine(const Vec3& p0, const Vec3& p1, double pitch0, double pitch1, double duration, double dt,
                            const IKOptions& ik, const JointVec* q_start, JointTrajectory& out, std::string& err,
                            double start_tol = 0.05, double max_jump = 0.12);
};

}  // namespace am
