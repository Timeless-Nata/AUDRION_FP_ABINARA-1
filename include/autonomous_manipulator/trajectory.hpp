// trajectory.hpp - time-parameterised joint trajectories (pure C++17, no ROS / MuJoCo).
//
// A Trajectory is a dense list of joint-space waypoints q_i at path parameters s_i in [0,1] (piecewise linear in
// s) driven by a quintic (minimum-jerk) time scaling s(t): zero velocity and acceleration at both ends.
// A plain point-to-point move is the 2-waypoint case, a Cartesian line is a dense list of IK solutions.
#pragma once
#include <vector>

#include "autonomous_manipulator/kinematics.hpp"

namespace arm {

struct JointLimits {
  double vmax = 1.0;  // [rad/s]  CONFIG values chosen for payload stability, NOT servo specifications
  double amax = 4.0;  // [rad/s^2]
};

// s(u), ds/du, d2s/du2 of the quintic blend, u in [0,1]
void quintic(double u, double& s, double& ds, double& dds);

// Shortest quintic duration for which |v| <= vmax and |a| <= amax on every joint for the given joint deltas.
double minDuration(const JointVec& delta, const JointLimits& lim);

class Trajectory {
 public:
  Trajectory() = default;
  // `waypoints` has >= 2 entries; the path is linear between neighbours with equal parameter spacing is NOT assumed:
  // `s` holds the cumulative path parameter of each waypoint (s.front()=0, s.back()=1, strictly increasing).
  Trajectory(std::vector<JointVec> waypoints, std::vector<double> s, double duration);

  // Point-to-point trajectory a -> b. duration <= 0 picks the minimum allowed by `lim`.
  static Trajectory pointToPoint(const JointVec& a, const JointVec& b, const JointLimits& lim, double duration = 0.0);
  // Dense path (e.g. Cartesian line). The parameter s of each waypoint is its normalised joint-space path length.
  // duration <= 0: the slowest of the joint limits and `min_duration_hint` is used.
  static Trajectory fromPath(const std::vector<JointVec>& path, const JointLimits& lim, double duration = 0.0,
                             double min_duration_hint = 0.0);

  bool empty() const { return wp_.size() < 2; }
  double duration() const { return duration_; }
  const JointVec& start() const { return wp_.front(); }
  const JointVec& goal() const { return wp_.back(); }
  const std::vector<JointVec>& waypoints() const { return wp_; }

  // Position and velocity at time t (clamped to [0, duration]).
  void sample(double t, JointVec& q, JointVec& qd) const;

 private:
  std::vector<JointVec> wp_;
  std::vector<double> s_;
  double duration_ = 0.0;
};

}  // namespace arm
