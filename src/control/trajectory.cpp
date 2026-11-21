#include "autonomous_manipulator/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm {

namespace {
constexpr double kQuinticVmax = 15.0 / 8.0;                  // max of ds/du
const double kQuinticAmax = 10.0 / std::sqrt(3.0);           // max of |d2s/du2|
}  // namespace

void quintic(double u, double& s, double& ds, double& dds) {
  u = std::min(1.0, std::max(0.0, u));
  s = 10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u;
  ds = 30 * u * u - 60 * u * u * u + 30 * u * u * u * u;
  dds = 60 * u - 180 * u * u + 120 * u * u * u;
}

double minDuration(const JointVec& delta, const JointLimits& lim) {
  double t = 0.0;
  for (double d : delta) {
    const double a = std::fabs(d);
    t = std::max(t, kQuinticVmax * a / lim.vmax);
    t = std::max(t, std::sqrt(kQuinticAmax * a / lim.amax));
  }
  return t;
}

Trajectory::Trajectory(std::vector<JointVec> waypoints, std::vector<double> s, double duration)
    : wp_(std::move(waypoints)), s_(std::move(s)), duration_(duration) {
  if (wp_.size() < 2 || wp_.size() != s_.size()) throw std::invalid_argument("Trajectory: need >= 2 waypoints and one s each");
  for (size_t i = 1; i < s_.size(); ++i)
    if (!(s_[i] > s_[i - 1])) throw std::invalid_argument("Trajectory: s must be strictly increasing");
}

Trajectory Trajectory::pointToPoint(const JointVec& a, const JointVec& b, const JointLimits& lim, double duration) {
  JointVec d;
  for (int i = 0; i < kArmDof; ++i) d[i] = b[i] - a[i];
  const double tmin = minDuration(d, lim);
  double T = duration > 0.0 ? duration : tmin;
  T = std::max(T, 1e-3);  // zero-length moves still get a valid (tiny) duration
  return Trajectory({a, b}, {0.0, 1.0}, T);
}

Trajectory Trajectory::fromPath(const std::vector<JointVec>& path, const JointLimits& lim, double duration,
                                double min_duration_hint) {
  if (path.size() < 2) throw std::invalid_argument("fromPath: need >= 2 waypoints");
  // drop duplicates so that s is strictly increasing
  std::vector<JointVec> wp{path.front()};
  std::vector<double> len{0.0};
  for (size_t i = 1; i < path.size(); ++i) {
    double d = 0;
    for (int j = 0; j < kArmDof; ++j) d = std::max(d, std::fabs(path[i][j] - wp.back()[j]));
    if (d < 1e-9) continue;
    wp.push_back(path[i]);
    len.push_back(len.back() + d);  // max-norm path length
  }
  if (wp.size() < 2) wp.push_back(path.back()), len.push_back(1.0);
  std::vector<double> s(len.size());
  for (size_t i = 0; i < len.size(); ++i) s[i] = len[i] / len.back();

  // minimum duration: peak velocity/acceleration of q(t) = q(s(t)) scale with the largest per-unit-s slope
  double slope = 0.0;
  for (size_t i = 1; i < wp.size(); ++i)
    for (int j = 0; j < kArmDof; ++j) slope = std::max(slope, std::fabs(wp[i][j] - wp[i - 1][j]) / (s[i] - s[i - 1]));
  const double tv = kQuinticVmax * slope / lim.vmax;
  const double ta = std::sqrt(kQuinticAmax * slope / lim.amax);
  double T = duration > 0.0 ? duration : std::max({tv, ta, min_duration_hint, 1e-3});
  return Trajectory(std::move(wp), std::move(s), T);
}

void Trajectory::sample(double t, JointVec& q, JointVec& qd) const {
  const double u = duration_ > 0 ? std::min(1.0, std::max(0.0, t / duration_)) : 1.0;
  double s, ds, dds;
  quintic(u, s, ds, dds);
  const double sdot = duration_ > 0 ? ds / duration_ : 0.0;
  const size_t k = std::min<size_t>(std::upper_bound(s_.begin(), s_.end(), s) - s_.begin(), s_.size() - 1);
  const size_t i1 = std::max<size_t>(k, 1), i0 = i1 - 1;
  const double f = (s - s_[i0]) / (s_[i1] - s_[i0]);
  for (int j = 0; j < kArmDof; ++j) {
    const double slope = (wp_[i1][j] - wp_[i0][j]) / (s_[i1] - s_[i0]);
    q[j] = wp_[i0][j] + f * (wp_[i1][j] - wp_[i0][j]);
    qd[j] = slope * sdot;
  }
}

}  // namespace arm
