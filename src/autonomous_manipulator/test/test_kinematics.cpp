// Stand-alone test (no ROS / MuJoCo needed):  FK->IK round trip, limits, trajectory smoothness.
#include <cstdio>
#include <cstdlib>
#include "autonomous_manipulator/robot_model.hpp"
#include "autonomous_manipulator/trajectory_planner.hpp"
using namespace am;
static double urand(double a, double b) { return a + (b - a) * (std::rand() / (double)RAND_MAX); }
int main() {
  std::srand(7);
  int ok = 0, tried = 0;
  double worst_pos = 0, worst_or = 0;
  for (int i = 0; i < 20000; ++i) {
    JointVec q = {urand(-1.4, 1.4), urand(-1.4, 1.4), urand(0.05, 2.9), 0};
    q[3] = -(q[1] + q[2]) + urand(-0.01, 0.01);  // level hand
    if (!withinLimits(q, 0.03)) continue;
    ++tried;
    Pose p = forwardKinematics(q);
    IKResult r = solveIK(p.p, 0.0);
    if (!r.ok) { std::printf("IK failed for feasible pose: %s\n", r.reason.c_str()); return 1; }
    Pose p2 = forwardKinematics(r.q);
    worst_pos = std::max(worst_pos, norm(p2.p - p.p));
    worst_or = std::max(worst_or, r.orient_err);
    ++ok;
  }
  std::printf("round trips ok: %d/%d, worst pos err %.2e m, worst orient err %.4f rad\n", ok, tried, worst_pos, worst_or);
  // unreachable / limit handling
  IKResult far = solveIK({1.0, 0, 0.1}, 0.0);
  std::printf("far target -> ok=%d (%s)\n", far.ok, far.reason.c_str());
  IKResult near = solveIK({0.001, 0, 0.1}, 0.0);
  std::printf("axis target -> ok=%d (%s)\n", near.ok, near.reason.c_str());
  // line smoothness
  JointTrajectory tr; std::string err;
  bool lok = TrajectoryPlanner::cartesianLine({0.26, 0.0, 0.10}, {0.30, 0.05, 0.05}, 0.0, 0.0, 2.0, 0.01, IKOptions(), nullptr, tr, err);
  std::printf("line ok=%d n=%zu dur=%.2f %s\n", lok, tr.q.size(), tr.duration(), err.c_str());
  // dump reference values for cross-check against independent MJCF-derived FK
  JointVec q = {0.3, 0.2, 1.0, -0.4};
  Pose e = forwardKinematics(q), c = cameraPose(q), u = ultrasonicPose(q);
  std::printf("REF ee %.4f %.4f %.4f | cam %.4f %.4f %.4f | us %.4f %.4f %.4f\n", e.p.x, e.p.y, e.p.z, c.p.x, c.p.y, c.p.z, u.p.x, u.p.y, u.p.z);
  Vec3 cz = c.R.col(2), uz = ultrasonicDirection(q);
  std::printf("REF cam_z %.4f %.4f %.4f | us_dir %.4f %.4f %.4f\n", cz.x, cz.y, cz.z, uz.x, uz.y, uz.z);
  return 0;
}
