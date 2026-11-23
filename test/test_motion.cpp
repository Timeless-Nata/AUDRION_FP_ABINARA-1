// test_motion.cpp - unit tests of trajectory generation, planning and execution (no ROS, no MuJoCo).
//   arm_motion_test <config/robot_model.txt>
// The "robot" is a software servo (first-order lag with a constant sag offset), only to exercise the executor.
#include <cmath>
#include <cstdio>

#include "autonomous_manipulator/executor.hpp"
#include "autonomous_manipulator/motion_planner.hpp"

using namespace arm;
static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    ++g_checks;                                           \
    if (!(cond)) {                                        \
      ++g_fail;                                           \
      std::printf("  FAIL %s:%d  ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

struct SimServo : RobotInterface, Clock {
  JointVec q{}, cmd{}, sag{};
  double t = 0, tau = 0.05, block_at = 1e9;  // joint_1 stops following when q[0] >= block_at (simulated obstruction)
  bool readJoints(JointVec& out) override { out = q; return true; }
  void sendJoints(const JointVec& c) override { cmd = c; }
  double now() override { return t; }
  void sleep(double s) override {
    const int n = std::max(1, static_cast<int>(s / 0.002));
    const double h = s / n, k = 1 - std::exp(-h / tau);
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < kArmDof; ++j) {
        const double target = cmd[j] + sag[j];
        if (j == 0 && q[0] >= block_at && target > q[0]) continue;
        q[j] += (target - q[j]) * k;
      }
    t += s;
  }
};

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s robot_model.txt\n", argv[0]); return 2; }
  const RobotModel model = RobotModel::loadFromFile(argv[1]);
  const Kinematics kin(model);
  const CollisionChecker chk(model, CollisionOptions::withMargin(0.002));
  const InverseKinematics ik(kin, &chk);
  PlannerConfig cfg;
  const MotionPlanner mp(kin, chk, ik, cfg);
  const JointVec home{0.0, 0.8901, 0.35, -0.540};

  std::printf("[trajectory]\n");
  {
    const JointVec b{0.5, 1.1, 0.2, -0.3};
    const Trajectory tr = Trajectory::pointToPoint(home, b, cfg.limits);
    JointVec q, qd, q2, qd2;
    tr.sample(0, q, qd);
    CHECK(std::fabs(q[0] - home[0]) < 1e-12 && std::fabs(qd[0]) < 1e-12, "start");
    tr.sample(tr.duration(), q, qd);
    CHECK(std::fabs(q[0] - b[0]) < 1e-12 && std::fabs(qd[1]) < 1e-9, "end");
    double vpeak = 0, apeak = 0;
    for (double t = 0; t < tr.duration(); t += 1e-3) {
      tr.sample(t, q, qd);
      tr.sample(t + 1e-3, q2, qd2);
      for (int j = 0; j < kArmDof; ++j) {
        vpeak = std::max(vpeak, std::fabs(qd[j]));
        apeak = std::max(apeak, std::fabs((qd2[j] - qd[j]) / 1e-3));
        CHECK(std::fabs(q2[j] - q[j]) < 0.01, "jump in q");
      }
    }
    std::printf("   T = %.3f s  peak v %.3f rad/s (limit %.1f)  peak a %.3f rad/s^2 (limit %.1f)\n", tr.duration(), vpeak,
                cfg.limits.vmax, apeak, cfg.limits.amax);
    CHECK(vpeak <= cfg.limits.vmax * 1.01, "velocity limit");
    CHECK(apeak <= cfg.limits.amax * 1.02, "acceleration limit");
    const Trajectory z = Trajectory::pointToPoint(home, home, cfg.limits);
    CHECK(z.duration() > 0, "zero-length move has a duration");
  }

  std::printf("[planner: joint moves]\n");
  {
    PlanResult r = mp.planJoints(home, {0.5, 1.0, 0.3, -0.4});
    CHECK(r.ok, "free joint move: %s", r.message.c_str());
    r = mp.planJoints(home, {0.0, 0.8901, 0.7, -0.540});  // q3 = 0.7 overlaps the base box
    CHECK(!r.ok && r.message.find("overlap") != std::string::npos, "colliding goal must be rejected: %s", r.message.c_str());
    std::printf("   colliding goal  -> %s\n", r.message.c_str());
    r = mp.planJoints(home, {0.0, 0.8901, 1.7, -0.540});
    CHECK(!r.ok, "goal beyond the joint limits must be rejected");
  }

  std::printf("[planner: tool poses and lines, six dumbbells]\n");
  const double B[6][3] = {{0.181809, -0.224054, 0.0305}, {0.241995, -0.157139, 0.0305}, {0.278638, -0.0749363, 0.0305},
                          {0.288171, 0.0145574, 0.0305}, {0.269668, 0.102635, 0.0305},  {0.224928, 0.180727, 0.0305}};
  for (int b = 0; b < 6; ++b) {
    const double r = std::hypot(B[b][0], B[b][1]);
    const Vec3 pre{B[b][0] * (r - 0.06) / r, B[b][1] * (r - 0.06) / r, B[b][2]};
    const Vec3 grasp{B[b][0], B[b][1], B[b][2]};
    PlanResult a = mp.planPose(home, pre, 0.0, PoseMode::Pose, 0.0, false);
    CHECK(a.ok, "block %d pre-grasp: %s", b + 1, a.message.c_str());
    if (!a.ok) { std::printf("   block %d pre-grasp FAILED: %s\n", b + 1, a.message.c_str()); continue; }
    CHECK(a.pos_error < 5e-4, "block %d pre-grasp IK error %.3f mm", b + 1, a.pos_error * 1000);
    PlanResult l = mp.planLine(a.q_goal, grasp, 0.0);
    CHECK(l.ok, "block %d approach line: %s", b + 1, l.message.c_str());
    if (l.ok) {
      CHECK(l.pos_error < 5e-4, "block %d line end error %.3f mm", b + 1, l.pos_error * 1000);
      // every sample of the line stays within the straight line (1 mm) and keeps the tool horizontal
      double dev = 0, pdev = 0;
      const Vec3 d = normalized(grasp - pre);
      for (double t = 0; t <= l.traj.duration(); t += l.traj.duration() / 40) {
        JointVec q, qd;
        l.traj.sample(t, q, qd);
        const Pose T = kin.toolPose(q);
        const Vec3 v = T.p - pre;
        const double along = dot(v, d);
        dev = std::max(dev, norm(v - along * d));
        pdev = std::max(pdev, std::fabs(Kinematics::pitchOf(T)));
        CHECK(!chk.check(q, cfg.finger_angle).in_collision, "block %d line sample collides", b + 1);
      }
      CHECK(dev < 1e-3, "block %d line deviates %.2f mm", b + 1, dev * 1000);
      std::printf("   block %d: pre-grasp ok (T=%.2fs), 6 cm line ok (T=%.2fs, max deviation %.2f mm, max |pitch| %.2f deg)\n", b + 1,
                  a.traj.duration(), l.traj.duration(), dev * 1000, pdev * 180 / kPi);
    } else {
      std::printf("   block %d approach line FAILED: %s\n", b + 1, l.message.c_str());
    }
  }
  {
    PlanResult r = mp.planPose(home, {0.30, 0.0, 0.40}, 0.0, PoseMode::Pose, 0.0, false);  // far outside the reach
    CHECK(!r.ok, "unreachable pose must fail");
    std::printf("   unreachable pose -> %s\n", r.message.c_str());
    r = mp.planPose(home, {0.2, 0.0, -0.05}, 0.0, PoseMode::Position, 0.0, true);  // below the floor
    CHECK(!r.ok, "line below the floor must fail");
    std::printf("   line below the floor -> %s\n", r.message.c_str());
    r = mp.planPose(home, {0.27, 0.10, 0.0305}, 0.0, PoseMode::Position, -0.3, false);
    CHECK(r.ok, "position-only mode: %s", r.message.c_str());
  }

  std::printf("[executor on a software servo]\n");
  {
    SimServo s;
    s.q = home;
    s.cmd = home;
    for (int j = 0; j < kArmDof; ++j) s.sag[j] = -0.03;     // constant offset like gravity sag
    const JointVec goal{0.4, 1.0, 0.25, -0.45};
    const PlanResult r = mp.planJoints(home, goal);
    ExecOptions eo;
    ExecResult e = executeTrajectory(r.traj, s, s, eo);
    CHECK(!e.ok, "without trim the 0.03 rad sag must be reported as 'did not settle' (err %.4f)", e.max_error);
    std::printf("   no trim:   %s\n", e.message.c_str());
    s.q = home; s.cmd = home;
    eo.integral_gain = 3.0;
    e = executeTrajectory(r.traj, s, s, eo);
    CHECK(e.ok && e.max_error < 0.004, "with trim: %s (err %.4f)", e.message.c_str(), e.max_error);
    std::printf("   with trim: %s, final error %.4f rad\n", e.message.c_str(), e.max_error);

    s.q = home; s.cmd = home; s.block_at = 0.1;              // joint_1 gets stuck at 0.1 rad
    e = executeTrajectory(r.traj, s, s, ExecOptions());
    CHECK(!e.ok, "an obstructed joint must abort the move");
    std::printf("   blocked:   %s\n", e.message.c_str());

    s.q = home; s.cmd = home; s.block_at = 1e9;
    int n = 0;
    e = executeTrajectory(r.traj, s, s, ExecOptions(), [&] { return ++n > 20; });
    CHECK(!e.ok && e.message == "cancelled", "cancel");
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
