// arm_core_cli - command line front end of the arm core (no ROS, no MuJoCo).
//   arm_core_cli <robot_model.txt> fk      q1 q2 q3 q4
//   arm_core_cli <robot_model.txt> collide q1 q2 q3 q4 [finger_active]
//   arm_core_cli <robot_model.txt> ranges  q1 q2 q3 q4 [finger_active]   collision-free range of each joint
//   arm_core_cli <robot_model.txt> ik      x y z pitch_deg [q1 q2 q3 q4]  all solutions with collision status
//   arm_core_cli <robot_model.txt> ikpos   x y z [q1 q2 q3 q4]            position only (pitch 0, +-5, ..., +-30 deg)
// Angles in radians except the pitch argument (degrees). Positions in metres (world frame).
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

#include "autonomous_manipulator/inverse_kinematics.hpp"

using namespace arm;

static JointVec readQ(char** a) {
  return {std::atof(a[0]), std::atof(a[1]), std::atof(a[2]), std::atof(a[3])};
}

static void printReport(const CollisionReport& r) {
  if (!r.in_collision) {
    std::printf("  no overlap\n");
    return;
  }
  std::printf("  %zu overlapping pair(s), deepest %.2f mm:\n", r.contacts.size(), r.max_depth * 1000.0);
  for (const Contact& c : r.contacts) std::printf("    %-24s <-> %-24s %.2f mm\n", c.a.c_str(), c.b.c_str(), c.depth * 1000.0);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <robot_model.txt> fk|collide|ranges|ik|ikpos ...\n", argv[0]);
    return 2;
  }
  try {
    const RobotModel model = RobotModel::loadFromFile(argv[1]);
    const Kinematics kin(model);
    const CollisionChecker chk(model);
    const InverseKinematics ik(kin, &chk);
    const std::string cmd = argv[2];
    const int n = argc - 3;
    if (cmd == "fk" && n >= 4) {
      const JointVec q = readQ(argv + 3);
      const Pose T = kin.toolPose(q);
      std::printf("tool position  %.5f %.5f %.5f\ntool x axis    %.5f %.5f %.5f\npitch          %.3f deg\nwithin limits  %s\n", T.p.x,
                  T.p.y, T.p.z, T.R(0, 0), T.R(1, 0), T.R(2, 0), Kinematics::pitchOf(T) * 180 / kPi,
                  kin.withinLimits(q) ? "yes" : "NO");
    } else if (cmd == "collide" && n >= 4) {
      const double f = n >= 5 ? std::atof(argv[7]) : -0.7;
      printReport(chk.check(readQ(argv + 3), f));
    } else if (cmd == "ranges" && n >= 4) {
      const double f = n >= 5 ? std::atof(argv[7]) : -0.7;
      const JointVec q = readQ(argv + 3);
      for (int j = 0; j < kArmDof; ++j) {
        std::printf("joint_%d free in:", j + 1);
        for (const auto& iv : chk.freeIntervals(q, j, f)) std::printf(" [%+.2f, %+.2f]", iv.lo, iv.hi);
        std::printf("   (limits %+.3f .. %+.3f)\n", kin.lower()[j], kin.upper()[j]);
      }
    } else if ((cmd == "ik" && n >= 4) || (cmd == "ikpos" && n >= 3)) {
      const Vec3 p{std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5])};
      const int si = (cmd == "ik") ? 7 : 6;
      JointVec seed{0, 0.8901, 0.4538, -0.4363};
      if (argc >= si + 4) seed = readQ(argv + si);
      IKSolution best;
      if (cmd == "ik") {
        const double pitch = std::atof(argv[6]) * kPi / 180.0;
        const auto all = ik.solveAll(p, pitch, seed);
        std::printf("%zu solution(s):\n", all.size());
        for (const auto& s : all)
          std::printf("  q = [%+.4f %+.4f %+.4f %+.4f]  err %.3f mm %.3f deg  %s%s%s\n", s.q[0], s.q[1], s.q[2], s.q[3],
                      s.pos_err * 1000, s.pitch_err * 180 / kPi, s.collision_free ? "collision-free" : "OVERLAP: ",
                      s.collision_free ? "" : s.first_pair.c_str(), s.collision_free ? "" : "");
      } else {
        std::vector<double> pitches = {0};
        for (int d = 5; d <= 30; d += 5) {
          pitches.push_back(d * kPi / 180);
          pitches.push_back(-d * kPi / 180);
        }
        double used = 0;
        const IKStatus st = ik.solvePosition(p, pitches, seed, IKOptions(), best, &used);
        const char* names[] = {"OK", "NO SOLUTION (unreachable)", "ALL SOLUTIONS OVERLAP"};
        std::printf("status: %s\n", names[static_cast<int>(st)]);
        if (st != IKStatus::NoSolution)
          std::printf("pitch %.1f deg  q = [%+.4f %+.4f %+.4f %+.4f]  err %.3f mm\n", used * 180 / kPi, best.q[0], best.q[1],
                      best.q[2], best.q[3], best.pos_err * 1000);
      }
    } else {
      std::fprintf(stderr, "bad arguments\n");
      return 2;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
