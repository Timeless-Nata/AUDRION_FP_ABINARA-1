// test_core.cpp - unit tests of the arm core (no ROS, no MuJoCo, no test framework needed).
//   arm_core_test <config/robot_model.txt> <test/reference_vectors.txt>
// Exit code 0 = all checks passed.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>

#include "autonomous_manipulator/inverse_kinematics.hpp"

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

static JointVec randomQ(std::mt19937& g, const Kinematics& k, double margin) {
  JointVec q;
  for (int i = 0; i < kArmDof; ++i) q[i] = std::uniform_real_distribution<double>(k.lower()[i] + margin, k.upper()[i] - margin)(g);
  return q;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s robot_model.txt reference_vectors.txt\n", argv[0]);
    return 2;
  }
  const RobotModel model = RobotModel::loadFromFile(argv[1]);
  const Kinematics kin(model);
  CollisionOptions copt;
  const CollisionChecker chk(model, copt);
  std::mt19937 rng(7);

  std::printf("[model] bodies=%zu hulls=%zu frames=%zu collision pairs=%zu\n", model.bodies.size(), model.hulls.size(),
              model.frames.size(), chk.pairCount());
  CHECK(model.bodies.size() == 7 && model.hulls.size() == 48, "unexpected model size");
  CHECK(model.bodies[model.bodyIndex("gripper_pasif")].coef == -1.0, "finger_passive must be coupled with -1");

  // ---- reference vectors (independent numpy implementation reading the MJCF directly)
  std::ifstream rf(argv[2]);
  if (!rf) { std::fprintf(stderr, "cannot open %s\n", argv[2]); return 2; }
  std::string line;
  int nfk = 0, ncol = 0;
  double worstPos = 0, worstRot = 0, worstDepth = 0;
  while (std::getline(rf, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    std::string kind;
    is >> kind;
    if (kind == "fk") {
      JointVec q;
      Vec3 p;
      double R[9];
      is >> q[0] >> q[1] >> q[2] >> q[3] >> p.x >> p.y >> p.z;
      for (double& r : R) is >> r;
      const Pose T = kin.toolPose(q);
      worstPos = std::fmax(worstPos, norm(T.p - p));
      for (int i = 0; i < 9; ++i) worstRot = std::fmax(worstRot, std::fabs(T.R.m[i] - R[i]));
      ++nfk;
    } else if (kind == "col") {
      JointVec q;
      double fa;
      int n;
      is >> q[0] >> q[1] >> q[2] >> q[3] >> fa >> n;
      std::map<std::string, double> ref;
      for (int i = 0; i < n; ++i) {
        std::string tok;
        is >> tok;
        const size_t eq = tok.rfind('=');
        ref[tok.substr(0, eq)] = std::atof(tok.substr(eq + 1).c_str());
      }
      const CollisionReport rep = chk.check(q, fa);
      std::map<std::string, double> got;
      for (const Contact& c : rep.contacts) {
        std::string a = c.a, b = c.b;
        if (b < a) std::swap(a, b);
        got[a + "|" + b] = c.depth;
      }
      std::set<std::string> kr, kg;
      for (auto& kv : ref) kr.insert(kv.first);
      for (auto& kv : got) kg.insert(kv.first);
      CHECK(kr == kg, "collision sample %d: pair sets differ (ref %zu, C++ %zu)", ncol, kr.size(), kg.size());
      for (auto& kv : ref)
        if (got.count(kv.first)) worstDepth = std::fmax(worstDepth, std::fabs(got[kv.first] - kv.second));
      ++ncol;
    }
  }
  std::printf("[fk]        %d samples: worst position error %.2e m, worst rotation error %.2e\n", nfk, worstPos, worstRot);
  CHECK(nfk >= 20 && worstPos < 1e-6 && worstRot < 1e-6, "FK disagrees with the numpy reference");
  std::printf("[collision] %d samples: pair sets identical, worst depth difference %.2e m\n", ncol, worstDepth);
  CHECK(ncol >= 40 && worstDepth < 1e-5, "collision depth disagrees with the numpy reference");

  // ---- Jacobian and pitch gradient vs finite differences
  double worstJ = 0, worstG = 0;
  for (int t = 0; t < 50; ++t) {
    const JointVec q = randomQ(rng, kin, 0.05);
    const Jacobian J = kin.toolJacobian(q);
    const JointVec g = kin.pitchGradient(q);
    const Pose T0 = kin.toolPose(q);
    for (int i = 0; i < kArmDof; ++i) {
      JointVec qp = q, qm = q;
      const double h = 1e-6;
      qp[i] += h; qm[i] -= h;
      const Pose Tp = kin.toolPose(qp), Tm = kin.toolPose(qm);
      const Vec3 fd = (1.0 / (2 * h)) * (Tp.p - Tm.p);
      worstJ = std::fmax(worstJ, norm(fd - Vec3{J.J[0][i], J.J[1][i], J.J[2][i]}));
      // angular: dR R^T gives the skew matrix of omega
      const Mat3 dR = Mat3{};  (void)dR;
      const Vec3 wFd = [&] {
        Mat3 A;
        for (int k = 0; k < 9; ++k) A.m[k] = (Tp.R.m[k] - Tm.R.m[k]) / (2 * h);
        const Mat3 S = A * transpose(T0.R);
        return Vec3{S(2, 1), S(0, 2), S(1, 0)};
      }();
      worstJ = std::fmax(worstJ, norm(wFd - Vec3{J.J[3][i], J.J[4][i], J.J[5][i]}));
      worstG = std::fmax(worstG, std::fabs((Kinematics::pitchOf(Tp) - Kinematics::pitchOf(Tm)) / (2 * h) - g[i]));
    }
  }
  std::printf("[jacobian]  vs finite differences: worst %.2e (6x4), pitch gradient worst %.2e\n", worstJ, worstG);
  CHECK(worstJ < 1e-6 && worstG < 1e-6, "Jacobian mismatch");

  // ---- IK round trip, joint limits only
  {
    const InverseKinematics ik(kin, nullptr);
    IKOptions o;
    int ok = 0, total = 0;
    double worstP = 0, worstA = 0;
    int skipped = 0;
    while (total < 300) {
      const JointVec qt = randomQ(rng, kin, 0.1);
      const Pose T = kin.toolPose(qt);
      // IK scope: the arm in FRONT of the base axis (tool x > 0.05 m, radius > 0.08 m). Folded-back poses behind the
      // base axis and poses on the axis (yaw undefined) are outside the picking workspace.
      if (T.p.x < 0.05 || std::hypot(T.p.x, T.p.y) < 0.08) { ++skipped; continue; }
      const double pitch = Kinematics::pitchOf(T);
      JointVec seed = qt;
      for (double& s : seed) s += std::uniform_real_distribution<double>(-0.4, 0.4)(rng);
      seed = kin.clampToLimits(seed);
      IKSolution s;
      ++total;
      if (ik.solve(T.p, pitch, seed, o, s) == IKStatus::Ok) {
        ++ok;
        const Pose Ts = kin.toolPose(s.q);
        worstP = std::fmax(worstP, norm(Ts.p - T.p));
        worstA = std::fmax(worstA, std::fabs(Kinematics::pitchOf(Ts) - pitch));
        CHECK(kin.withinLimits(s.q), "IK solution outside limits");
      }
    }
    std::printf("[ik]        round trip in the front workspace: %d/%d solved (%d random poses behind/on the base axis skipped), worst position error %.2e m, worst pitch error %.2e rad\n", ok, total, skipped, worstP, worstA);
    CHECK(ok == total && worstP < 2e-4 && worstA < 2e-3, "IK round trip failed");
  }

  // ---- collision-aware IK
  {
    const InverseKinematics ik(kin, &chk);
    IKOptions o;
    // a pose whose "natural" joint vector overlaps (joint_3 pushed into the base/forearm region)
    const JointVec qbad = {0, 0.8901, 0.7538, -0.4363};
    CHECK(chk.check(qbad, -0.7).in_collision, "qbad should overlap");
    const Pose Tb = kin.toolPose(qbad);
    IKSolution s;
    const IKStatus st = ik.solve(Tb.p, Kinematics::pitchOf(Tb), qbad, o, s);
    if (st == IKStatus::Ok) {
      CHECK(!chk.check(s.q, -0.7).in_collision, "IK returned a colliding solution as Ok");
      std::printf("[ik+coll]   pose of an overlapping configuration: IK found a different collision-free posture q=[%+.3f %+.3f %+.3f %+.3f]\n",
                  s.q[0], s.q[1], s.q[2], s.q[3]);
    } else {
      std::printf("[ik+coll]   pose of an overlapping configuration: status %d (no collision-free posture) - reported, not returned\n", static_cast<int>(st));
      CHECK(st == IKStatus::AllInCollision || st == IKStatus::NoSolution, "unexpected status");
    }
  }

  // ---- the six dumbbells of scene.xml (positions at the time of writing): horizontal side grasp, plus a pre-grasp 6 cm further out
  {
    const InverseKinematics ik(kin, &chk);
    const double B[6][3] = {{0.181809, -0.224054, 0.0305}, {0.241995, -0.157139, 0.0305}, {0.278638, -0.0749363, 0.0305},
                            {0.288171, 0.0145574, 0.0305}, {0.269668, 0.102635, 0.0305},  {0.224928, 0.180727, 0.0305}};
    const char* names[] = {"OK", "NO SOLUTION", "ALL OVERLAP"};
    std::printf("[blocks]    tool at the bar centre, pitch 0 (horizontal), open gripper:\n");
    for (int b = 0; b < 6; ++b) {
      const double r = std::hypot(B[b][0], B[b][1]);
      for (int stage = 0; stage < 2; ++stage) {
        const double rr = r - (stage == 0 ? 0.06 : 0.0);   // stage 0 = pre-grasp (6 cm closer to the base)
        const Vec3 p{B[b][0] * rr / r, B[b][1] * rr / r, B[b][2]};
        IKSolution s;
        const IKStatus st = ik.solve(p, 0.0, JointVec{std::atan2(p.y, p.x), 0.8901, 0.4538, -0.4363}, IKOptions(), s);
        std::printf("   block %d %-9s r=%.3f : %-12s", b + 1, stage == 0 ? "pre-grasp" : "grasp", rr, names[static_cast<int>(st)]);
        if (st != IKStatus::NoSolution) {
          std::printf(" q=[%+.3f %+.3f %+.3f %+.3f]", s.q[0], s.q[1], s.q[2], s.q[3]);
          if (!s.collision_free) std::printf("  %s %.1f mm", s.first_pair.c_str(), s.depth * 1000);
        }
        std::printf("\n");
      }
    }
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
