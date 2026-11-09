// Offline reachability audit of every pose the mission will command (uses the same formulas as mission_controller.cpp).
#include <cstdio>
#include <vector>
#include "autonomous_manipulator/geometry_utils.hpp"
#include "autonomous_manipulator/robot_model.hpp"
using namespace am;
static IKOptions ik;
static bool scanPose(const Vec3& P, double dist, double pitch_deg, double ustar, const CameraIntrinsics& K, IKResult& out) {
  const double phi = pitch_deg * M_PI / 180.0, al = std::atan((ustar - K.cx) / K.fx);
  double h = std::atan2(P.y, P.x) + al;
  for (int it = 0; it < 8; ++it) {
    Vec3 f{std::cos(phi) * std::cos(h), std::cos(phi) * std::sin(h), -std::sin(phi)};
    Vec3 rgt{std::sin(h), -std::cos(h), 0}, yh{-std::sin(h), std::cos(h), 0};
    Vec3 d = normalized(f + std::tan(al) * rgt);
    Vec3 C = P - dist * d;
    Vec3 ee = C + (robot::kEeX - robot::kCamX) * f - robot::kCamY * yh;
    out = solveIK(ee, phi, ik);
    if (!out.ok) return false;
    if (std::fabs(out.q[0] - h) < 1e-6) return true;
    h = out.q[0];
  }
  return out.ok;
}
int main() {
  ik.pitch_tol = 0.01;
  const double objs[6][2] = {{0.15, -0.25981}, {0.21034, -0.15282}, {0.29344, -0.06237}, {0.25432, 0.05406}, {0.24271, 0.17634}, {0.13, 0.22517}};
  const char* nm[6] = {"1 orange", "2 green", "3 orange", "4 white", "5 green", "6 orange"};
  CameraIntrinsics K = CameraIntrinsics::fromFovy(20, 640, 480);
  int bad = 0;
  // pick poses
  for (int i = 0; i < 6; ++i) {
    Vec3 T{objs[i][0], objs[i][1], 0.035};
    IKResult r = solveIK(T, 0.0, ik);
    if (!r.ok) { std::printf("obj %s: grasp pose IK FAIL (%s)\n", nm[i], r.reason.c_str()); ++bad; continue; }
    double h = r.q[0], XT = T.x * std::cos(h) + T.y * std::sin(h);
    double dpre = clampd(XT - 0.245, 0.0, 0.03);
    bool found = false;
    for (double tilt : {0.30, 0.40, 0.20}) {
      Vec3 P = eeOnPlane(h, XT - dpre, 0.035), S = eeOnPlane(h, XT - dpre - 0.05, 0.045), Q = S + Vec3{0, 0, 0.03};
      if (solveIK(P, 0, ik).ok && solveIK(S, tilt, ik).ok && solveIK(Q, tilt, ik).ok) { std::printf("obj %s: pick OK (tilt %.2f, d_pre %.3f, lift/transport z .16 ok=%d)\n", nm[i], tilt, dpre, (int)solveIK({T.x, T.y, 0.16}, 0, ik).ok); found = true; break; }
    }
    if (!found) { std::printf("obj %s: no standoff\n", nm[i]); ++bad; }
  }
  // stack slots
  double sc[2] = {0.0590, 0.3348}, az = std::atan2(sc[1], sc[0]), sp = 0.046;
  Vec3 tg{-std::sin(az), std::cos(az), 0};
  int counts[3] = {3, 2, 1};
  for (int l = 0; l < 3; ++l)
    for (int i = 0; i < counts[l]; ++i) {
      double off = (i - (counts[l] - 1) / 2.0) * sp;
      Vec3 c = Vec3{sc[0], sc[1], 0.037 + l * 0.074} + off * tg;
      double ztr = std::max(0.16, 0.074 * (l + 1) + 0.02 + 0.017 + 0.037);
      bool a = solveIK({c.x, c.y, c.z + 0.017 + 0.02}, 0, ik).ok, b = solveIK({c.x, c.y, c.z - 0.002}, 0, ik).ok,
           d = solveIK({c.x, c.y, ztr + 0.0}, 0, ik).ok;
      IKResult rr = solveIK({c.x, c.y, c.z - 0.002}, 0, ik);
      Vec3 bk = c + Vec3{-0.05 * std::cos(rr.q[0]), -0.05 * std::sin(rr.q[0]), 0};
      bool e = solveIK({bk.x, bk.y, c.z - 0.002}, 0, ik).ok;
      std::printf("slot L%d.%d (%.3f,%.3f,%.3f): hover %d release %d transport(z=%.3f) %d retract %d\n", l + 1, i, c.x, c.y, c.z, a, b, ztr, d, e);
      bad += !(a && b && d && e);
      // verification view
      bool vv = false;
      const double V[8][3] = {{80,.14,170},{70,.14,170},{70,.20,170},{60,.14,170},{60,.20,170},{50,.14,170},{50,.20,170},{40,.14,170}};
      for (auto& v : V) { IKResult p; if (scanPose({c.x, c.y, c.z + 0.037}, v[1], v[0], v[2], K, p)) { vv = true; break; } }
      std::printf("   verification viewpoint reachable: %d\n", (int)vv);
      bad += !vv;
    }
  // scan stations
  int sbad = 0, stot = 0;
  for (double r : {0.28, 0.32, 0.30})
    for (int azd : {-64, -52, -40, -28, -16, -4, 8, 20, 32, 44, 56, 68}) {
      IKResult p; ++stot;
      if (!scanPose({r * std::cos(azd * M_PI / 180), r * std::sin(azd * M_PI / 180), 0.074}, 0.2, 80, 170, K, p)) { ++sbad; std::printf("  unreachable station r=%.2f az=%d\n", r, azd); }
    }
  std::printf("scan stations unreachable: %d / %d\n", sbad, stot);
  std::printf("localize viewpoints (above each object): ");
  for (int i = 0; i < 6; ++i) { IKResult p; std::printf("%d ", (int)scanPose({objs[i][0], objs[i][1], 0.074}, 0.2, 80, 170, K, p)); }
  std::printf("\nTOTAL problems: %d\n", bad);
  return bad;
}
