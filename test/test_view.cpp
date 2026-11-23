// test_view.cpp - view planning and the single-colour view search (no ROS, no MuJoCo).
//   arm_view_test <config/robot_model.txt>
// The search is run against a synthetic renderer: it paints the six dumbbells of scene.xml (positions and colours from
// the scene) into a small image through the camera model, and the REAL ColorClassifier judges that image.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "autonomous_manipulator/view_search.hpp"

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

struct Block { double x, y; const char* color; };
static const Block kBlocks[6] = {{0.181809, -0.224054, "black"}, {0.241995, -0.157139, "orange"}, {0.278638, -0.0749363, "beige"},
                                 {0.288171, 0.0145574, "black"}, {0.269668, 0.102635, "orange"},  {0.224928, 0.180727, "orange"}};

static void rgbOf(const std::string& c, uint8_t* o) {
  if (c == "orange") { o[0] = 255; o[1] = 128; o[2] = 0; }
  else if (c == "black") { o[0] = 20; o[1] = 20; o[2] = 20; }
  else { o[0] = 240; o[1] = 225; o[2] = 185; }  // beige
}

// Paints the dumbbells as two 50 mm square plates each (z = 0.0555 and 0.0055), far to near.
static std::vector<uint8_t> render(const Pose& cam, const CameraModel& cm, int W, int H) {
  std::vector<uint8_t> img(static_cast<size_t>(W) * H * 3, 235);
  struct Item { double depth; int b; double z; };
  std::vector<Item> items;
  const Vec3 f = cm.forward(cam);
  for (int b = 0; b < 6; ++b)
    for (double z : {0.0055, 0.0555}) items.push_back({dot(Vec3{kBlocks[b].x, kBlocks[b].y, z} - cam.p, f), b, z});
  std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.depth > b.depth; });
  const double fpx = (cm.height / 2.0) / std::tan(cm.fovy_deg * kPi / 360.0);  // focal length in full-resolution pixels
  for (const Item& it : items) {
    double u, v;
    if (!cm.project(cam, {kBlocks[it.b].x, kBlocks[it.b].y, it.z}, u, v)) continue;
    const double hs = fpx * 0.025 / it.depth;
    uint8_t c[3];
    rgbOf(kBlocks[it.b].color, c);
    for (int y = std::max(0, static_cast<int>((v - hs) * H / cm.height)); y < std::min(H, static_cast<int>((v + hs) * H / cm.height)); ++y)
      for (int x = std::max(0, static_cast<int>((u - hs) * W / cm.width)); x < std::min(W, static_cast<int>((u + hs) * W / cm.width)); ++x)
        for (int k = 0; k < 3; ++k) img[(static_cast<size_t>(y) * W + x) * 3 + k] = c[k];
  }
  return img;
}

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s robot_model.txt\n", argv[0]); return 2; }
  const RobotModel model = RobotModel::loadFromFile(argv[1]);
  const Kinematics kin(model);
  const CollisionChecker chk(model, CollisionOptions::withMargin(0.003));
  const InverseKinematics ik(kin, &chk);
  const ViewPlanner planner(kin, chk, ik);
  const CameraModel cm;
  const JointVec home{0.0, 0.8901, 0.35, -0.540};

  std::printf("[camera model]\n");
  {
    // measured in MuJoCo: pose [0.05 0.40 -0.10 -1.40], block 4 top-plate centre ~ (200,~120); model: (200,103), bar centre (235,240)
    const Pose cam = kin.framePose("cam_esp32cam", {0.05, 0.40, -0.10, -1.40}, -0.7);
    double u, v;
    CHECK(cm.project(cam, {0.288171, 0.0145574, 0.0305}, u, v) && std::fabs(u - 235) < 3 && std::fabs(v - 240) < 3, "projection (%.0f,%.0f)", u, v);
  }

  std::printf("[view planner: every dumbbell, standoff 12 cm, pitch -40 deg]\n");
  for (int b = 0; b < 6; ++b) {
    const Vec3 t{kBlocks[b].x, kBlocks[b].y, 0.0305};
    ViewPose vp;
    std::string why;
    const bool ok = planner.plan(t, 0.12, -40.0 * kPi / 180, 0.0, home, vp, &why);
    CHECK(ok, "block %d: %s", b + 1, why.c_str());
    if (!ok) { std::printf("   block %d (%s): %s\n", b + 1, kBlocks[b].color, why.c_str()); continue; }
    double u, v;
    CHECK(cm.project(vp.cam, t, u, v) && std::fabs(u - 320) < 8 && std::fabs(v - 240) < 8, "block %d not centred (%.0f,%.0f)", b + 1, u, v);
    CHECK(!chk.check(vp.q, -0.7).in_collision, "collision");
    std::printf("   block %d (%-6s): q=[%+.3f %+.3f %+.3f %+.3f] camera z=%.3f m, block at (%.0f,%.0f)\n", b + 1, kBlocks[b].color, vp.q[0],
                vp.q[1], vp.q[2], vp.q[3], vp.cam.p.z, u, v);
  }
  {
    ViewPose a, l;
    const Vec3 t{kBlocks[3].x, kBlocks[3].y, 0.0305};
    CHECK(planner.plan(t, 0.10, -40.0 * kPi / 180, 0.0, home, a) && planner.plan(t, 0.10, -40.0 * kPi / 180, 0.03, home, l), "lateral plan");
    double ua, va, ul, vl;
    cm.project(a.cam, t, ua, va);
    cm.project(l.cam, t, ul, vl);
    std::printf("   lateral +30 mm (aim to the left): block moves from u=%.0f to u=%.0f\n", ua, ul);
    CHECK(ul > ua + 20, "aiming left must move the block to the RIGHT in the image");
  }

  std::printf("[closed-loop search, real classifier on a synthetic render]\n");
  ColorConfig ccfg = ColorConfig::simulationDefaults();
  for (int b = 0; b < 6; ++b) {
    const Vec3 t{kBlocks[b].x, kBlocks[b].y, 0.0305};
    auto evaluate = [&](const ViewPose& vp, ColorResult& r) {
      const std::vector<uint8_t> img = render(vp.cam, cm, 160, 120);
      r = classifyColor(img.data(), 160, 120, ccfg);
      return true;
    };
    const ViewSearchResult sr = searchSingleColorView(planner, t, home, evaluate);
    std::printf("   block %d (%s): %s %s after %d frame(s)\n", b + 1, kBlocks[b].color,
                sr.status == SearchStatus::Unique ? "UNIQUE" : sr.status == SearchStatus::Ambiguous ? "AMBIGUOUS" : sr.status == SearchStatus::NoObject ? "NO OBJECT" : "UNREACHABLE",
                sr.color.c_str(), sr.attempts);
    for (const std::string& l : sr.trace) std::printf("        %s\n", l.c_str());
    CHECK(sr.status == SearchStatus::Unique && sr.color == kBlocks[b].color, "block %d expected %s", b + 1, kBlocks[b].color);
  }

  std::printf("[search logic with fake evaluators]\n");
  {
    const Vec3 t{kBlocks[3].x, kBlocks[3].y, 0.0305};
    ColorResult amb;
    amb.status = ColorStatus::Ambiguous; amb.name = "black"; amb.dominant = 0; amb.second = 2; amb.second_ratio = 0.5; amb.second_cx = 0.8;
    int calls = 0;
    std::vector<double> laterals;
    auto never = [&](const ViewPose& vp, ColorResult& r) { ++calls; laterals.push_back(vp.lateral); r = amb; return true; };
    ViewSearchResult sr = searchSingleColorView(planner, t, home, never);
    CHECK(sr.status == SearchStatus::Ambiguous && calls == 6, "never resolves: %d calls", calls);
    bool right_dir = true;
    for (double l : laterals) if (l > 1e-9) right_dir = false;  // second colour on the right -> aim right (negative)
    CHECK(right_dir, "sideways shift must be away from a second colour on the right");
    auto none = [&](const ViewPose&, ColorResult& r) { r = ColorResult(); return true; };
    sr = searchSingleColorView(planner, t, home, none);
    CHECK(sr.status == SearchStatus::NoObject && sr.attempts <= 2, "no object: attempts %d", sr.attempts);
    auto failmove = [&](const ViewPose&, ColorResult&) { return false; };
    sr = searchSingleColorView(planner, t, home, failmove);
    CHECK(sr.status == SearchStatus::Unreachable, "move failures");
    sr = searchSingleColorView(planner, {0.9, 0.0, 0.03}, home, never);
    CHECK(sr.status == SearchStatus::Unreachable && sr.attempts == 0, "unreachable target");
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
