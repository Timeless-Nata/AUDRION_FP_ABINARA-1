// test_color.cpp - tests of the colour classifier: synthetic images + two real MuJoCo camera frames (160x120 PPM).
//   arm_color_test <test/data directory>
#include <cstdio>
#include <fstream>
#include <string>

#include "autonomous_manipulator/color_classifier.hpp"

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

static bool readPpm(const std::string& path, int& w, int& h, std::vector<uint8_t>& px) {
  std::ifstream f(path, std::ios::binary);
  std::string magic;
  int maxv;
  if (!(f >> magic >> w >> h >> maxv) || magic != "P6") return false;
  f.get();
  px.resize(static_cast<size_t>(w) * h * 3);
  f.read(reinterpret_cast<char*>(px.data()), px.size());
  return static_cast<size_t>(f.gcount()) == px.size();
}

static std::vector<uint8_t> fill(int w, int h, uint8_t r, uint8_t g, uint8_t b) {
  std::vector<uint8_t> p(static_cast<size_t>(w) * h * 3);
  for (size_t i = 0; i < p.size(); i += 3) { p[i] = r; p[i + 1] = g; p[i + 2] = b; }
  return p;
}
static void paint(std::vector<uint8_t>& p, int w, int x0, int x1, int y0, int y1, uint8_t r, uint8_t g, uint8_t b) {
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) { size_t i = (static_cast<size_t>(y) * w + x) * 3; p[i] = r; p[i + 1] = g; p[i + 2] = b; }
}

int main(int argc, char** argv) {
  const ColorConfig cfg = ColorConfig::simulationDefaults();
  std::printf("[hsv conversion]\n");
  int h, s, v;
  rgbToHsv(255, 128, 0, h, s, v);  CHECK(h == 15 && s == 255 && v == 255, "orange -> %d %d %d", h, s, v);
  rgbToHsv(0, 0, 0, h, s, v);      CHECK(v == 0 && s == 0, "black");
  rgbToHsv(255, 0, 0, h, s, v);    CHECK(h == 0 && s == 255, "red");
  rgbToHsv(0, 0, 255, h, s, v);    CHECK(h == 120, "blue h=%d", h);

  std::printf("[synthetic]\n");
  {
    auto img = fill(100, 100, 240, 240, 240);  // white-grey floor
    ColorResult r = classifyColor(img.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::NoObject, "floor only");
    paint(img, 100, 20, 70, 20, 70, 255, 128, 0);  // orange block
    r = classifyColor(img.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::Unique && r.name == "orange", "orange block: %s", r.name.c_str());
    paint(img, 100, 75, 95, 20, 70, 20, 20, 20);   // a black block next to it (second ratio 0.4)
    r = classifyColor(img.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::Ambiguous && r.name == "orange" && r.second == 0, "two blocks must be ambiguous");
    auto img2 = fill(100, 100, 230, 230, 230);
    paint(img2, 100, 10, 90, 10, 90, 20, 20, 20);
    paint(img2, 100, 90, 92, 10, 90, 240, 120, 10);  // 2.5 % orange sliver next to a large black block
    r = classifyColor(img2.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::Unique && r.name == "black", "sliver tolerated: status %d %s", static_cast<int>(r.status), r.name.c_str());
    auto img3 = fill(100, 100, 150, 175, 150);  // grey-green shadow (H60) must not match any class
    r = classifyColor(img3.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::NoObject, "shadow");
    // blue sky must not match any class either
    auto img4 = fill(100, 100, 110, 140, 190);
    r = classifyColor(img4.data(), 100, 100, cfg);
    CHECK(r.status == ColorStatus::NoObject, "sky");
    ColorConfig roi = cfg;
    roi.roi_x0 = 0.0; roi.roi_x1 = 0.5;  // ROI hides the right half
    auto img5 = fill(100, 100, 230, 230, 230);
    paint(img5, 100, 10, 40, 10, 90, 255, 128, 0);
    paint(img5, 100, 60, 90, 10, 90, 20, 20, 20);
    r = classifyColor(img5.data(), 100, 100, roi);
    CHECK(r.status == ColorStatus::Unique && r.name == "orange", "ROI excludes the black block");
  }

  std::printf("[real MuJoCo frames]\n");
  if (argc >= 2) {
    int w, hh;
    std::vector<uint8_t> px;
    CHECK(readPpm(std::string(argv[1]) + "/sim_frame_black_close.ppm", w, hh, px), "read black_close");
    ColorResult r = classifyColor(px.data(), w, hh, cfg);
    std::printf("   close view of the black block: dominant %s, black %.1f %%, orange %.1f %%, beige %.1f %%, second/dominant %.2f, %s\n",
                r.name.c_str(), 100 * r.fraction[0], 100 * r.fraction[1], 100 * r.fraction[2], r.second_ratio,
                r.status == ColorStatus::Unique ? "UNIQUE" : "AMBIGUOUS");
    CHECK(r.name == "black" && r.status == ColorStatus::Unique, "black close view");
    CHECK(readPpm(std::string(argv[1]) + "/sim_frame_home_scan_2blocks.ppm", w, hh, px), "read scan frame");
    r = classifyColor(px.data(), w, hh, cfg);
    std::printf("   view with a black and a beige block (+ orange at the edges): black %.1f %%, orange %.1f %%, beige %.1f %%, %s\n",
                100 * r.fraction[0], 100 * r.fraction[1], 100 * r.fraction[2], r.status == ColorStatus::Unique ? "UNIQUE" : "AMBIGUOUS");
    CHECK(r.status == ColorStatus::Ambiguous, "several blocks in view must be ambiguous");
  } else {
    std::printf("   (skipped: pass the test/data directory)\n");
  }
  std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}
