#include "autonomous_manipulator/color_classifier.hpp"

#include <algorithm>
#include <cmath>

namespace arm {

ColorConfig ColorConfig::simulationDefaults() {
  ColorConfig c;
  // order matters: black first (very dark pixels have undefined hue), then the saturated orange, then the pale beige
  c.classes = {
      {"black", 0, 180, 0, 255, 0, 45},
      {"orange", 8, 24, 180, 255, 80, 255},
      {"beige", 12, 34, 25, 140, 90, 255},  // cream top (H30 S37 V255) and tan sides (H19 S85 V100-170); floor is S<=18, shadow is H60
  };
  return c;
}

void rgbToHsv(uint8_t r8, uint8_t g8, uint8_t b8, int& h, int& s, int& v) {
  const int mx = std::max({static_cast<int>(r8), static_cast<int>(g8), static_cast<int>(b8)});
  const int mn = std::min({static_cast<int>(r8), static_cast<int>(g8), static_cast<int>(b8)});
  const int d = mx - mn;
  v = mx;
  s = mx > 0 ? static_cast<int>(std::lround(255.0 * d / mx)) : 0;
  if (d == 0) {
    h = 0;
    return;
  }
  double hh;  // degrees 0..360
  if (mx == r8) hh = 60.0 * (static_cast<double>(g8) - b8) / d;
  else if (mx == g8) hh = 120.0 + 60.0 * (static_cast<double>(b8) - r8) / d;
  else hh = 240.0 + 60.0 * (static_cast<double>(r8) - g8) / d;
  if (hh < 0) hh += 360.0;
  h = static_cast<int>(std::lround(hh / 2.0)) % 180;
}

static bool inRange(const HsvRange& r, int h, int s, int v) {
  const bool hok = r.h_lo <= r.h_hi ? (h >= r.h_lo && h <= r.h_hi) : (h >= r.h_lo || h <= r.h_hi);
  return hok && s >= r.s_lo && s <= r.s_hi && v >= r.v_lo && v <= r.v_hi;
}

ColorResult classifyColor(const uint8_t* rgb, int width, int height, const ColorConfig& cfg) {
  ColorResult res;
  const size_t n = cfg.classes.size();
  res.fraction.assign(n, 0.0);
  if (!rgb || width <= 0 || height <= 0 || n == 0) return res;
  const int x0 = std::max(0, static_cast<int>(cfg.roi_x0 * width)), x1 = std::min(width, static_cast<int>(cfg.roi_x1 * width));
  const int y0 = std::max(0, static_cast<int>(cfg.roi_y0 * height)), y1 = std::min(height, static_cast<int>(cfg.roi_y1 * height));
  if (x1 <= x0 || y1 <= y0) return res;

  std::vector<long> count(n, 0);
  std::vector<double> sum_x(n, 0.0);
  for (int y = y0; y < y1; ++y) {
    const uint8_t* row = rgb + static_cast<size_t>(y) * width * 3;
    for (int x = x0; x < x1; ++x) {
      int h, s, v;
      rgbToHsv(row[3 * x], row[3 * x + 1], row[3 * x + 2], h, s, v);
      for (size_t k = 0; k < n; ++k)
        if (inRange(cfg.classes[k], h, s, v)) {
          ++count[k];
          sum_x[k] += x;
          break;
        }
    }
  }
  const double total = static_cast<double>(x1 - x0) * (y1 - y0);
  int first = -1, second = -1;
  for (size_t k = 0; k < n; ++k) {
    res.fraction[k] = count[k] / total;
    if (first < 0 || count[k] > count[first]) {
      second = first;
      first = static_cast<int>(k);
    } else if (second < 0 || count[k] > count[second]) {
      second = static_cast<int>(k);
    }
  }
  if (count[first] <= 0 || res.fraction[first] < cfg.min_fraction) return res;  // NoObject
  res.dominant = first;
  res.name = cfg.classes[first].name;
  res.second = (second >= 0 && count[second] > 0) ? second : -1;
  if (res.second >= 0) res.second_cx = (sum_x[res.second] / count[res.second] + 0.5) / width;
  res.second_ratio = res.second >= 0 ? static_cast<double>(count[second]) / count[first] : 0.0;
  res.status = res.second_ratio > cfg.max_second_ratio ? ColorStatus::Ambiguous : ColorStatus::Unique;
  // 0..1: how far the second colour is below the ambiguity limit, times how large the dominant colour is (full credit from 3x min_fraction)
  res.confidence = std::max(0.0, std::min(1.0, 1.0 - res.second_ratio / cfg.max_second_ratio)) *
                   std::min(1.0, res.fraction[first] / (3.0 * cfg.min_fraction));
  return res;
}

}  // namespace arm
