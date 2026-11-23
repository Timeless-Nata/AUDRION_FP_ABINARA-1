#include "autonomous_manipulator/view_search.hpp"

#include <cstdio>

namespace arm {

ViewSearchResult searchSingleColorView(const ViewPlanner& planner, Vec3 target, const JointVec& seed, const ViewEvaluator& evaluate,
                                       const ViewSearchParams& P) {
  ViewSearchResult res;
  if (P.standoffs.empty()) return res;
  const double pitch0 = P.pitch_deg * kPi / 180.0;
  const double d_min = P.standoffs.back();

  struct Cand {
    double d, pitch, lat;
    std::string why;
  };
  std::vector<Cand> queue{{P.standoffs[0], pitch0, 0.0, "first view"}};
  size_t qi = 0, stage = 1;  // stage = next escalation step
  JointVec cur_seed = seed;
  ColorResult last;
  bool have_last = false, tried_far = false;
  int candidates = 0;

  auto describe = [](const Cand& c) {
    char b[160];
    std::snprintf(b, sizeof b, "%s [standoff %.0f mm, pitch %.0f deg, lateral %+.0f mm]", c.why.c_str(), c.d * 1000,
                  c.pitch * 180 / kPi, c.lat * 1000);
    return std::string(b);
  };

  // Adds the next candidate after a view that was not unique: 1) closer  2) sideways, away from the second colour
  // 3) steeper pitch. Nothing in view: one wider view.
  auto escalate = [&]() {
    if (last.status == ColorStatus::NoObject) {
      if (!tried_far) {
        tried_far = true;
        queue.push_back({P.standoffs[0] * 1.3, pitch0, 0.0, "nothing in view: wider"});
      }
      return;
    }
    const size_t nd = P.standoffs.size(), nl = P.lateral_steps.size(), np = P.steeper_pitches_deg.size();
    if (stage < nd) {
      queue.push_back({P.standoffs[stage], pitch0, 0.0, "closer"});
    } else if (stage < nd + nl) {
      const double step = P.lateral_steps[stage - nd];
      // second colour on the right of the image -> aim further right (negative lateral) so the target moves left, away from it
      queue.push_back({d_min, pitch0, last.second_cx > 0.5 ? -step : step, "sideways"});
    } else if (stage < nd + nl + np) {
      queue.push_back({P.steeper_standoff, P.steeper_pitches_deg[stage - nd - nl] * kPi / 180.0, 0.0, "steeper"});
    } else {
      return;
    }
    ++stage;
  };

  while (res.attempts < P.max_attempts && candidates < P.max_candidates) {
    if (qi == queue.size()) {
      if (!have_last) break;
      escalate();
      if (qi == queue.size()) break;  // nothing left to try
    }
    const Cand c = queue[qi++];
    ++candidates;
    ViewPose view;
    std::string why;
    if (!planner.plan(target, c.d, c.pitch, c.lat, cur_seed, view, &why)) {
      res.trace.push_back("skip " + describe(c) + ": " + why);
      continue;
    }
    ++res.attempts;
    res.view = view;
    ColorResult r;
    if (!evaluate(view, r)) {
      res.trace.push_back("move failed " + describe(c));
      continue;
    }
    cur_seed = view.q;
    last = r;
    have_last = true;
    res.color = r.name;
    res.confidence = r.confidence;
    std::string line = describe(c) + " -> ";
    if (r.status == ColorStatus::Unique) line += "UNIQUE " + r.name;
    else if (r.status == ColorStatus::Ambiguous) line += "AMBIGUOUS " + r.name + " (second colour " + std::to_string(static_cast<int>(100 * r.second_ratio)) + " % of it)";
    else line += "no object";
    res.trace.push_back(line);
    if (r.status == ColorStatus::Unique) {
      res.status = SearchStatus::Unique;
      return res;
    }
    escalate();
  }
  res.status = !have_last ? SearchStatus::Unreachable : last.status == ColorStatus::NoObject ? SearchStatus::NoObject : SearchStatus::Ambiguous;
  return res;
}

}  // namespace arm
