// view_search.hpp - find a camera view that shows exactly ONE colour (pure C++17).
//
// Closed loop: go to a view, classify the frame, and if a second colour is also visible move the camera (standoff,
// sideways away from the second colour, steeper pitch) until only one colour is left or the attempts run out.
// Moving the arm and grabbing the frame is done by a callback, so the logic does not know ROS or MuJoCo.
#pragma once
#include <functional>
#include <string>
#include <vector>

#include "autonomous_manipulator/color_classifier.hpp"
#include "autonomous_manipulator/view_planner.hpp"

namespace arm {

struct ViewSearchParams {
  double pitch_deg = -40.0;                          // first view: optical axis elevation (steeper is only reachable very close)
  std::vector<double> standoffs = {0.12, 0.10, 0.08};  // [m] tried in this order (closer = fewer neighbours in view)
  std::vector<double> lateral_steps = {0.02, 0.04};    // [m] sideways shifts, away from the second colour
  std::vector<double> steeper_pitches_deg = {-45.0};   // reachable only at a short standoff (see steeper_standoff)
  double steeper_standoff = 0.065;                    // [m] standoff used for the steeper pitches
  int max_attempts = 6;                              // frames evaluated at most
  int max_candidates = 20;                           // planned views at most (unreachable ones do not use an attempt)
};

enum class SearchStatus { Unique, Ambiguous, NoObject, Unreachable };

struct ViewSearchResult {
  SearchStatus status = SearchStatus::Unreachable;
  std::string color;       // dominant colour name when Unique (or the best guess when Ambiguous)
  double confidence = 0;
  int attempts = 0;        // frames evaluated
  ViewPose view;           // last evaluated view
  std::vector<std::string> trace;
};

// Moves the arm to `view` and returns the classification of the frame taken there. false = the move failed.
using ViewEvaluator = std::function<bool(const ViewPose& view, ColorResult& result)>;

ViewSearchResult searchSingleColorView(const ViewPlanner& planner, Vec3 target, const JointVec& seed,
                                       const ViewEvaluator& evaluate, const ViewSearchParams& params = ViewSearchParams());

}  // namespace arm
