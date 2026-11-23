// collision_checker.hpp - detects whether a joint configuration makes links overlap
// (link-link, link-base/mount plate, link-floor), using the convex collision hulls of the model.
//
// This is the "do the links overlap at this pose?" test. It uses the exact separating-axis test for convex
// polyhedra, so penetration depths are exact for the hulls. Which pairs are tested follows the rules MuJoCo
// applies, so the result predicts what the simulation will report:
//   * geoms on the same body are never tested,
//   * a body and its direct parent are not tested, EXCEPT the world (revolute_1 is tested against the base),
//   * static-static pairs (base_link vs mount plate) are not tested.
#pragma once
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "autonomous_manipulator/kinematics.hpp"

namespace arm {

struct CollisionOptions {
  double margin = 0.0;          // [m] a pair counts as colliding when it is closer than this (0 = only real overlap)
  bool check_floor = true;      // robot hulls must stay above floor_z
  double floor_z = 0.0;         // [m]
  // pairs that may overlap by design, matched on hull names ("*" matches anything)
  std::vector<std::pair<std::string, std::string>> allowed;

  // Options with a safety margin. revolute_1 sits flush on the base box by design and only rotates about the
  // vertical axis, so that contact never changes with the joints: it is excluded (it would always sit inside any margin).
  static CollisionOptions withMargin(double margin) {
    CollisionOptions o;
    o.margin = margin;
    for (int i = 0; i < 6; ++i) o.allowed.emplace_back("base_link_g1", "col_revolute_1_" + std::to_string(i));
    return o;
  }
};

struct Contact {
  std::string a, b;   // hull names ("FLOOR" for the floor plane)
  double depth = 0;   // [m] penetration depth (> 0 overlap; <= 0 only inside the margin)
};

struct CollisionReport {
  bool in_collision = false;
  double max_depth = -std::numeric_limits<double>::infinity();
  std::vector<Contact> contacts;  // sorted, deepest first
};

class CollisionChecker {
 public:
  CollisionChecker(const RobotModel& model, const CollisionOptions& opt = CollisionOptions());

  CollisionReport check(const JointVec& q, double finger_active) const;

  // Checks the straight joint-space segment a -> b sampled every `step_rad` (max over joints). True if free.
  bool pathCollisionFree(const JointVec& a, const JointVec& b, double finger_active, double step_rad,
                         CollisionReport* first_hit = nullptr) const;

  struct Interval {
    double lo, hi;
  };
  // Collision-free ranges of one joint (others fixed at q), scanned over its full limits.
  std::vector<Interval> freeIntervals(JointVec q, int joint, double finger_active, double step_rad = 0.01) const;

  const Kinematics& kinematics() const { return kin_; }
  size_t pairCount() const { return pairs_.size(); }

 private:
  struct WorldHull {
    std::vector<Vec3> v, n, e;
    Vec3 c;
    double r = 0;
    bool ready = false;
  };
  void prepare(const Pose& T, const Hull& h, WorldHull& w) const;

  const RobotModel* model_;
  Kinematics kin_;
  CollisionOptions opt_;
  std::vector<std::pair<int, int>> pairs_;  // hull index pairs to test
};

}  // namespace arm
