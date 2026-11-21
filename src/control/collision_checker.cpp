#include "autonomous_manipulator/collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace arm {
namespace {

bool nameMatch(const std::string& pat, const std::string& n) { return pat == "*" || pat == n; }

// Separating-axis test for two convex polyhedra. Returns the smallest overlap over all axes
// (> 0 penetration depth, <= 0 the polyhedra are separated along that axis), exiting early as soon as an axis
// separates them by at least `margin`. `separated` tells which case happened.
double satOverlap(const std::vector<Vec3>& va, const std::vector<Vec3>& na, const std::vector<Vec3>& ea,
                  const std::vector<Vec3>& vb, const std::vector<Vec3>& nb, const std::vector<Vec3>& eb, double margin,
                  bool& separated) {
  double minOv = std::numeric_limits<double>::infinity();
  separated = false;
  auto testAxis = [&](Vec3 ax) -> bool {  // true = separating axis found
    double aMin = 1e300, aMax = -1e300, bMin = 1e300, bMax = -1e300;
    for (const Vec3& p : va) {
      const double d = dot(p, ax);
      aMin = std::fmin(aMin, d);
      aMax = std::fmax(aMax, d);
    }
    for (const Vec3& p : vb) {
      const double d = dot(p, ax);
      bMin = std::fmin(bMin, d);
      bMax = std::fmax(bMax, d);
    }
    const double ov = std::fmin(aMax - bMin, bMax - aMin);
    minOv = std::fmin(minOv, ov);
    return ov <= -margin;
  };
  for (const Vec3& a : na)
    if (testAxis(a)) { separated = true; return minOv; }
  for (const Vec3& a : nb)
    if (testAxis(a)) { separated = true; return minOv; }
  for (const Vec3& a : ea) {
    for (const Vec3& b : eb) {
      Vec3 c = cross(a, b);
      const double L = norm(c);
      if (L < 1e-6) continue;  // (nearly) parallel edges: covered by the face axes
      if (testAxis((1.0 / L) * c)) { separated = true; return minOv; }
    }
  }
  return minOv;
}

}  // namespace

CollisionChecker::CollisionChecker(const RobotModel& model, const CollisionOptions& opt)
    : model_(&model), kin_(model), opt_(opt) {
  const std::vector<Hull>& H = model.hulls;
  for (size_t i = 0; i < H.size(); ++i) {
    for (size_t j = i + 1; j < H.size(); ++j) {
      const int a = H[i].body, b = H[j].body;
      if (a == b) continue;                                    // same body
      if (a == 0 && b == 0) continue;                          // static vs static
      if (a != 0 && b != 0 && (model.bodies[a].parent == b || model.bodies[b].parent == a)) continue;  // direct parent/child
      bool allowed = false;
      for (const auto& p : opt.allowed) {
        if ((nameMatch(p.first, H[i].name) && nameMatch(p.second, H[j].name)) ||
            (nameMatch(p.first, H[j].name) && nameMatch(p.second, H[i].name)))
          allowed = true;
      }
      if (!allowed) pairs_.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
  }
}

void CollisionChecker::prepare(const Pose& T, const Hull& h, WorldHull& w) const {
  w.v.resize(h.v.size());
  for (size_t i = 0; i < h.v.size(); ++i) w.v[i] = apply(T, h.v[i]);
  w.n.resize(h.normals.size());
  for (size_t i = 0; i < h.normals.size(); ++i) w.n[i] = T.R * h.normals[i];
  w.e.resize(h.edges.size());
  for (size_t i = 0; i < h.edges.size(); ++i) w.e[i] = T.R * h.edges[i];
  w.c = apply(T, h.center);
  w.r = h.radius;
  w.ready = true;
}

CollisionReport CollisionChecker::check(const JointVec& q, double finger_active) const {
  const DofVec d{q[0], q[1], q[2], q[3], finger_active};
  const std::vector<Pose> W = kin_.bodyPoses(d);
  const std::vector<Hull>& H = model_->hulls;
  std::vector<WorldHull> wh(H.size());
  CollisionReport rep;
  const double m = opt_.margin;

  auto add = [&](const std::string& a, const std::string& b, double depth) {
    rep.contacts.push_back({a, b, depth});
    rep.max_depth = std::max(rep.max_depth, depth);
    rep.in_collision = true;
  };

  if (opt_.check_floor) {
    for (size_t i = 0; i < H.size(); ++i) {
      if (H[i].body == 0) continue;
      double zmin = 1e300;
      const Pose& T = W[H[i].body];
      for (const Vec3& p : H[i].v) zmin = std::fmin(zmin, apply(T, p).z);
      const double depth = opt_.floor_z - zmin;
      if (depth > -m) add("FLOOR", H[i].name, depth);
    }
  }
  for (const auto& pr : pairs_) {
    const Hull& ha = H[pr.first];
    const Hull& hb = H[pr.second];
    WorldHull& A = wh[pr.first];
    WorldHull& B = wh[pr.second];
    if (!A.ready) prepare(W[ha.body], ha, A);
    if (!B.ready) prepare(W[hb.body], hb, B);
    if (norm(A.c - B.c) > A.r + B.r + m) continue;  // bounding spheres apart
    bool sep = false;
    const double ov = satOverlap(A.v, A.n, A.e, B.v, B.n, B.e, m, sep);
    if (!sep) add(ha.name, hb.name, ov);
  }
  std::sort(rep.contacts.begin(), rep.contacts.end(), [](const Contact& x, const Contact& y) { return x.depth > y.depth; });
  return rep;
}

bool CollisionChecker::pathCollisionFree(const JointVec& a, const JointVec& b, double finger_active, double step_rad,
                                         CollisionReport* first_hit) const {
  double span = 0;
  for (int i = 0; i < kArmDof; ++i) span = std::fmax(span, std::fabs(b[i] - a[i]));
  const int n = std::max(1, static_cast<int>(std::ceil(span / std::fmax(step_rad, 1e-4))));
  for (int k = 0; k <= n; ++k) {
    JointVec q;
    const double s = static_cast<double>(k) / n;
    for (int i = 0; i < kArmDof; ++i) q[i] = a[i] + s * (b[i] - a[i]);
    CollisionReport r = check(q, finger_active);
    if (r.in_collision) {
      if (first_hit) *first_hit = r;
      return false;
    }
  }
  return true;
}

std::vector<CollisionChecker::Interval> CollisionChecker::freeIntervals(JointVec q, int joint, double finger_active,
                                                                         double step_rad) const {
  std::vector<Interval> out;
  const double lo = kin_.lower()[joint], hi = kin_.upper()[joint];
  const int n = std::max(1, static_cast<int>(std::ceil((hi - lo) / step_rad)));
  bool open = false;
  double start = lo, last = lo;
  for (int k = 0; k <= n; ++k) {
    const double v = lo + (hi - lo) * k / n;
    q[joint] = v;
    const bool free = !check(q, finger_active).in_collision;
    if (free && !open) { open = true; start = v; }
    if (!free && open) { out.push_back({start, last}); open = false; }
    last = v;
  }
  if (open) out.push_back({start, last});
  return out;
}

}  // namespace arm
