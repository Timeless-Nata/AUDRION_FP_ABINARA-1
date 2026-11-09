#include "autonomous_manipulator/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace am {

MotionPlanner::~MotionPlanner() {
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

bool MotionPlanner::load(const std::string& path, double margin) {
  char err[1000] = "";
  m_ = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
  if (!m_) { ROS_FATAL("planner: cannot load MJCF: %s", err); return false; }
  d_ = mj_makeData(m_);
  const char* jn[6] = {"hip", "shoulder", "arm", "hand", "finger1", "finger2"};
  for (int i = 0; i < 6; ++i) {
    int j = mj_name2id(m_, mjOBJ_JOINT, jn[i]);
    if (j < 0) { ROS_FATAL("planner: joint %s missing", jn[i]); return false; }
    qadr_[i] = m_->jnt_qposadr[j];
  }
  jaw_up_ = mj_name2id(m_, mjOBJ_BODY, "jaw_upper");
  jaw_lo_ = mj_name2id(m_, mjOBJ_BODY, "jaw_lower");
  int base = mj_name2id(m_, mjOBJ_BODY, "base");
  robot_body_.assign(m_->nbody, 0);
  obj_flag_.assign(m_->nbody, 0);
  for (int b = 1; b < m_->nbody; ++b) {
    const char* nm = mj_id2name(m_, mjOBJ_BODY, b);
    if (nm && std::string(nm).rfind("dumbbell_", 0) == 0) {
      obj_body_.push_back(b);
      std::string s(nm);
      obj_color_.push_back(s.substr(s.rfind('_') + 1));
      obj_flag_[b] = 1;
    }
  }
  // robot bodies = base and all of its descendants
  for (int b = 1; b < m_->nbody; ++b) {
    int x = b;
    while (x > 0 && x != base) x = m_->body_parentid[x];
    if (x == base) robot_body_[b] = 1;
  }
  for (int g = 0; g < m_->ngeom; ++g) m_->geom_margin[g] = margin;
  // static contacts of the neutral pose with all objects parked far away
  PlanContext ctx;
  for (double f : {0.0, 0.9}) {
    ctx.finger = f;
    JointVec q{};
    inCollision(q, ctx, nullptr);  // fills nothing; we recompute below
    for (int i = 0; i < d_->ncon; ++i) {
      const mjContact& c = d_->contact[i];
      static_pairs_.insert({std::min(c.geom1, c.geom2), std::max(c.geom1, c.geom2)});
    }
  }
  ROS_INFO("planner: %zu geom pairs in neutral contact are whitelisted (model artefacts)", static_pairs_.size());
  return true;
}

void MotionPlanner::setFree(int i, const Vec3& p) {
  int b = obj_body_[i];
  int adr = m_->jnt_qposadr[m_->body_jntadr[b]];
  d_->qpos[adr] = p.x; d_->qpos[adr + 1] = p.y; d_->qpos[adr + 2] = p.z;
  // upright: body-local Y (bar axis) -> world Z  == rotation of +90 deg about X
  d_->qpos[adr + 3] = std::sqrt(0.5); d_->qpos[adr + 4] = std::sqrt(0.5); d_->qpos[adr + 5] = 0; d_->qpos[adr + 6] = 0;
}

bool MotionPlanner::inCollision(const JointVec& q, const PlanContext& ctx, std::string* why) {
  for (int i = 0; i < 4; ++i) d_->qpos[qadr_[i]] = q[i];
  d_->qpos[qadr_[4]] = d_->qpos[qadr_[5]] = ctx.finger;
  held_body_ = target_body_ = -1;
  std::vector<int> used(obj_body_.size(), 0);
  Pose ee = forwardKinematics(q);
  for (size_t k = 0; k < ctx.objects.size(); ++k) {
    // assign the k-th believed object to an unused body of the same colour
    for (size_t b = 0; b < obj_body_.size(); ++b) {
      if (!used[b] && obj_color_[b] == ctx.objects[k].color) {
        used[b] = 1;
        Vec3 c = ctx.objects[k].center;
        if (static_cast<int>(k) == ctx.held) c = ee.p - Vec3{0, 0, ctx.carry_dz};
        setFree(static_cast<int>(b), c);
        if (static_cast<int>(k) == ctx.held) held_body_ = obj_body_[b];
        if (static_cast<int>(k) == ctx.target) target_body_ = obj_body_[b];
        break;
      }
    }
  }
  int park = 0;
  for (size_t b = 0; b < obj_body_.size(); ++b)
    if (!used[b]) setFree(static_cast<int>(b), {3.0 + 0.2 * park++, 3.0, 0.037});
  mj_forward(m_, d_);
  for (int i = 0; i < d_->ncon; ++i) {
    const mjContact& c = d_->contact[i];
    int g1 = std::min(c.geom1, c.geom2), g2 = std::max(c.geom1, c.geom2);
    if (static_pairs_.count({g1, g2})) continue;
    int b1 = m_->geom_bodyid[g1], b2 = m_->geom_bodyid[g2];
    bool r1 = robot_body_[b1], r2 = robot_body_[b2];
    bool o1 = obj_flag_[b1], o2 = obj_flag_[b2];
    if (!(r1 || r2 || (o1 && o2) || (o1 && b2 == 0) || (o2 && b1 == 0))) continue;
    int ob = o1 ? b1 : b2, other = o1 ? b2 : b1;
    // the held object touches the gripper by design (it hangs on / sits in the ring)
    if (held_body_ >= 0 && ((b1 == held_body_ && r2) || (b2 == held_body_ && r1))) continue;
    if (ctx.target >= 0 && (o1 != o2) && ob == target_body_ && (other == jaw_up_ || other == jaw_lo_)) {
      const char* gn = mj_id2name(m_, mjOBJ_GEOM, o1 ? g1 : g2);
      if (gn && std::string(gn).find("_bar") != std::string::npos) continue;  // jaws may touch the bar (grasp)
    }
    if (why) {
      const char* a = mj_id2name(m_, mjOBJ_BODY, b1);
      const char* b = mj_id2name(m_, mjOBJ_BODY, b2);
      *why = std::string("contact ") + (a ? a : "world") + " <-> " + (b ? b : "world") + " (dist " + std::to_string(c.dist) + ")";
    }
    return true;
  }
  return false;
}

bool MotionPlanner::validate(const JointTrajectory& tr, const PlanContext& ctx, std::string* why) {
  if (tr.empty()) return true;
  size_t stride = 5;
  Vec3 last = forwardKinematics(tr.q.front()).p;
  if (inCollision(tr.q.front(), ctx, why)) return false;
  for (size_t i = 1; i < tr.q.size(); ++i) {
    bool end = (i + 1 == tr.q.size());
    Vec3 p = forwardKinematics(tr.q[i]).p;
    if (end || i % stride == 0 || norm(p - last) > 0.02) {
      last = p;
      if (inCollision(tr.q[i], ctx, why)) return false;
    }
  }
  return true;
}

}  // namespace am
