// mission_controller: state machine that builds the 3-2-1 pyramid autonomously.
//   perception (camera + parallax height check + ultrasonic) -> colour-by-colour scan -> object selection -> grasp planning (analytic IK) -> collision-checked
//   quintic trajectories -> grasp -> verification -> transport -> placement -> verification -> next object.
// The simulator's ground truth is NEVER used for decisions; it is only logged in the final validation report.
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <sstream>
#include <utility>

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "autonomous_manipulator/ObjectDetectionArray.h"
#include "autonomous_manipulator/geometry_utils.hpp"
#include "autonomous_manipulator/gripper_controller.hpp"
#include "autonomous_manipulator/motion_planner.hpp"
#include "autonomous_manipulator/robot_interface.hpp"

using namespace am;

enum class State {
  INITIALIZATION, WAIT_FOR_SIMULATION, SCAN_WORKSPACE, CLASSIFY_COLORS, SELECT_OBJECT, LOCALIZE_OBJECT,
  CALCULATE_GRASP, EXECUTE_GRASP, VERIFY_GRASP, TRANSPORT_OBJECT, PLACE_OBJECT, VERIFY_PLACEMENT,
  MISSION_COMPLETE, MISSION_FAILED
};
static const char* stateName(State s) {
  static const char* n[] = {"INITIALIZATION", "WAIT_FOR_SIMULATION", "SCAN_WORKSPACE", "CLASSIFY_COLORS", "SELECT_OBJECT",
                            "LOCALIZE_OBJECT", "CALCULATE_GRASP", "EXECUTE_GRASP", "VERIFY_GRASP", "TRANSPORT_OBJECT",
                            "PLACE_OBJECT", "VERIFY_PLACEMENT", "MISSION_COMPLETE", "MISSION_FAILED"};
  return n[static_cast<int>(s)];
}

struct Tracked {
  std::string color;
  Vec3 pos;       // body-origin (centre) position
  int obs = 0;
  bool placed = false;
};
struct Slot {
  int level;
  std::string color;
  Vec3 center;
  bool done = false;
};
struct Det {
  std::string color;
  Vec3 pos, ray_o, ray_d;
  int n = 0;
};
struct Cand {  // unverified detection
  std::string color;
  Vec3 pos;
  int obs = 0;
};
enum class HeightCheck { PASS, FAIL, UNKNOWN };
struct GraspPlan {
  double hip = 0, XT = 0, z = 0, d_pre = 0, tilt = 0;
};

class Mission {
 public:
  Mission(ros::NodeHandle& nh)
      : nh_(nh), cfg_("/am/mission"), robot_(nh), grip_(robot_, 0.9, 0.0) {
    loadParams();
    state_pub_ = nh_.advertise<std_msgs::String>("/mission/state", 5, true);
    det_sub_ = nh_.subscribe("/perception/detections", 20, &Mission::onDetections, this);
    gt_sub_ = nh_.subscribe("/sim/debug/object_poses", 1, &Mission::onGroundTruth, this);
    names_sub_ = nh_.subscribe("/sim/debug/object_names", 1, &Mission::onNames, this);
  }

  int run() {
    State s = State::INITIALIZATION;
    while (ros::ok()) {
      publishState(s);
      switch (s) {
        case State::INITIALIZATION: s = initialization(); break;
        case State::WAIT_FOR_SIMULATION: s = waitForSimulation(); break;
        case State::SCAN_WORKSPACE: s = scanWorkspace(); break;
        case State::CLASSIFY_COLORS: s = classify(); break;
        case State::SELECT_OBJECT: s = selectObject(); break;
        case State::LOCALIZE_OBJECT: s = localizeObject(); break;
        case State::CALCULATE_GRASP: s = calculateGrasp(); break;
        case State::EXECUTE_GRASP: s = executeGrasp(); break;
        case State::VERIFY_GRASP: s = verifyGrasp(); break;
        case State::TRANSPORT_OBJECT: s = transport(); break;
        case State::PLACE_OBJECT: s = place(); break;
        case State::VERIFY_PLACEMENT: s = verifyPlacement(); break;
        case State::MISSION_COMPLETE:
          ROS_INFO("=== MISSION COMPLETE: all %zu objects placed ===", slots_.size());
          groundTruthReport();
          return 0;
        case State::MISSION_FAILED:
          ROS_ERROR("=== MISSION FAILED: %s ===", fail_reason_.c_str());
          return 1;
      }
    }
    return 2;
  }

 private:
  // ------------------------------------------------------------------ parameters
  void loadParams() {
    std::vector<std::string> lc;
    std::vector<int> cnt;
    cfg_.getParam("level_colors", lc);
    cfg_.getParam("level_counts", cnt);
    std::vector<double> sc;
    cfg_.getParam("stack_center_xy", sc);
    if (lc.size() != cnt.size() || lc.empty() || sc.size() != 2) { ROS_FATAL("mission: bad level/stack parameters"); ros::shutdown(); return; }
    cfg_.param("slot_spacing", spacing_, 0.046);
    cfg_.param("piece_height", piece_h_, 0.074);
    cfg_.param("piece_center_z", piece_z_, 0.037);
    double az = std::atan2(sc[1], sc[0]);
    Vec3 tang{-std::sin(az), std::cos(az), 0};
    for (size_t l = 0; l < lc.size(); ++l)
      for (int i = 0; i < cnt[l]; ++i) {
        double off = (i - (cnt[l] - 1) / 2.0) * spacing_;
        Slot s;
        s.level = static_cast<int>(l);
        s.color = lc[l];
        s.center = Vec3{sc[0], sc[1], piece_z_ + l * piece_h_} + off * tang;
        slots_.push_back(s);
      }
    cfg_.getParam("scan_azimuths_deg", scan_az_);
    cfg_.getParam("scan_aim_radii", scan_r_);
    for (double r : scan_r_) for (double a : scan_az_) stations_.push_back({r, a});
    cfg_.getParam("scan_dist_scales", dist_scales_);  // camera-distance passes tried one after the other
    if (dist_scales_.empty()) dist_scales_ = {1.0, 0.85, 1.2};
    cfg_.param("blob_z_min", blob_z_min_, 0.037);  // blob must be above this height (floor ~0, dumbbell plate ~0.074)
    cfg_.param("blob_z_max", blob_z_max_, 0.12);
    cfg_.param("parallax_half_baseline", par_half_, 0.025);
    cfg_.param("reject_radius", reject_r_, 0.035);
    cfg_.param("scan_gate_radius", gate_r_, 0.07);
    cfg_.param("scan_pitch_deg", scan_pitch_, 80.0);
    cfg_.param("scan_distance", scan_dist_, 0.2);
    cfg_.param("scan_u_star", u_star_, 170.0);
    cfg_.param("observe_frames", frames_, 5);
    cfg_.param("merge_distance", merge_d_, 0.03);
    cfg_.param<std::string>("grasp_mode", grasp_mode_, "contact");
    cfg_.param("grasp_z_offset", grasp_dz_, -0.002);
    cfg_.param("carry_dz_contact", carry_contact_, 0.017);
    cfg_.param("carry_dz_weld", carry_weld_, -0.002);
    cfg_.param("pre_dist_max", pre_max_, 0.03);
    cfg_.param("min_level_radius", min_level_r_, 0.245);
    cfg_.param("standoff_back", back_, 0.05);
    cfg_.param("standoff_up", standoff_up_, 0.03);
    cfg_.getParam("standoff_tilt_candidates", tilts_);
    cfg_.param("lift_engage", lift_engage_, 0.03);
    cfg_.param("lift_height", lift_h_, 0.09);
    cfg_.param("use_ultrasonic", use_us_, true);
    cfg_.param("us_origin_to_ee", us_to_ee_, 0.058);
    cfg_.param("us_plate_half_extent", us_e_, 0.0217);
    cfg_.param("us_max_correction", us_maxc_, 0.015);
    cfg_.param("object_mass", mass_, 0.0411);
    cfg_.param("effort_fraction", eff_frac_, 0.4);
    cfg_.param("effort_min", eff_min_, 0.02);
    double go, gc;
    cfg_.param("gripper_open", go, 0.9);
    cfg_.param("gripper_closed", gc, 0.0);
    grip_.setAngles(go, gc);
    std::vector<double> vm;
    cfg_.getParam("joint_vmax", vm);
    for (int i = 0; i < 4; ++i) vmax_[i] = vm.size() == 4 ? vm[i] : 0.8;
    cfg_.param("cart_speed_fast", v_fast_, 0.10);
    cfg_.param("cart_speed_slow", v_slow_, 0.03);
    cfg_.param("cart_speed_final", v_final_, 0.015);
    cfg_.param("transport_min_z", tr_min_z_, 0.16);
    cfg_.param("transport_clearance", tr_clear_, 0.02);
    cfg_.param("hover_above_place", hover_, 0.02);
    cfg_.param("retract_dist", retract_, 0.05);
    cfg_.param("collision_margin", margin_, 0.002);
    cfg_.getParam("verify_views", verify_v_);
    cfg_.param("verify_tolerance", verify_tol_, 0.02);
    cfg_.param("max_pick_retries", max_retries_, 2);
    nh_.param("/am/sim/image_width", img_w_, 640);
    nh_.param("/am/sim/image_height", img_h_, 480);
    nh_.param("/am/sim/camera_fovy_deg", fovy_, 20.0);
    K_ = CameraIntrinsics::fromFovy(fovy_, img_w_, img_h_);
    carry_ = grasp_mode_ == "weld_assist" ? carry_weld_ : carry_contact_;
    ik_.pitch_tol = 0.01;
  }

  // ------------------------------------------------------------------ ROS callbacks
  void onDetections(const autonomous_manipulator::ObjectDetectionArrayConstPtr& m) {
    std::lock_guard<std::mutex> l(det_mtx_);
    dets_.push_back(*m);
    if (dets_.size() > 40) dets_.pop_front();
  }
  void onGroundTruth(const geometry_msgs::PoseArrayConstPtr& m) { std::lock_guard<std::mutex> l(gt_mtx_); gt_ = *m; }
  void onNames(const std_msgs::StringConstPtr& m) {
    std::lock_guard<std::mutex> l(gt_mtx_);
    names_.clear();
    std::stringstream ss(m->data);
    std::string t;
    while (std::getline(ss, t, ',')) names_.push_back(t);
  }
  void publishState(State s) {
    if (s == last_state_) return;
    last_state_ = s;
    std_msgs::String m;
    m.data = stateName(s);
    state_pub_.publish(m);
    ROS_INFO("[state] %s", stateName(s));
  }
  State fail(const std::string& why) { fail_reason_ = why; return State::MISSION_FAILED; }

  // ------------------------------------------------------------------ helpers: perception
  // Collects `frames` fresh detection messages (arm must be still) and clusters them per colour.
  std::vector<Det> observe(int frames, double timeout_s = 8.0) {
    ros::Time t0 = ros::Time::now();
    std::vector<autonomous_manipulator::ObjectDetectionArray> got;
    ros::Rate rate(50);
    while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout_s) {
      robot_.hold(0.02);
      {
        std::lock_guard<std::mutex> l(det_mtx_);
        got.clear();
        for (auto& m : dets_) if (m.header.stamp > t0) got.push_back(m);
      }
      if (static_cast<int>(got.size()) >= frames) break;
      rate.sleep();
    }
    std::vector<Det> cl;
    for (auto& m : got)
      for (auto& d : m.detections) {
        Vec3 p{d.position.x, d.position.y, d.position.z};
        Det* hit = nullptr;
        for (auto& c : cl) {
          Vec3 mean = (1.0 / c.n) * c.pos;
          if (c.color == d.color && std::hypot(mean.x - p.x, mean.y - p.y) < 0.02) { hit = &c; break; }
        }
        if (!hit) { cl.push_back(Det{d.color, {}, {}, {}, 0}); hit = &cl.back(); }
        hit->pos = hit->pos + p;
        hit->ray_o = hit->ray_o + Vec3{d.ray_origin.x, d.ray_origin.y, d.ray_origin.z};
        hit->ray_d = hit->ray_d + Vec3{d.ray_direction.x, d.ray_direction.y, d.ray_direction.z};
        hit->n++;
      }
    std::vector<Det> out;
    for (auto& c : cl) {
      if (c.n < std::max(2, frames / 2)) continue;  // not seen consistently
      double k = 1.0 / c.n;
      out.push_back(Det{c.color, k * c.pos, k * c.ray_o, normalized(c.ray_d), c.n});
    }
    return out;
  }

  // Camera pose that looks at P with the aim point at pixel column u_star (the left part of the image is not occluded).
  bool scanPose(const Vec3& P, double dist, IKResult& out, double pitch_deg = -1, double ustar = -1) {
    const double phi = (pitch_deg < 0 ? scan_pitch_ : pitch_deg) * M_PI / 180.0;
    const double al = std::atan(((ustar < 0 ? u_star_ : ustar) - K_.cx) / K_.fx);
    double h = std::atan2(P.y, P.x) + al;
    for (int it = 0; it < 8; ++it) {
      Vec3 f{std::cos(phi) * std::cos(h), std::cos(phi) * std::sin(h), -std::sin(phi)};
      Vec3 rgt{std::sin(h), -std::cos(h), 0}, yh{-std::sin(h), std::cos(h), 0};
      Vec3 d = normalized(f + std::tan(al) * rgt);
      Vec3 C = P - dist * d;
      Vec3 ee = C + (robot::kEeX - robot::kCamX) * f - robot::kCamY * yh;
      out = solveIK(ee, phi, ik_);
      if (!out.ok) return false;
      if (std::fabs(out.q[0] - h) < 1e-6) return true;
      h = out.q[0];
    }
    return out.ok;
  }

  // ------------------------------------------------------------------ helpers: motion
  PlanContext ctx(int held = -1, int target = -1, double finger = -1, double carry = -1) {
    PlanContext c;
    for (auto& t : tracked_) c.objects.push_back({t.color, t.pos});
    c.held = held;
    c.target = target;
    c.finger = finger < 0 ? grip_.openAngle() : finger;
    c.carry_dz = carry < 0 ? carry_ : carry;
    // seen-but-not-yet-picked objects are still obstacles (except look-alikes right at the target)
    for (auto& cd : cand_) {
      if (target >= 0 && target < static_cast<int>(tracked_.size()) && std::hypot(tracked_[target].pos.x - cd.pos.x, tracked_[target].pos.y - cd.pos.y) < 0.045) continue;
      c.objects.push_back({cd.color, cd.pos});
    }
    return c;
  }
  // Context for survey moves: the tracked list is still being built from noisy detections, and two overlapping
  // phantom objects would make the planner reject every move ("contact A <-> B") even though the arm is clear.
  // The scan poses hover well above the table, so only the robot vs. static world is checked.
  PlanContext scanCtx() {
    PlanContext c = ctx();
    c.objects.clear();
    return c;
  }
  bool execute(const JointTrajectory& tr, const PlanContext& c, const std::string& what) {
    std::string why;
    if (!planner_.validate(tr, c, &why)) { ROS_WARN("plan '%s' rejected by collision check: %s", what.c_str(), why.c_str()); return false; }
    if (!robot_.execute(tr)) return false;
    if (!robot_.waitArmSettled(0.01, 4.0)) ROS_WARN("'%s': arm did not settle within 4 s (continuing)", what.c_str());
    return true;
  }
  bool jointMove(const JointVec& q1, const PlanContext& c, const std::string& what) {
    JointVec q0 = robot_.armQ();
    double dur = TrajectoryPlanner::jointMoveDuration(q0, q1, vmax_, 0.5);
    return execute(TrajectoryPlanner::jointSpace(q0, q1, dur, 0.01), c, what);
  }
  bool moveTo(const Vec3& p, double pitch, const PlanContext& c, const std::string& what) {
    IKResult r = solveIK(p, pitch, ik_);
    if (!r.ok) { ROS_WARN("'%s': IK failed: %s", what.c_str(), r.reason.c_str()); return false; }
    return jointMove(r.q, c, what);
  }
  bool line(const Vec3& p1, double pitch1, double speed, const PlanContext& c, const std::string& what) {
    JointVec q0 = robot_.armQ();
    Pose e = forwardKinematics(q0);
    double pitch0 = std::asin(clampd(-e.R.col(0).z, -1.0, 1.0));
    double L = norm(p1 - e.p);
    double dur = std::max({0.5, 1.875 * L / speed, 1.875 * std::fabs(pitch1 - pitch0) / 0.8});
    JointTrajectory tr;
    std::string err;
    if (!TrajectoryPlanner::cartesianLine(e.p, p1, pitch0, pitch1, dur, 0.01, ik_, &q0, tr, err)) {
      ROS_WARN("line '%s' failed: %s", what.c_str(), err.c_str());
      return false;
    }
    return execute(tr, c, what);
  }
  double currentPitch() { return std::asin(clampd(-forwardKinematics(robot_.armQ()).R.col(0).z, -1.0, 1.0)); }

  // ------------------------------------------------------------------ states
  State initialization() {
    std::string path;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("model_path", path, "");
    if (!planner_.load(path, margin_)) return fail("planner could not load the MJCF");
    ROS_INFO("mission: %zu slots, grasp_mode=%s, carry_dz=%.3f", slots_.size(), grasp_mode_.c_str(), carry_);
    for (auto& s : slots_) ROS_INFO("  slot L%d %-6s (%.3f, %.3f, %.3f)", s.level, s.color.c_str(), s.center.x, s.center.y, s.center.z);
    // sanity: every slot must be reachable (level hand, hover and place heights)
    for (auto& s : slots_)
      for (double dz : {carry_ + hover_ + 0.05, grasp_dz_}) {  // transport-side hover height and release height
        Vec3 p{s.center.x, s.center.y, s.center.z + dz};
        IKResult r = solveIK(p, 0.0, ik_);
        if (!r.ok) return fail("stack slot unreachable: " + r.reason);
      }
    return State::WAIT_FOR_SIMULATION;
  }

  State waitForSimulation() {
    if (!robot_.waitForState(60.0)) return fail("no joint states / clock from mujoco_sim_node");
    ROS_INFO("mission: simulation is running");
    grip_.open();
    return State::SELECT_OBJECT;
  }

  int remaining(const std::string& color) const {
    int n = 0;
    for (auto& t : tracked_) n += (!t.placed && t.color == color);
    return n;
  }
  int needed(const std::string& color) const {
    int n = 0;
    for (auto& s : slots_) n += (!s.done && s.color == color);
    return n;
  }
  bool nearPlaced(const Vec3& p) const {
    for (auto& t : tracked_) if (t.placed && std::hypot(t.pos.x - p.x, t.pos.y - p.y) < 0.05) return true;
    return false;
  }

  // ---- candidate bookkeeping --------------------------------------------------------------------------------
  // Detections are first CANDIDATES. Only candidates that pass the height (parallax) check become tracked objects.
  bool isRejected(const Vec3& p) const {
    for (auto& r : rejected_) if (std::hypot(r.x - p.x, r.y - p.y) < reject_r_) return true;
    return false;
  }
  bool hasCandidate(const std::string& color) const {
    for (auto& c : cand_) if (c.color == color) return true;
    return false;
  }
  void addCandidate(const Det& d) {
    if (nearPlaced(d.pos) || isRejected(d.pos)) return;
    for (auto& t : tracked_)
      if (!t.placed && std::hypot(t.pos.x - d.pos.x, t.pos.y - d.pos.y) < merge_d_) return;  // already a verified object
    Cand* hit = nullptr;
    for (auto& c : cand_)
      if (c.color == d.color && std::hypot(c.pos.x - d.pos.x, c.pos.y - d.pos.y) < merge_d_) { hit = &c; break; }
    if (!hit) {
      for (auto& c : cand_)
        if (c.color != d.color && std::hypot(c.pos.x - d.pos.x, c.pos.y - d.pos.y) < merge_d_) {
          if (d.n > c.obs) { c.color = d.color; c.pos.x = d.pos.x; c.pos.y = d.pos.y; c.obs = d.n; }  // same blob, keep the better-observed label
          return;
        }
      cand_.push_back({d.color, {d.pos.x, d.pos.y, piece_z_}, 1});
      return;
    }
    hit->pos.x = (hit->pos.x * hit->obs + d.pos.x) / (hit->obs + 1);
    hit->pos.y = (hit->pos.y * hit->obs + d.pos.y) / (hit->obs + 1);
    hit->obs++;
  }

  // Depth check by motion parallax: look at the blob from two camera positions (+-parallax_half_baseline sideways)
  // and triangulate the two viewing rays. A dumbbell plate is ~0.074 m above the floor, a floor patch is at z ~ 0.
  // A plain white floor can therefore NOT pass this test just because it has the dumbbell's colour.
  HeightCheck verifyHeight(const Cand& c, double& z_est) {
    const double az = std::atan2(c.pos.y, c.pos.x);
    const Vec3 tang{-std::sin(az), std::cos(az), 0};
    Vec3 o[2], dir[2];
    bool got[2] = {false, false};
    for (int k = 0; k < 2; ++k) {
      const double sgn = k == 0 ? -1.0 : 1.0;
      Vec3 P{c.pos.x + sgn * par_half_ * tang.x, c.pos.y + sgn * par_half_ * tang.y, 0.074};
      IKResult pose;
      if (!scanPose(P, scan_dist_, pose) || !jointMove(pose.q, ctx(), "parallax view")) continue;
      double best = 0.04;
      const Det* b = nullptr;
      auto dets = observe(frames_ + 3);
      for (auto& d : dets)
        if (d.color == c.color) {
          double e = std::hypot(d.pos.x - c.pos.x, d.pos.y - c.pos.y);
          if (e < best) { best = e; b = &d; }
        }
      if (b) { o[k] = b->ray_o; dir[k] = normalized(b->ray_d); got[k] = true; }
    }
    if (!got[0] || !got[1]) return HeightCheck::UNKNOWN;
    auto dot = [](const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const Vec3 w0 = o[0] - o[1];
    const double b_ = dot(dir[0], dir[1]), d_ = dot(dir[0], w0), e_ = dot(dir[1], w0);
    const double den = 1.0 - b_ * b_;
    if (den < 1e-6) return HeightCheck::UNKNOWN;  // parallel rays: no depth information
    const double t0 = (b_ * e_ - d_) / den, t1 = (e_ - b_ * d_) / den;
    const Vec3 p0 = o[0] + t0 * dir[0], p1 = o[1] + t1 * dir[1];
    const double gap = norm(p0 - p1);
    z_est = 0.5 * (p0.z + p1.z);
    if (gap > 0.02) return HeightCheck::UNKNOWN;  // rays do not meet: the two views did not see the same blob
    const double zmin = blob_z_min_ - 0.006 * pass_;  // gate relaxes slightly in later scan passes
    return (z_est >= zmin && z_est <= blob_z_max_) ? HeightCheck::PASS : HeightCheck::FAIL;
  }

  // Scans (one station after the other, continuing where the last scan stopped) until a candidate of the colour
  // that is needed NOW has been seen. Other colours seen on the way are only remembered, not touched.
  // Each full cycle without success switches to the next pass: another camera distance and a wider gate.
  State scanWorkspace() {
    const std::string target = slots_[cur_slot_].color;
    const int npass = static_cast<int>(dist_scales_.size());
    if (stations_.empty() || npass == 0) return fail("no scan stations / distance scales configured");
    while (ros::ok()) {
      if (scanned_ >= static_cast<int>(stations_.size())) {
        scanned_ = 0;
        if (++pass_ >= npass) return fail("no valid '" + target + "' object found in any scan pass");
        ROS_WARN("scan cycle finished without a '%s' object: pass %d, camera distance x%.2f", target.c_str(), pass_ + 1, dist_scales_[pass_]);
      }
      const double scale = dist_scales_[pass_];
      const double gate = gate_r_ * (1.0 + 0.4 * pass_);  // accepted distance from the aim point (relative to the base)
      const double r = stations_[st_idx_].first, azd = stations_[st_idx_].second;
      st_idx_ = (st_idx_ + 1) % stations_.size();
      ++scanned_;
      const double az = azd * M_PI / 180.0;
      Vec3 P{r * std::cos(az), r * std::sin(az), 0.074};
      IKResult pose;
      if (!scanPose(P, scan_dist_ * scale, pose)) { ROS_WARN("scan station az=%.0f r=%.2f unreachable, skipped", azd, r); continue; }
      if (!jointMove(pose.q, scanCtx(), "scan move")) { ROS_WARN("scan station az=%.0f skipped (plan rejected)", azd); continue; }
      ROS_INFO("scan (az=%.0f, r=%.2f, dist x%.2f): searching for '%s'", azd, r, scale, target.c_str());
      for (auto& d : observe(frames_)) {
        if (std::hypot(d.pos.x - P.x, d.pos.y - P.y) > gate) continue;  // outside the range gate around the aim point
        addCandidate(d);
      }
      if (hasCandidate(target)) return State::CLASSIFY_COLORS;
      ROS_INFO("  no '%s' here, continuing the scan", target.c_str());
    }
    return State::MISSION_FAILED;
  }

  // Verifies the candidates of the needed colour (best first). The first one that is really above the floor becomes
  // an object; floor look-alikes are blacklisted and the scan continues.
  State classify() {
    const Slot& s = slots_[cur_slot_];
    while (true) {
      int bi = -1;
      double best = 1e9;
      for (size_t i = 0; i < cand_.size(); ++i) {
        if (cand_[i].color != s.color) continue;
        double d = std::hypot(cand_[i].pos.x - s.center.x, cand_[i].pos.y - s.center.y);
        if (d < best) { best = d; bi = static_cast<int>(i); }
      }
      if (bi < 0) return State::SCAN_WORKSPACE;  // nothing (left) -> keep scanning for this colour
      Cand c = cand_[bi];
      cand_.erase(cand_.begin() + bi);
      double z = 0;
      HeightCheck h = verifyHeight(c, z);
      if (h == HeightCheck::FAIL) {
        rejected_.push_back(c.pos);
        ROS_WARN("candidate '%s' at (%.3f, %.3f): height %.3f m is floor level -> rejected", c.color.c_str(), c.pos.x, c.pos.y, z);
        continue;
      }
      if (h == HeightCheck::PASS) ROS_INFO("candidate '%s' at (%.3f, %.3f): height %.3f m -> real object", c.color.c_str(), c.pos.x, c.pos.y, z);
      else ROS_WARN("candidate '%s' at (%.3f, %.3f): parallax inconclusive, accepted (ultrasonic check follows at the grasp)", c.color.c_str(), c.pos.x, c.pos.y);
      tracked_.push_back({c.color, {c.pos.x, c.pos.y, piece_z_}, c.obs, false});
      pass_ = 0;
      scanned_ = 0;
      return State::SELECT_OBJECT;
    }
  }

  State selectObject() {
    cur_slot_ = -1;
    for (size_t i = 0; i < slots_.size(); ++i) if (!slots_[i].done) { cur_slot_ = static_cast<int>(i); break; }
    if (cur_slot_ < 0) return State::MISSION_COMPLETE;
    const Slot& s = slots_[cur_slot_];
    cur_obj_ = -1;
    double best = 1e9;
    for (size_t i = 0; i < tracked_.size(); ++i) {
      if (tracked_[i].placed || tracked_[i].color != s.color) continue;
      double d = std::hypot(tracked_[i].pos.x - s.center.x, tracked_[i].pos.y - s.center.y);
      if (d < best) { best = d; cur_obj_ = static_cast<int>(i); }
    }
    if (cur_obj_ < 0) {  // none known yet: use remembered candidates, otherwise scan on until this colour appears
      ROS_INFO("select: slot %d needs '%s' (level %d): none verified yet", cur_slot_, s.color.c_str(), s.level + 1);
      return hasCandidate(s.color) ? State::CLASSIFY_COLORS : State::SCAN_WORKSPACE;
    }
    ROS_INFO("select: slot %d (level %d, %s) <- object %d at (%.3f, %.3f)", cur_slot_, s.level + 1, s.color.c_str(), cur_obj_,
             tracked_[cur_obj_].pos.x, tracked_[cur_obj_].pos.y);
    return State::LOCALIZE_OBJECT;
  }

  // Re-observe the selected object from directly above and refine its position estimate.
  State localizeObject() {
    Tracked& o = tracked_[cur_obj_];
    IKResult pose;
    if (!scanPose({o.pos.x, o.pos.y, 0.074}, scan_dist_, pose) || !jointMove(pose.q, ctx(), "localize move"))
      return pickFailed("cannot reach a viewpoint above the object");
    auto dets = observe(frames_ + 3);
    double best = 0.05;
    const Det* b = nullptr;
    for (auto& d : dets)
      if (d.color == o.color) {
        double e = std::hypot(d.pos.x - o.pos.x, d.pos.y - o.pos.y);
        if (e < best) { best = e; b = &d; }
      }
    if (!b) return pickFailed("object not seen at its expected position");
    ROS_INFO("localize: refined (%.4f,%.4f) -> (%.4f,%.4f)", o.pos.x, o.pos.y, b->pos.x, b->pos.y);
    o.pos.x = b->pos.x;
    o.pos.y = b->pos.y;
    return State::CALCULATE_GRASP;
  }

  State calculateGrasp() {
    const Tracked& o = tracked_[cur_obj_];
    Vec3 T{o.pos.x, o.pos.y, piece_z_ + grasp_dz_};
    IKResult r = solveIK(T, 0.0, ik_);
    if (!r.ok) return pickFailed("grasp pose unreachable with a level hand: " + r.reason);
    plan_.hip = r.q[0];
    plan_.XT = T.x * std::cos(plan_.hip) + T.y * std::sin(plan_.hip);
    plan_.z = T.z;
    plan_.d_pre = clampd(plan_.XT - min_level_r_, 0.0, pre_max_);
    for (double tilt : tilts_) {
      Vec3 P = eeOnPlane(plan_.hip, plan_.XT - plan_.d_pre, plan_.z);
      Vec3 S = eeOnPlane(plan_.hip, plan_.XT - plan_.d_pre - back_, plan_.z + 0.01);
      Vec3 Q = S + Vec3{0, 0, standoff_up_};
      if (solveIK(P, 0.0, ik_).ok && solveIK(S, tilt, ik_).ok && solveIK(Q, tilt, ik_).ok) { plan_.tilt = tilt; ROS_INFO("grasp plan: hip=%.3f XT=%.3f d_pre=%.3f tilt=%.2f", plan_.hip, plan_.XT, plan_.d_pre, tilt); return State::EXECUTE_GRASP; }
    }
    return pickFailed("no reachable standoff / pre-grasp pose for this object");
  }

  State executeGrasp() {
    grip_.open();
    const int idx = cur_obj_;
    const double h = plan_.hip, z = plan_.z;
    Vec3 P = eeOnPlane(h, plan_.XT - plan_.d_pre, z);
    Vec3 S = eeOnPlane(h, plan_.XT - plan_.d_pre - back_, z + 0.01);
    Vec3 Q = S + Vec3{0, 0, standoff_up_};
    PlanContext open_ctx = ctx(-1, idx, grip_.openAngle());
    // 1-2: safe pre-grasp (above and behind), then approach down to the standoff with the hand pitched
    if (!moveTo(Q, plan_.tilt, open_ctx, "transit to pre-grasp")) return pickFailed("transit to pre-grasp");
    if (!line(S, plan_.tilt, v_fast_, open_ctx, "descend to standoff")) return pickFailed("descend to standoff");
    // 3: level the hand while sliding to the level pre-grasp pose (jaw plane inside the 50 mm gap between the plates)
    if (!line(P, 0.0, v_slow_, open_ctx, "level pre-grasp")) return pickFailed("level pre-grasp");
    robot_.hold(0.6);
    double XT = plan_.XT;
    if (use_us_) {  // ultrasonic: the beam (at plate height) must hit the top plate; its range corrects the insertion depth
      double sum = 0;
      int n = 0;
      ros::Rate rate(100);
      ros::Time t0 = ros::Time::now();
      while (ros::ok() && (ros::Time::now() - t0).toSec() < 0.4) {
        double rr;
        robot_.hold(0.0);
        if (robot_.range(rr)) { sum += rr; ++n; }
        robot_.hold(0.01);
        rate.sleep();
      }
      if (n < 20 && retries_ >= 1) rejected_.push_back(tracked_[idx].pos);  // no echo twice: it is not a standing object
      if (n < 20) return pickFailed("ultrasonic: no echo from the dumbbell plate (lateral/radial estimate off)");
      double rm = sum / n;
      double XTn = (plan_.XT - plan_.d_pre) - us_to_ee_ + rm + us_e_;
      ROS_INFO("ultrasonic: range %.4f m -> bar-centre X %.4f (camera estimate %.4f)", rm, XTn, plan_.XT);
      if (std::fabs(XTn - plan_.XT) > us_maxc_) return pickFailed("ultrasonic range inconsistent with the camera estimate");
      XT = XTn;
    }
    // 4: final approach: the bar slides into the open mouth of the ring
    Vec3 T = eeOnPlane(h, XT, z);
    if (!line(T, 0.0, v_final_, open_ctx, "final approach")) return pickFailed("final approach");
    effort0_ = robot_.meanEffort(0.4);
    // 5: close
    grip_.close();
    robot_.hold(0.3);
    PlanContext target_ctx = ctx(-1, idx, grip_.openAngle() * 0.0);
    if (grasp_mode_ == "weld_assist") {
      std::string msg;
      if (!robot_.setWeld(true, msg)) return pickFailed("weld assist failed: " + msg);
      ROS_WARN("SIMULATION-ASSISTED GRASP (weld constraint): %s", msg.c_str());
    }
    // 6: lift - first the small 'engage' lift in which the ring hooks under the top plate, then the full lift
    grasp_T_ = T;
    if (!line(T + Vec3{0, 0, lift_engage_}, 0.0, v_final_, target_ctx, "engage lift")) return pickFailed("engage lift");
    robot_.hold(0.4);
    PlanContext held_ctx = ctx(idx, -1, 0.0, carry_);
    bool lifted = false;  // lift as high as the (shoulder-limited) workspace allows near the base
    for (double lh : {lift_h_, lift_h_ - 0.02, lift_h_ - 0.04}) {
      if (line(T + Vec3{0, 0, lh}, 0.0, v_slow_, held_ctx, "lift")) { lifted = true; break; }
    }
    if (!lifted) return pickFailed("lift");
    return State::VERIFY_GRASP;
  }

  // Physical verification: after the lift the shoulder motor must carry the object's weight (measured actuator torque).
  State verifyGrasp() {
    robot_.hold(0.5);
    JointVec e1 = robot_.meanEffort(0.5);
    double X = std::hypot(grasp_T_.x, grasp_T_.y);
    double expected = mass_ * 9.81 * std::max(0.05, X - robot::kShoulderX);
    double delta = std::fabs(e1[1] - effort0_[1]);
    double thr = std::max(eff_min_, eff_frac_ * expected);
    ROS_INFO("verify grasp: shoulder torque change %.4f N m (expected ~%.4f, threshold %.4f)", delta, expected, thr);
    if (delta < thr) {
      ROS_ERROR("grasp NOT verified: the object is not hanging on the gripper");
      std::string m;
      if (grasp_mode_ == "weld_assist") robot_.setWeld(false, m);
      return pickFailed("grasp verification failed");
    }
    retries_ = 0;
    return State::TRANSPORT_OBJECT;
  }

  double obstacleTop() const {
    double top = 0;
    for (size_t i = 0; i < tracked_.size(); ++i)
      if (static_cast<int>(i) != cur_obj_) top = std::max(top, tracked_[i].pos.z + 0.037);
    return top;
  }

  State transport() {
    const Slot& s = slots_[cur_slot_];
    PlanContext held = ctx(cur_obj_, -1, 0.0, carry_);
    double ztr = std::max(tr_min_z_, obstacleTop() + tr_clear_ + carry_ + 0.037);
    bool ok = false;
    for (double extra : {0.0, 0.03, 0.06}) {
      double z = ztr + extra;
      Pose e = forwardKinematics(robot_.armQ());
      if (e.p.z < z - 1e-3) line({e.p.x, e.p.y, z}, 0.0, v_slow_, held, "raise to transport height");  // may be impossible close to the base; the joint move below then raises while moving outward
      if (!moveTo({s.center.x, s.center.y, z}, 0.0, held, "transport")) continue;
      ok = true;
      ztr = z;
      break;
    }
    if (!ok) return fail("transport path blocked / unreachable (collision check)");
    double zh = s.center.z + carry_ + hover_;
    if (!line({s.center.x, s.center.y, zh}, 0.0, v_slow_, held, "lower to hover")) return fail("cannot lower above the stack slot");
    transport_z_ = ztr;
    return State::PLACE_OBJECT;
  }

  State place() {
    const Slot& s = slots_[cur_slot_];
    PlanContext held = ctx(cur_obj_, -1, 0.0, carry_);
    double z_touch = s.center.z + carry_;
    double z_rel = grasp_mode_ == "weld_assist" ? z_touch : s.center.z + grasp_dz_;
    if (!line({s.center.x, s.center.y, z_touch}, 0.0, v_final_, held, "lower onto support")) return fail("cannot lower onto support");
    if (z_rel < z_touch - 1e-4 && !line({s.center.x, s.center.y, z_rel}, 0.0, 0.01, held, "slide ring below the plate")) return fail("release descent failed");
    robot_.hold(0.3);
    std::string msg;
    if (grasp_mode_ == "weld_assist") robot_.setWeld(false, msg);
    grip_.open();
    robot_.hold(0.3);
    Tracked& o = tracked_[cur_obj_];
    o.pos = s.center;
    o.placed = true;
    // retract along the hand axis (jaws open, ring leaves the bar through its mouth), then rise
    double h = robot_.armQ()[0];
    Pose e = forwardKinematics(robot_.armQ());
    PlanContext free_ctx = ctx(-1, -1, grip_.openAngle());
    Vec3 back = e.p + Vec3{-retract_ * std::cos(h), -retract_ * std::sin(h), 0};
    if (!line(back, 0.0, v_slow_, free_ctx, "retract")) return fail("retract blocked after release");
    if (!line(back + Vec3{0, 0, 0.08}, 0.0, v_slow_, free_ctx, "rise after release")) ROS_WARN("rise after release failed (continuing)");
    return State::VERIFY_PLACEMENT;
  }

  State verifyPlacement() {
    Slot& s = slots_[cur_slot_];
    double ztop = s.center.z + 0.037;
    for (size_t vi = 0; vi + 2 < verify_v_.size(); vi += 3) {
      IKResult pose;
      if (!scanPose({s.center.x, s.center.y, ztop}, verify_v_[vi + 1], pose, verify_v_[vi], verify_v_[vi + 2])) continue;
      if (!jointMove(pose.q, ctx(), "verification view")) continue;
      double best = 1e9;
      for (auto& d : observe(frames_ + 3)) {
        if (d.color != s.color) continue;
        Vec3 hit;
        if (!rayPlane(d.ray_o, d.ray_d, ztop, hit)) continue;
        best = std::min(best, std::hypot(hit.x - s.center.x, hit.y - s.center.y));
      }
      ROS_INFO("verify placement: nearest '%s' blob %.4f m from the slot centre (tolerance %.3f)", s.color.c_str(), best, verify_tol_);
      if (best <= verify_tol_) {
        s.done = true;
        ROS_INFO("placement verified: slot %d done (%d/%zu)", cur_slot_, cur_slot_ + 1, slots_.size());
        return State::SELECT_OBJECT;
      }
      return fail("placement not confirmed by the camera (object missing / displaced)");
    }
    return fail("no reachable viewpoint to verify the placement");
  }

  // Failed pick: release, retreat upwards, survey again and retry (bounded). Failed attempts never continue the mission.
  State pickFailed(const std::string& why) {
    ROS_ERROR("pick failed: %s (attempt %d/%d)", why.c_str(), retries_ + 1, max_retries_ + 1);
    std::string m;
    if (grasp_mode_ == "weld_assist") robot_.setWeld(false, m);
    grip_.open();
    Pose e = forwardKinematics(robot_.armQ());
    double h = robot_.armQ()[0];
    Vec3 back = e.p + Vec3{-0.05 * std::cos(h), -0.05 * std::sin(h), 0.0};
    PlanContext c = ctx(-1, -1, grip_.openAngle());
    line(back, 0.0, v_slow_, c, "retreat");
    line(back + Vec3{0, 0, 0.10}, 0.0, v_slow_, c, "retreat up");
    tracked_.erase(std::remove_if(tracked_.begin(), tracked_.end(), [](const Tracked& t) { return !t.placed; }), tracked_.end());
    cand_.clear();
    st_idx_ = 0; scanned_ = 0; pass_ = 0;  // fresh survey for the colour that is needed now
    if (++retries_ > max_retries_) return fail("pick failed " + std::to_string(retries_) + " times: " + why);
    return State::SCAN_WORKSPACE;
  }

  // ------------------------------------------------------------------ validation only
  void groundTruthReport() {
    std::lock_guard<std::mutex> l(gt_mtx_);
    if (gt_.poses.empty() || names_.size() != gt_.poses.size()) { ROS_INFO("validation: simulator ground truth not available"); return; }
    ROS_INFO("--- validation report (simulator ground truth, NOT used by the mission) ---");
    for (size_t i = 0; i < names_.size(); ++i) {
      const auto& p = gt_.poses[i].position;
      std::string col = names_[i].substr(names_[i].rfind('_') + 1);
      double best = 1e9;
      const Slot* bs = nullptr;
      for (auto& s : slots_)
        if (s.color == col) {
          double d = std::hypot(p.x - s.center.x, p.y - s.center.y) + std::fabs(p.z - s.center.z);
          if (d < best) { best = d; bs = &s; }
        }
      ROS_INFO("  %-18s at (%.3f, %.3f, %.3f)  nearest slot L%d offset %.4f m", names_[i].c_str(), p.x, p.y, p.z, bs ? bs->level + 1 : 0, best);
    }
  }

  // ------------------------------------------------------------------ members
  ros::NodeHandle nh_, cfg_;
  RobotInterface robot_;
  GripperController grip_;
  MotionPlanner planner_;
  IKOptions ik_;
  CameraIntrinsics K_;
  ros::Publisher state_pub_;
  ros::Subscriber det_sub_, gt_sub_, names_sub_;
  std::mutex det_mtx_, gt_mtx_;
  std::deque<autonomous_manipulator::ObjectDetectionArray> dets_;
  geometry_msgs::PoseArray gt_;
  std::vector<std::string> names_;
  std::vector<Tracked> tracked_;
  std::vector<Cand> cand_;
  std::vector<Vec3> rejected_;
  std::vector<std::pair<double, double>> stations_;  // (aim radius, azimuth deg)
  std::vector<double> dist_scales_;
  size_t st_idx_ = 0;
  int scanned_ = 0, pass_ = 0;
  double blob_z_min_ = 0.037, blob_z_max_ = 0.12, par_half_ = 0.025, reject_r_ = 0.035, gate_r_ = 0.07;
  std::vector<Slot> slots_;
  State last_state_ = State::MISSION_FAILED;
  std::string fail_reason_, grasp_mode_;
  GraspPlan plan_;
  JointVec effort0_{};
  Vec3 grasp_T_;
  int cur_slot_ = -1, cur_obj_ = -1, retries_ = 0, max_retries_ = 2, frames_ = 5, img_w_, img_h_;
  double spacing_, piece_h_, piece_z_, scan_pitch_, scan_dist_, u_star_, merge_d_, grasp_dz_, carry_contact_, carry_weld_,
      carry_ = 0.017, standoff_up_ = 0.03, pre_max_, min_level_r_, back_, lift_engage_, lift_h_, us_to_ee_, us_e_, us_maxc_, mass_, eff_frac_,
      eff_min_, v_fast_, v_slow_, v_final_, tr_min_z_, tr_clear_, hover_, retract_, margin_, verify_tol_, fovy_,
      transport_z_ = 0;
  bool use_us_ = true;
  JointVec vmax_{};
  std::vector<double> scan_az_, scan_r_, tilts_, verify_v_;  // verify_v_: flat list of (pitch_deg, distance, u_star) triples
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_controller");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  Mission m(nh);
  int rc = m.run();
  ros::shutdown();
  return rc;
}
