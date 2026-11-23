// color_view_node.cpp - ROS wrapper of the single-colour view search (Phase 4b).
//
//   service     /perception/validate_color (ValidateColor)
//   uses        /motion/move_joints (MoveJoints, motion_node) to move the arm, /camera/image_raw for the frames
//   subscribes  /arm/joint_states (seed of the planner), /camera/image_raw
// The camera does not locate anything: the caller gives the dumbbell position. The search logic is the pure C++
// library arm_core (view_planner, view_search, color_classifier).
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include "autonomous_manipulator/MoveJoints.h"
#include "autonomous_manipulator/ValidateColor.h"
#include "autonomous_manipulator/view_search.hpp"

using namespace arm;

namespace {
const char* kJointNames[kArmDof] = {"joint_1", "joint_2", "joint_3", "joint_4"};

class ColorViewNode {
 public:
  ColorViewNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : model_(RobotModel::loadFromFile(param<std::string>(pnh, "robot_model_path", "config/robot_model.txt"))),
        kin_(model_),
        chk_(model_, CollisionOptions::withMargin(param(pnh, "collision_margin", 0.003))),
        ik_(kin_, &chk_),
        planner_(kin_, chk_, ik_) {
    cfg_ = ColorConfig::simulationDefaults();  // SIMULATION thresholds; the hardware camera needs its own calibration
    cfg_.min_fraction = param(pnh, "min_fraction", cfg_.min_fraction);
    cfg_.max_second_ratio = param(pnh, "max_second_ratio", cfg_.max_second_ratio);
    cfg_.roi_x0 = param(pnh, "roi_x0", 0.0);
    cfg_.roi_x1 = param(pnh, "roi_x1", 1.0);
    cfg_.roi_y0 = param(pnh, "roi_y0", 0.0);
    cfg_.roi_y1 = param(pnh, "roi_y1", 1.0);
    sp_.pitch_deg = param(pnh, "pitch_deg", sp_.pitch_deg);
    sp_.max_attempts = param(pnh, "max_attempts", sp_.max_attempts);
    pnh.getParam("standoffs", sp_.standoffs);
    pnh.getParam("lateral_steps", sp_.lateral_steps);
    pnh.getParam("steeper_pitches_deg", sp_.steeper_pitches_deg);
    sp_.steeper_standoff = param(pnh, "steeper_standoff", sp_.steeper_standoff);
    settle_wait_ = param(pnh, "frame_wait", 0.4);  // [s] pause after a move before a frame is accepted

    img_sub_ = nh.subscribe(param<std::string>(pnh, "image_topic", "/camera/image_raw"), 1, &ColorViewNode::onImage, this);
    js_sub_ = nh.subscribe(param<std::string>(pnh, "joint_states_topic", "/arm/joint_states"), 10, &ColorViewNode::onJoints, this);
    move_ = nh.serviceClient<autonomous_manipulator::MoveJoints>("/motion/move_joints");
    srv_ = nh.advertiseService("/perception/validate_color", &ColorViewNode::onValidate, this);
    ROS_INFO("color_view_node ready (%zu colour classes)", cfg_.classes.size());
  }

 private:
  template <class T>
  static T param(ros::NodeHandle& n, const std::string& key, const T& def) {
    T v = def;
    n.param(key, v, def);
    return v;
  }

  void onJoints(const sensor_msgs::JointState::ConstPtr& m) {
    std::lock_guard<std::mutex> l(mu_);
    for (size_t k = 0; k < m->name.size() && k < m->position.size(); ++k)
      for (int i = 0; i < kArmDof; ++i)
        if (m->name[k] == kJointNames[i]) {
          q_[i] = m->position[k];
          if (i == kArmDof - 1) have_q_ = true;
        }
  }

  void onImage(const sensor_msgs::Image::ConstPtr& m) {
    std::lock_guard<std::mutex> l(mu_);
    if (m->width == 0 || m->height == 0 || m->step < m->width * 3 || m->data.size() < static_cast<size_t>(m->step) * m->height) return;
    const bool bgr = m->encoding == "bgr8";
    if (!bgr && m->encoding != "rgb8") return;
    rgb_.resize(static_cast<size_t>(m->width) * m->height * 3);
    for (uint32_t y = 0; y < m->height; ++y) {
      const uint8_t* src = &m->data[static_cast<size_t>(y) * m->step];
      uint8_t* dst = &rgb_[static_cast<size_t>(y) * m->width * 3];
      for (uint32_t x = 0; x < m->width; ++x) {
        dst[3 * x] = src[3 * x + (bgr ? 2 : 0)];
        dst[3 * x + 1] = src[3 * x + 1];
        dst[3 * x + 2] = src[3 * x + (bgr ? 0 : 2)];
      }
    }
    img_w_ = static_cast<int>(m->width);
    img_h_ = static_cast<int>(m->height);
    ++img_seq_;
  }

  // Moves the arm to the view and classifies a frame taken AFTER the move.
  bool evaluate(const ViewPose& v, ColorResult& r, bool dry_run) {
    if (dry_run) {
      r = ColorResult();
      return true;
    }
    autonomous_manipulator::MoveJoints srv;
    for (int i = 0; i < kArmDof; ++i) {
      srv.request.name.push_back(kJointNames[i]);
      srv.request.position.push_back(v.q[i]);
    }
    if (!move_.call(srv) || !srv.response.success) {
      ROS_WARN("move to the view failed: %s", move_.isValid() ? srv.response.message.c_str() : "service call failed");
      return false;
    }
    ros::WallDuration(settle_wait_).sleep();
    unsigned long seq0;
    {
      std::lock_guard<std::mutex> l(mu_);
      seq0 = img_seq_;
    }
    const ros::WallTime t0 = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now() - t0).toSec() < 3.0) {  // wait for a frame newer than the end of the move
      {
        std::lock_guard<std::mutex> l(mu_);
        if (img_seq_ >= seq0 + 2 && !rgb_.empty()) {  // two new frames: the first may have been exposed before the arm stopped
          r = classifyColor(rgb_.data(), img_w_, img_h_, cfg_);
          return true;
        }
      }
      ros::WallDuration(0.02).sleep();
    }
    ROS_WARN("no camera frame after the move");
    return false;
  }

  bool onValidate(autonomous_manipulator::ValidateColor::Request& req, autonomous_manipulator::ValidateColor::Response& res) {
    std::unique_lock<std::mutex> busy(busy_, std::try_to_lock);
    if (!busy.owns_lock()) {
      res.status = "BUSY";
      res.message = "another colour validation is running";
      return true;
    }
    JointVec seed;
    {
      std::lock_guard<std::mutex> l(mu_);
      if (!have_q_) {
        res.status = "UNREACHABLE";
        res.message = "no /arm/joint_states yet";
        return true;
      }
      seed = q_;
    }
    const Vec3 target{req.target.x, req.target.y, req.target.z};
    const bool dry = req.dry_run;
    const ViewSearchResult sr = searchSingleColorView(planner_, target, seed, [&](const ViewPose& v, ColorResult& r) { return evaluate(v, r, dry); }, sp_);
    static const char* names[] = {"UNIQUE", "AMBIGUOUS", "NO_OBJECT", "UNREACHABLE"};
    res.status = names[static_cast<int>(sr.status)];
    res.success = sr.status == SearchStatus::Unique;
    res.color = sr.color;
    res.confidence = static_cast<float>(sr.confidence);
    res.attempts = sr.attempts;
    for (double q : sr.view.q) res.q_view.push_back(static_cast<float>(q));
    res.trace = sr.trace;
    res.message = dry ? "dry run: only the first view was planned" : (res.success ? "ok" : "no single colour found");
    return true;
  }

  RobotModel model_;
  Kinematics kin_;
  CollisionChecker chk_;
  InverseKinematics ik_;
  ViewPlanner planner_;
  ColorConfig cfg_;
  ViewSearchParams sp_;
  double settle_wait_ = 0.4;
  ros::Subscriber img_sub_, js_sub_;
  ros::ServiceClient move_;
  ros::ServiceServer srv_;
  std::mutex mu_, busy_;
  JointVec q_{};
  bool have_q_ = false;
  std::vector<uint8_t> rgb_;
  int img_w_ = 0, img_h_ = 0;
  unsigned long img_seq_ = 0;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_view_node");
  ros::NodeHandle nh, pnh("~");
  try {
    ColorViewNode node(nh, pnh);
    ros::AsyncSpinner spinner(3);  // the service blocks while it moves the arm; image / joint callbacks need their own threads
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("color_view_node: %s", e.what());
    return 1;
  }
  return 0;
}
