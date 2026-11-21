/**
 * mujoco_sim_node.cpp  -  MuJoCo simulation backend (Phase 2)
 *
 * OWNERSHIP / THREADING
 *   This node is the ONLY owner of mjModel / mjData. Everything runs in ONE thread:
 *   ros::spinOnce() (command callbacks) -> mj_step() -> publish -> render. There is
 *   no concurrent access to mjData. Other nodes (mission, planners, ...) talk to the
 *   simulation only through ROS topics/services.
 *   (Phase 4 will add offscreen camera rendering in this same loop, because the OpenGL
 *   context belongs to this thread.)
 *
 * INTERFACE
 *   publishes   <joint_states_topic>  (default /arm/joint_states)  sensor_msgs/JointState
 *               position/velocity = MEASURED simulated state of every hinge/slide joint
 *               effort            = actuator force (0 for joints without actuator)
 *   subscribes  <joint_command_topic> (default /arm/joint_command) sensor_msgs/JointState
 *               name[i] -> position[i] = position target for the actuator of that joint.
 *               Targets are clamped to the actuator ctrlrange. Unknown joints and joints
 *               without an actuator (finger_passive) are ignored with a warning.
 *   service     /sim/reset (std_srvs/Empty): back to keyframe <keyframe>.
 *   service     /sim/contact_report (std_srvs/Trigger): JSON in `message` listing every contact
 *               that involved a robot geom since the previous call (checked after EVERY mj_step),
 *               then clears the log. Per pair: bodies, geoms, min_dist [m] (negative = penetration),
 *               count [steps], kind ("self" robot-robot, "env" robot-environment).
 *               Resting contact base_link <-> world is ignored. MuJoCo never generates contacts
 *               between a body and its direct parent, so those pairs cannot appear here.
 *
 * PERCEPTION INTERFACE (Phase 4; all rendering happens in this thread, GL context owner)
 *   publishes   /camera/image_raw    sensor_msgs/Image   rgb8, <camera_width> x <camera_height>, from MJCF camera
 *                                    <camera_name>; sites and the rangefinder ray are NOT drawn into it
 *               /camera/camera_info  sensor_msgs/CameraInfo  pinhole K derived from the camera's fovy
 *               /camera/pose         geometry_msgs/PoseStamped  OPTICAL frame (x right, y down, z forward)
 *                                    in world; published just before the image with the same stamp
 *               /hcsr04/range        sensor_msgs/Range   MuJoCo rangefinder; +inf = no echo, -inf = too close
 *               /hcsr04/pose         geometry_msgs/PoseStamped  sensor site frame in world (+z = sensing axis)
 *   A hidden window is created when render:=false so the camera still works (needs an X display).
 *   Viewer shows the camera as an inset (key C toggles it).
 *
 * VIEWER KEYS: Space = pause/resume, Backspace = reset, Esc = quit, C = camera inset on/off,
 *              left drag = rotate, right drag = pan, Shift+drag = other plane, wheel = zoom.
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <map>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

class MujocoSimNode {
 public:
  MujocoSimNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string js_topic, cmd_topic;
    if (!pnh.getParam("model_path", model_path_)) {
      throw std::runtime_error("private parameter ~model_path is required");
    }
    pnh.param<std::string>("keyframe", keyframe_, "home");
    pnh.param<std::string>("joint_states_topic", js_topic, "/arm/joint_states");
    pnh.param<std::string>("joint_command_topic", cmd_topic, "/arm/joint_command");
    pnh.param("realtime_factor", rtf_, 1.0);
    pnh.param("joint_states_rate", joint_rate_, 100.0);
    pnh.param("render", render_, true);
    pnh.param("render_rate", render_rate_, 60.0);
    pnh.param("start_paused", paused_, false);
    pnh.param("camera_enabled", camera_enabled_, true);
    pnh.param<std::string>("camera_name", camera_name_, "esp32cam");
    pnh.param("camera_width", cam_w_, 640);
    pnh.param("camera_height", cam_h_, 480);
    pnh.param("camera_rate", camera_rate_, 15.0);
    pnh.param("camera_preview", preview_, true);
    pnh.param<std::string>("range_sensor", range_name_, "hcsr04_range");
    pnh.param("range_rate", range_rate_, 20.0);
    pnh.param("range_noise_std", range_noise_, 0.0);
    if (camera_rate_ <= 0.0 || range_rate_ <= 0.0 || cam_w_ <= 0 || cam_h_ <= 0) {
      throw std::runtime_error("camera_rate, range_rate, camera_width and camera_height must be > 0");
    }
    if (rtf_ <= 0.0 || joint_rate_ <= 0.0 || render_rate_ <= 0.0) {
      throw std::runtime_error("realtime_factor, joint_states_rate and render_rate must be > 0");
    }

    loadModel();
    buildJointTable();
    buildContactMonitor(pnh);
    resetState();

    joint_pub_ = nh.advertise<sensor_msgs::JointState>(js_topic, 10);
    cmd_sub_ = nh.subscribe(cmd_topic, 10, &MujocoSimNode::onCommand, this);
    reset_srv_ = nh.advertiseService("/sim/reset", &MujocoSimNode::onReset, this);
    contact_srv_ = nh.advertiseService("/sim/contact_report", &MujocoSimNode::onContactReport, this);

    setupSensors(nh, pnh);
    if ((render_ || camera_enabled_) && !initViewer()) {
      ROS_WARN("Viewer/GL context could not be created: no window and NO CAMERA images (range still works).");
      render_ = false;
      camera_enabled_ = false;
    }
    ROS_INFO("mujoco_sim ready: state -> %s, commands <- %s", js_topic.c_str(), cmd_topic.c_str());
  }

  ~MujocoSimNode() {
    shutdownViewer();
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  void run() {
    ros::WallTime wall_ref = ros::WallTime::now();
    double sim_ref = data_->time;
    ros::WallTime next_pub = wall_ref;
    ros::WallTime next_render = wall_ref;
    ros::WallTime next_cam = wall_ref;
    ros::WallTime next_range = wall_ref;
    const ros::WallDuration pub_period(1.0 / joint_rate_);
    const ros::WallDuration render_period(1.0 / render_rate_);
    const ros::WallDuration cam_period(1.0 / camera_rate_);
    const ros::WallDuration range_period(1.0 / range_rate_);
    const int kMaxStepsPerIter = 100;

    while (ros::ok()) {
      ros::spinOnce();  // runs onCommand / onReset in THIS thread

      if (need_resync_) {
        wall_ref = ros::WallTime::now();
        sim_ref = data_->time;
        need_resync_ = false;
      }
      if (!paused_) {
        const double target = sim_ref + rtf_ * (ros::WallTime::now() - wall_ref).toSec();
        int steps = 0;
        while (data_->time < target && steps < kMaxStepsPerIter) {
          mj_step(model_, data_);
          recordContacts();
          ++steps;
        }
        if (data_->time < target - 0.1) {  // cannot keep real time: drop the backlog
          need_resync_ = true;
        }
      }

      const ros::WallTime now = ros::WallTime::now();
      if (now >= next_pub) {
        publishJointStates();
        next_pub = (now - next_pub > pub_period) ? now + pub_period : next_pub + pub_period;
      }
      if (camera_enabled_ && now >= next_cam) {
        captureCamera();
        next_cam = (now - next_cam > cam_period) ? now + cam_period : next_cam + cam_period;
      }
      if (range_id_ >= 0 && now >= next_range) {
        publishRange();
        next_range = (now - next_range > range_period) ? now + range_period : next_range + range_period;
      }
      if (render_ && now >= next_render) {
        renderFrame();
        next_render = (now - next_render > render_period) ? now + render_period : next_render + render_period;
        if (glfwWindowShouldClose(window_)) {
          ROS_INFO("Viewer closed: shutting down.");
          ros::shutdown();
        }
      }
      ros::WallDuration(0.001).sleep();
    }
  }

 private:
  struct JointInfo {
    std::string name;
    int jnt_id = -1;
    int qpos_adr = 0;
    int dof_adr = 0;
    int actuator_id = -1;  // -1: no actuator drives this joint (e.g. finger_passive)
  };

  // ---------------------------------------------------------------- model
  void loadModel() {
    char err[1000] = "";
    model_ = mj_loadXML(model_path_.c_str(), nullptr, err, sizeof(err));
    if (!model_) {
      throw std::runtime_error(std::string("mj_loadXML failed for '") + model_path_ + "': " + err);
    }
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("mj_makeData failed");
    ROS_INFO("Loaded %s: nbody=%d njnt=%d nq=%d nu=%d timestep=%.4f s", model_path_.c_str(),
             model_->nbody, model_->njnt, model_->nq, model_->nu, model_->opt.timestep);
  }

  void buildJointTable() {
    for (int j = 0; j < model_->njnt; ++j) {
      const int type = model_->jnt_type[j];
      if (type != mjJNT_HINGE && type != mjJNT_SLIDE) continue;  // skips the free joints of the blocks
      const char* n = mj_id2name(model_, mjOBJ_JOINT, j);
      JointInfo ji;
      ji.name = n ? n : "";
      ji.jnt_id = j;
      ji.qpos_adr = model_->jnt_qposadr[j];
      ji.dof_adr = model_->jnt_dofadr[j];
      joints_.push_back(ji);
    }
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] != mjTRN_JOINT) continue;
      const int j = model_->actuator_trnid[2 * a];
      for (auto& ji : joints_) {
        if (ji.jnt_id == j) ji.actuator_id = a;
      }
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      index_by_name_[joints_[i].name] = i;
      const int a = joints_[i].actuator_id;
      const char* an = (a >= 0) ? mj_id2name(model_, mjOBJ_ACTUATOR, a) : nullptr;
      ROS_INFO("  joint %-14s qpos[%d]  actuator: %s", joints_[i].name.c_str(), joints_[i].qpos_adr,
               an ? an : "(none)");
    }
    if (joints_.empty()) throw std::runtime_error("model has no hinge/slide joints");
  }

  void resetState() {
    const int kid = keyframe_.empty() ? -1 : mj_name2id(model_, mjOBJ_KEY, keyframe_.c_str());
    if (kid >= 0) {
      mj_resetDataKeyframe(model_, data_, kid);  // also restores ctrl from the keyframe
    } else {
      if (!keyframe_.empty()) ROS_WARN("keyframe '%s' not found: using default state", keyframe_.c_str());
      mj_resetData(model_, data_);
    }
    mj_forward(model_, data_);
    need_resync_ = true;
    contact_stats_.clear();
  }

  // ---------------------------------------------------------------- contact monitor
  struct ContactStat {
    int g1 = 0, g2 = 0;
    double min_dist = 1e9;
    long count = 0;
    bool env = false;
  };

  void buildContactMonitor(ros::NodeHandle& pnh) {
    std::string root_name;
    pnh.param<std::string>("robot_root_body", root_name, "base_link");
    robot_root_ = mj_name2id(model_, mjOBJ_BODY, root_name.c_str());
    robot_body_.assign(model_->nbody, 0);
    if (robot_root_ < 0) {
      ROS_WARN("robot_root_body '%s' not found: contact monitor disabled", root_name.c_str());
      return;
    }
    for (int b = 1; b < model_->nbody; ++b) {
      for (int a = b; a > 0; a = model_->body_parentid[a]) {
        if (a == robot_root_) {
          robot_body_[b] = 1;
          break;
        }
      }
    }
  }

  std::string bodyLabel(int b) const {
    const char* n = mj_id2name(model_, mjOBJ_BODY, b);
    return n ? std::string(n) : "body#" + std::to_string(b);
  }

  std::string geomLabel(int g) const {
    const char* n = mj_id2name(model_, mjOBJ_GEOM, g);
    if (n && *n) return n;
    if (model_->geom_type[g] == mjGEOM_MESH && model_->geom_dataid[g] >= 0) {
      const char* m = mj_id2name(model_, mjOBJ_MESH, model_->geom_dataid[g]);
      if (m) return m;
    }
    return "geom#" + std::to_string(g);
  }

  void recordContacts() {
    if (robot_root_ < 0) return;
    for (int i = 0; i < data_->ncon; ++i) {
      const mjContact& c = data_->contact[i];
      const int b1 = model_->geom_bodyid[c.geom1];
      const int b2 = model_->geom_bodyid[c.geom2];
      const bool r1 = robot_body_[b1] != 0, r2 = robot_body_[b2] != 0;
      if (!r1 && !r2) continue;  // e.g. block vs block, block vs floor
      if ((b1 == robot_root_ && b2 == 0) || (b2 == robot_root_ && b1 == 0)) continue;  // base resting on plate
      const int lo = std::min(c.geom1, c.geom2), hi = std::max(c.geom1, c.geom2);
      ContactStat& st = contact_stats_[std::make_pair(lo, hi)];
      st.g1 = lo;
      st.g2 = hi;
      st.env = !(r1 && r2);
      st.min_dist = std::min(st.min_dist, static_cast<double>(c.dist));
      ++st.count;
    }
  }

  bool onContactReport(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::ostringstream o;
    o.precision(6);
    o << "{\"sim_time\": " << data_->time << ", \"contacts\": [";
    bool first = true;
    for (const auto& kv : contact_stats_) {
      const ContactStat& st = kv.second;
      if (!first) o << ", ";
      first = false;
      o << "{\"kind\": \"" << (st.env ? "env" : "self") << "\""
        << ", \"body1\": \"" << bodyLabel(model_->geom_bodyid[st.g1]) << "\""
        << ", \"geom1\": \"" << geomLabel(st.g1) << "\""
        << ", \"body2\": \"" << bodyLabel(model_->geom_bodyid[st.g2]) << "\""
        << ", \"geom2\": \"" << geomLabel(st.g2) << "\""
        << ", \"min_dist\": " << st.min_dist << ", \"count\": " << st.count << "}";
    }
    o << "]}";
    contact_stats_.clear();
    res.success = true;
    res.message = o.str();
    return true;
  }

  // ---------------------------------------------------------------- ROS callbacks
  void onCommand(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() != msg->position.size()) {
      ROS_WARN_THROTTLE(2.0, "joint_command rejected: name/position size mismatch");
      return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = index_by_name_.find(msg->name[i]);
      if (it == index_by_name_.end()) {
        ROS_WARN_THROTTLE(2.0, "joint_command: unknown joint '%s'", msg->name[i].c_str());
        continue;
      }
      const int a = joints_[it->second].actuator_id;
      if (a < 0) {
        ROS_WARN_THROTTLE(2.0, "joint_command: joint '%s' has no actuator (ignored)", msg->name[i].c_str());
        continue;
      }
      double v = msg->position[i];
      if (!std::isfinite(v)) {
        ROS_WARN_THROTTLE(2.0, "joint_command: non-finite target for '%s' (ignored)", msg->name[i].c_str());
        continue;
      }
      if (model_->actuator_ctrllimited[a]) {
        v = std::min(std::max(v, model_->actuator_ctrlrange[2 * a]), model_->actuator_ctrlrange[2 * a + 1]);
      }
      data_->ctrl[a] = v;
    }
  }

  bool onReset(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    resetState();
    ROS_INFO("Simulation reset to keyframe '%s'", keyframe_.c_str());
    return true;
  }

  void publishJointStates() {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name.reserve(joints_.size());
    msg.position.reserve(joints_.size());
    msg.velocity.reserve(joints_.size());
    msg.effort.reserve(joints_.size());
    for (const auto& ji : joints_) {
      msg.name.push_back(ji.name);
      msg.position.push_back(data_->qpos[ji.qpos_adr]);
      msg.velocity.push_back(data_->qvel[ji.dof_adr]);
      msg.effort.push_back(ji.actuator_id >= 0 ? data_->actuator_force[ji.actuator_id] : 0.0);
    }
    joint_pub_.publish(msg);
  }

  // ---------------------------------------------------------------- sensors (Phase 4)
  void setupSensors(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string img_t, info_t, cpose_t, range_t, rpose_t;
    pnh.param<std::string>("camera_image_topic", img_t, "/camera/image_raw");
    pnh.param<std::string>("camera_info_topic", info_t, "/camera/camera_info");
    pnh.param<std::string>("camera_pose_topic", cpose_t, "/camera/pose");
    pnh.param<std::string>("range_topic", range_t, "/hcsr04/range");
    pnh.param<std::string>("range_pose_topic", rpose_t, "/hcsr04/pose");
    if (camera_enabled_) {
      cam_id_ = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
      if (cam_id_ < 0) {
        ROS_WARN("camera '%s' not found in the model: camera disabled", camera_name_.c_str());
        camera_enabled_ = false;
      } else {
        image_pub_ = nh.advertise<sensor_msgs::Image>(img_t, 2);
        info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(info_t, 2);
        cam_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(cpose_t, 5);
      }
    }
    if (!range_name_.empty()) {
      range_id_ = mj_name2id(model_, mjOBJ_SENSOR, range_name_.c_str());
      if (range_id_ < 0 || model_->sensor_type[range_id_] != mjSENS_RANGEFINDER) {
        ROS_WARN("rangefinder sensor '%s' not found: range publishing disabled", range_name_.c_str());
        range_id_ = -1;
      } else {
        range_adr_ = model_->sensor_adr[range_id_];
        range_site_ = model_->sensor_objid[range_id_];
        range_max_ = model_->sensor_cutoff[range_id_] > 0 ? model_->sensor_cutoff[range_id_] : 4.0;
        range_pub_ = nh.advertise<sensor_msgs::Range>(range_t, 5);
        range_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(rpose_t, 5);
      }
    }
  }

  // Pose from a MuJoCo position + row-major rotation matrix; column c of the matrix is scaled by sgn[c]
  // (used to turn MuJoCo's camera frame, which looks along -z with +y up, into the optical frame).
  static geometry_msgs::PoseStamped makePose(const ros::Time& t, const char* frame, const mjtNum* pos,
                                             const mjtNum* mat, const double* sgn) {
    mjtNum m[9], q[4];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) m[3 * r + c] = mat[3 * r + c] * sgn[c];
    }
    mju_mat2Quat(q, m);
    geometry_msgs::PoseStamped p;
    p.header.stamp = t;
    p.header.frame_id = frame;
    p.pose.position.x = pos[0];
    p.pose.position.y = pos[1];
    p.pose.position.z = pos[2];
    p.pose.orientation.w = q[0];
    p.pose.orientation.x = q[1];
    p.pose.orientation.y = q[2];
    p.pose.orientation.z = q[3];
    return p;
  }

  void captureCamera() {
    mjv_updateScene(model_, data_, &cam_opt_, nullptr, &fixed_cam_, mjCAT_ALL, &scn_);
    mjrRect vp = {0, 0, cam_w_, cam_h_};
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    mjr_render(vp, &scn_, &con_);
    mjr_readPixels(rgb_.data(), nullptr, vp, &con_);  // bottom-up rows
    mjr_setBuffer(mjFB_WINDOW, &con_);

    const ros::Time stamp = ros::Time::now();
    static const double kOptical[3] = {1.0, -1.0, -1.0};  // MuJoCo cam (x right, y up, -z fwd) -> optical
    cam_pose_pub_.publish(makePose(stamp, "world", data_->cam_xpos + 3 * cam_id_, data_->cam_xmat + 9 * cam_id_, kOptical));

    sensor_msgs::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = camera_name_ + "_optical";
    img.height = cam_h_;
    img.width = cam_w_;
    img.encoding = "rgb8";
    img.is_bigendian = 0;
    img.step = 3 * cam_w_;
    img.data.resize(static_cast<size_t>(img.step) * cam_h_);
    for (int r = 0; r < cam_h_; ++r) {  // flip: OpenGL is bottom-up, ROS images are top-down
      std::memcpy(&img.data[static_cast<size_t>(r) * img.step], &rgb_[static_cast<size_t>(cam_h_ - 1 - r) * img.step], img.step);
    }
    image_pub_.publish(img);

    const double fovy = model_->cam_fovy[cam_id_] * M_PI / 180.0;
    const double f = 0.5 * cam_h_ / std::tan(0.5 * fovy);
    sensor_msgs::CameraInfo ci;
    ci.header = img.header;
    ci.height = cam_h_;
    ci.width = cam_w_;
    ci.distortion_model = "plumb_bob";
    ci.D.assign(5, 0.0);
    ci.K = {f, 0, 0.5 * cam_w_, 0, f, 0.5 * cam_h_, 0, 0, 1};
    ci.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    ci.P = {f, 0, 0.5 * cam_w_, 0, 0, f, 0.5 * cam_h_, 0, 0, 0, 1, 0};
    info_pub_.publish(ci);

    updatePreview();
  }

  void updatePreview() {  // box-filter down to <= 320 px wide (mjr_drawPixels does not scale)
    const int f = std::max(1, cam_w_ / 320);
    prev_w_ = cam_w_ / f;
    prev_h_ = cam_h_ / f;
    preview_rgb_.assign(static_cast<size_t>(3) * prev_w_ * prev_h_, 0);
    for (int y = 0; y < prev_h_; ++y) {
      for (int x = 0; x < prev_w_; ++x) {
        for (int c = 0; c < 3; ++c) {
          int sum = 0;
          for (int dy = 0; dy < f; ++dy) {
            for (int dx = 0; dx < f; ++dx) sum += rgb_[(static_cast<size_t>(y * f + dy) * cam_w_ + (x * f + dx)) * 3 + c];
          }
          preview_rgb_[(static_cast<size_t>(y) * prev_w_ + x) * 3 + c] = static_cast<unsigned char>(sum / (f * f));
        }
      }
    }
  }

  void publishRange() {
    const ros::Time stamp = ros::Time::now();
    static const double kIdentity[3] = {1.0, 1.0, 1.0};
    range_pose_pub_.publish(makePose(stamp, "world", data_->site_xpos + 3 * range_site_, data_->site_xmat + 9 * range_site_, kIdentity));

    const double kMin = 0.02;  // HC-SR04 datasheet minimum
    double d = data_->sensordata[range_adr_];
    sensor_msgs::Range r;
    r.header.stamp = stamp;
    r.header.frame_id = "hcsr04";
    r.radiation_type = sensor_msgs::Range::ULTRASOUND;
    r.field_of_view = 0.26;  // ~15 deg cone on the real part; the simulated sensor is a single ray
    r.min_range = kMin;
    r.max_range = range_max_;
    if (d < 0.0 || d >= range_max_ - 1e-6) {
      r.range = std::numeric_limits<float>::infinity();  // no echo
    } else {
      if (range_noise_ > 0.0) d += std::normal_distribution<double>(0.0, range_noise_)(rng_);
      r.range = (d < kMin) ? -std::numeric_limits<float>::infinity() : static_cast<float>(std::min(d, range_max_));
    }
    range_pub_.publish(r);
  }

  // ---------------------------------------------------------------- viewer
  bool initViewer() {
    if (!glfwInit()) {
      ROS_ERROR("glfwInit failed");
      return false;
    }
    glfwWindowHint(GLFW_VISIBLE, render_ ? GLFW_TRUE : GLFW_FALSE);  // hidden window = camera-only mode
    window_ = glfwCreateWindow(1200, 900, "autonomous_manipulator - MuJoCo", nullptr, nullptr);
    if (!window_) {
      ROS_ERROR("glfwCreateWindow failed (is a display available?)");
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_defaultFreeCamera(model_, &cam_);
    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);
    viewer_ready_ = true;
    if (camera_enabled_) {
      const int ow = model_->vis.global.offwidth, oh = model_->vis.global.offheight;
      if (cam_w_ > ow || cam_h_ > oh) {
        ROS_WARN("camera %dx%d exceeds the offscreen buffer %dx%d: clamping. Add <visual><global offwidth=\"%d\" "
                 "offheight=\"%d\"/></visual> to the MJCF for the full size.",
                 cam_w_, cam_h_, ow, oh, cam_w_, cam_h_);
        cam_w_ = std::min(cam_w_, ow);
        cam_h_ = std::min(cam_h_, oh);
      }
      rgb_.assign(static_cast<size_t>(3) * cam_w_ * cam_h_, 0);
      mjv_defaultOption(&cam_opt_);  // what the camera sees: no site markers, no rangefinder ray
      for (int g = 0; g < mjNGROUP; ++g) cam_opt_.sitegroup[g] = 0;
      cam_opt_.flags[mjVIS_RANGEFINDER] = 0;
      mjv_defaultCamera(&fixed_cam_);
      fixed_cam_.type = mjCAMERA_FIXED;
      fixed_cam_.fixedcamid = cam_id_;
    }
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, &MujocoSimNode::onKey);
    glfwSetMouseButtonCallback(window_, &MujocoSimNode::onMouseButton);
    glfwSetCursorPosCallback(window_, &MujocoSimNode::onCursor);
    glfwSetScrollCallback(window_, &MujocoSimNode::onScroll);
    return true;
  }

  void shutdownViewer() {
    if (!window_) return;
    if (viewer_ready_) {
      mjr_freeContext(&con_);
      mjv_freeScene(&scn_);
      viewer_ready_ = false;
    }
    glfwDestroyWindow(window_);
    window_ = nullptr;
    glfwTerminate();
  }

  void renderFrame() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);
    if (camera_enabled_ && preview_ && !preview_rgb_.empty() && viewport.width > 0 && viewport.height > 0) {
      const int margin = 10;
      mjrRect pv = {viewport.width - prev_w_ - margin, margin, prev_w_, prev_h_};
      mjrRect frame = {pv.left - 2, pv.bottom - 2, pv.width + 4, pv.height + 4};
      mjr_rectangle(frame, 0.1f, 0.1f, 0.1f, 1.0f);
      mjr_drawPixels(preview_rgb_.data(), nullptr, pv, &con_);
      mjr_text(mjFONT_NORMAL, camera_name_.c_str(), &con_, static_cast<float>(pv.left) / viewport.width,
               static_cast<float>(pv.bottom + pv.height + 4) / viewport.height, 1.0f, 1.0f, 1.0f);
    }
    char text[160];
    std::snprintf(text, sizeof(text), "sim time %.2f s%s\n[Space] pause  [Backspace] reset  [C] camera  [Esc] quit",
                  data_->time, paused_ ? "  (PAUSED)" : "");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, text, nullptr, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  static MujocoSimNode* self(GLFWwindow* w) {
    return static_cast<MujocoSimNode*>(glfwGetWindowUserPointer(w));
  }

  static void onKey(GLFWwindow* w, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    MujocoSimNode* s = self(w);
    if (key == GLFW_KEY_SPACE) {
      s->paused_ = !s->paused_;
      if (!s->paused_) s->need_resync_ = true;
    } else if (key == GLFW_KEY_BACKSPACE) {
      s->resetState();
    } else if (key == GLFW_KEY_C) {
      s->preview_ = !s->preview_;
    } else if (key == GLFW_KEY_ESCAPE) {
      glfwSetWindowShouldClose(w, 1);
    }
  }

  static void onMouseButton(GLFWwindow* w, int, int, int) {
    MujocoSimNode* s = self(w);
    s->btn_left_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    s->btn_right_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(w, &s->last_x_, &s->last_y_);
  }

  static void onCursor(GLFWwindow* w, double x, double y) {
    MujocoSimNode* s = self(w);
    const double dx = x - s->last_x_;
    const double dy = y - s->last_y_;
    s->last_x_ = x;
    s->last_y_ = y;
    if (!s->btn_left_ && !s->btn_right_) return;
    int width = 0, height = 1;
    glfwGetWindowSize(w, &width, &height);
    const bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    int action;
    if (s->btn_right_) {
      action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else {
      action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    mjv_moveCamera(s->model_, action, dx / height, dy / height, &s->scn_, &s->cam_);
  }

  static void onScroll(GLFWwindow* w, double, double yoffset) {
    MujocoSimNode* s = self(w);
    mjv_moveCamera(s->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &s->scn_, &s->cam_);
  }

  // ---------------------------------------------------------------- state
  std::string model_path_, keyframe_;
  double rtf_ = 1.0, joint_rate_ = 100.0, render_rate_ = 60.0;
  bool render_ = true, paused_ = false, need_resync_ = true;

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::vector<JointInfo> joints_;
  std::map<std::string, size_t> index_by_name_;

  ros::Publisher joint_pub_;
  ros::Subscriber cmd_sub_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer contact_srv_;
  int robot_root_ = -1;
  std::vector<char> robot_body_;
  std::map<std::pair<int, int>, ContactStat> contact_stats_;

  GLFWwindow* window_ = nullptr;
  bool viewer_ready_ = false;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  bool btn_left_ = false, btn_right_ = false;
  double last_x_ = 0.0, last_y_ = 0.0;

  // Phase 4 sensors
  bool camera_enabled_ = true, preview_ = true;
  std::string camera_name_, range_name_;
  int cam_w_ = 640, cam_h_ = 480, cam_id_ = -1;
  int prev_w_ = 0, prev_h_ = 0;
  double camera_rate_ = 15.0, range_rate_ = 20.0, range_noise_ = 0.0, range_max_ = 4.0;
  int range_id_ = -1, range_adr_ = 0, range_site_ = 0;
  std::vector<unsigned char> rgb_, preview_rgb_;
  mjvOption cam_opt_;
  mjvCamera fixed_cam_;
  std::mt19937 rng_{42};
  ros::Publisher image_pub_, info_pub_, cam_pose_pub_, range_pub_, range_pose_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mujoco_sim");
  ros::NodeHandle nh, pnh("~");
  try {
    MujocoSimNode node(nh, pnh);
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("mujoco_sim: %s", e.what());
    return 1;
  }
  return 0;
}
