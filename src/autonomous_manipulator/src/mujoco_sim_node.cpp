// mujoco_sim_node: the ONLY owner of mjModel / mjData.
//   * main thread: GLFW window, mj_step loop (real-time paced), camera rendering, publishing, service requests
//   * ROS callbacks (AsyncSpinner threads) only store commands under a mutex or enqueue closures that the main thread
//     executes between steps  =>  no concurrent access to MuJoCo data.
// Published: /clock, /joint_states (pos, vel, effort=actuator force), /ultrasonic/range (same stamp as joint_states),
//            /color_cam/image_raw + camera_info, /sim/ee_pose (debug), /sim/debug/object_poses (VALIDATION ONLY).
// Subscribed: /arm/joint_command (Float64MultiArray: hip, shoulder, arm, hand, finger1, finger2 position targets).
// Service: /sim/grasp_weld (std_srvs/SetBool) - optional simulation-assisted grasp constraint.
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <limits>
#include <memory>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include "autonomous_manipulator/robot_model.hpp"

namespace {
const char* kJoints[6] = {"hip", "shoulder", "arm", "hand", "finger1", "finger2"};
const char* kActs[6] = {"hip_act", "shoulder_act", "arm_act", "hand_act", "finger1_act", "finger2_act"};
}  // namespace

class SimNode {
 public:
  SimNode() : pnh_("~"), cfg_("/am/sim") {}
  ~SimNode() {
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
  }

  bool init() {
    std::string path;
    pnh_.param<std::string>("model_path", path, "");
    pnh_.param("gui", gui_, true);
    cfg_.param("realtime_factor", rtf_, 1.0);
    cfg_.param("image_width", W_, 640);
    cfg_.param("image_height", H_, 480);
    double cam_hz, st_hz;
    cfg_.param("camera_rate_hz", cam_hz, 10.0);
    cfg_.param("state_rate_hz", st_hz, 100.0);
    cam_period_ = 1.0 / cam_hz;
    state_period_ = 1.0 / st_hz;
    std::string cam_name;
    cfg_.param<std::string>("camera_name", cam_name, "color_cam");

    char err[1000] = "";
    m_ = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
    if (!m_) { ROS_FATAL("sim: cannot load MJCF '%s': %s", path.c_str(), err); return false; }
    d_ = mj_makeData(m_);

    for (int i = 0; i < 6; ++i) {
      int j = mj_name2id(m_, mjOBJ_JOINT, kJoints[i]);
      int a = mj_name2id(m_, mjOBJ_ACTUATOR, kActs[i]);
      if (j < 0 || a < 0) { ROS_FATAL("sim: joint '%s' / actuator '%s' missing in MJCF", kJoints[i], kActs[i]); return false; }
      qadr_[i] = m_->jnt_qposadr[j]; vadr_[i] = m_->jnt_dofadr[j]; act_[i] = a;
      lo_[i] = m_->actuator_ctrlrange[2 * a]; hi_[i] = m_->actuator_ctrlrange[2 * a + 1];
      limited_[i] = m_->actuator_ctrllimited[a];
    }
    cam_id_ = mj_name2id(m_, mjOBJ_CAMERA, cam_name.c_str());
    ee_site_ = mj_name2id(m_, mjOBJ_SITE, "ee");
    us_site_ = mj_name2id(m_, mjOBJ_SITE, "ultrasonic_site");
    int us_sensor = mj_name2id(m_, mjOBJ_SENSOR, "ultrasonic_range");
    hand_body_ = mj_name2id(m_, mjOBJ_BODY, "base_hand");
    if (cam_id_ < 0 || ee_site_ < 0 || us_site_ < 0 || us_sensor < 0 || hand_body_ < 0) {
      ROS_FATAL("sim: camera/site/sensor/body missing (color_cam, ee, ultrasonic_site, ultrasonic_range, base_hand)");
      return false;
    }
    us_adr_ = m_->sensor_adr[us_sensor];
    for (int b = 0; b < m_->nbody; ++b) {
      const char* nm = mj_id2name(m_, mjOBJ_BODY, b);
      if (nm && std::string(nm).rfind("dumbbell_", 0) == 0) { obj_body_.push_back(b); obj_name_.push_back(nm); }
    }
    for (size_t i = 0; i < obj_body_.size(); ++i) {
      std::string wn = "weld_d" + obj_name_[i].substr(9, 1);
      weld_.push_back(mj_name2id(m_, mjOBJ_EQUALITY, wn.c_str()));
    }
    if (W_ > m_->vis.global.offwidth || H_ > m_->vis.global.offheight) {
      ROS_FATAL("sim: image %dx%d exceeds offscreen buffer %dx%d (set <visual><global offwidth offheight>)", W_, H_,
                m_->vis.global.offwidth, m_->vis.global.offheight);
      return false;
    }

    std::vector<double> init;
    cfg_.getParam("initial_joints", init);
    double fin = 0.9;
    cfg_.param("initial_finger", fin, 0.9);
    if (init.size() != 4) init = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) { d_->qpos[qadr_[i]] = init[i]; cmd_[i] = init[i]; }
    d_->qpos[qadr_[4]] = d_->qpos[qadr_[5]] = fin;
    cmd_[4] = cmd_[5] = fin;
    mj_forward(m_, d_);
    if (!kinematicsSelfCheck()) return false;
    for (int i = 0; i < 4; ++i) { d_->qpos[qadr_[i]] = init[i]; }
    d_->qpos[qadr_[4]] = d_->qpos[qadr_[5]] = fin;
    mj_forward(m_, d_);

    if (!initGL()) return false;

    clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);
    js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    range_pub_ = nh_.advertise<sensor_msgs::Range>("/ultrasonic/range", 10);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/color_cam/image_raw", 2);
    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/color_cam/camera_info", 2);
    ee_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/sim/ee_pose", 5);
    obj_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/sim/debug/object_poses", 2);
    names_pub_ = nh_.advertise<std_msgs::String>("/sim/debug/object_names", 1, true);
    std_msgs::String names;
    for (size_t i = 0; i < obj_name_.size(); ++i) names.data += (i ? "," : "") + obj_name_[i];
    names_pub_.publish(names);
    cmd_sub_ = nh_.subscribe("/arm/joint_command", 5, &SimNode::onCommand, this);
    weld_srv_ = nh_.advertiseService("/sim/grasp_weld", &SimNode::onWeld, this);
    ROS_INFO("sim: model loaded (%d bodies, dt=%.4f s). Kinematics self-check passed.", m_->nbody, m_->opt.timestep);
    return true;
  }

  void run() {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    auto wall0 = std::chrono::steady_clock::now();
    double sim0 = d_->time;
    double next_state = d_->time, next_cam = d_->time, next_clock = d_->time;
    auto last_draw = wall0;
    while (ros::ok() && !glfwWindowShouldClose(win_)) {
      runRequests();
      {
        std::lock_guard<std::mutex> l(cmd_mtx_);
        for (int i = 0; i < 6; ++i) {
          double c = cmd_[i];
          if (limited_[i]) c = std::min(hi_[i], std::max(lo_[i], c));
          d_->ctrl[act_[i]] = c;
        }
      }
      double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
      double target = sim0 + rtf_ * wall;
      int steps = 0;
      while (d_->time < target && steps < 50) {
        mj_step(m_, d_);
        ++steps;
        if (d_->time >= next_clock) { publishClock(); next_clock += 0.002; }
        if (d_->time >= next_state) { publishState(); next_state += state_period_; }
      }
      if (d_->time < target - 0.25) { wall0 = std::chrono::steady_clock::now(); sim0 = d_->time; }  // cannot keep up: re-sync
      if (d_->time >= next_cam) { publishCamera(); next_cam += cam_period_; }
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_draw).count() > 0.02) { draw(); last_draw = now; }
      glfwPollEvents();
      if (steps == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    spinner.stop();
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    mjv_freeScene(&scn_cam_);
    glfwDestroyWindow(win_);
    glfwTerminate();
    ros::shutdown();
  }

 private:
  // ---------- start-up validation of the analytic kinematics against MuJoCo ----------
  bool kinematicsSelfCheck() {
    const double qs[4][4] = {{0.3, 0.2, 1.0, -0.4}, {-0.8, -0.5, 1.9, -1.1}, {1.2, 0.6, 0.4, 0.9}, {0.0, -1.2, 2.5, -0.3}};
    double worst = 0;
    auto dist = [](const am::Vec3& a, const mjtNum* b) {
      return std::sqrt((a.x - b[0]) * (a.x - b[0]) + (a.y - b[1]) * (a.y - b[1]) + (a.z - b[2]) * (a.z - b[2]));
    };
    auto adiff = [](const am::Vec3& a, const mjtNum* R, int c) {  // column c of a row-major 3x3
      return std::fabs(a.x - R[c]) + std::fabs(a.y - R[3 + c]) + std::fabs(a.z - R[6 + c]);
    };
    for (int k = 0; k < 4; ++k) {
      const double* qa = qs[k];
      for (int i = 0; i < 4; ++i) d_->qpos[qadr_[i]] = qa[i];
      mj_kinematics(m_, d_);
      mj_camlight(m_, d_);
      am::JointVec q = {qa[0], qa[1], qa[2], qa[3]};
      am::Pose ee = am::forwardKinematics(q), cam = am::cameraPose(q), us = am::ultrasonicPose(q);
      double e_ee = dist(ee.p, d_->site_xpos + 3 * ee_site_);
      double e_cp = dist(cam.p, d_->cam_xpos + 3 * cam_id_);
      double e_up = dist(us.p, d_->site_xpos + 3 * us_site_);
      double e_cz = adiff(cam.R.col(2), d_->cam_xmat + 9 * cam_id_, 2);
      double e_uz = adiff(am::ultrasonicDirection(q), d_->site_xmat + 9 * us_site_, 2);
      ROS_INFO("selfcheck q%d: ee=%.4f cam_pos=%.4f us_pos=%.4f cam_z=%.4f us_z=%.4f", k, e_ee, e_cp, e_up, e_cz, e_uz);
      if (e_ee > 1e-3 || e_cp > 1e-3 || e_up > 1e-3 || e_cz > 1e-3 || e_uz > 1e-3) {
        const mjtNum* p = d_->site_xpos + 3 * ee_site_;
        ROS_ERROR("  MuJoCo ee=(%.5f %.5f %.5f)  model ee=(%.5f %.5f %.5f)", p[0], p[1], p[2], ee.p.x, ee.p.y, ee.p.z);
      }
      worst = std::max({worst, e_ee, e_cp, e_up, e_cz, e_uz});
    }
    if (worst > 1e-3) {
      ROS_FATAL("sim: analytic FK disagrees with MuJoCo by %.4f - robot_model.hpp does not match the MJCF", worst);
      return false;
    }
    ROS_INFO("sim: FK/camera/ultrasonic model matches MuJoCo (worst deviation %.2e)", worst);
    return true;
  }

  // ---------- OpenGL / rendering ----------
  bool initGL() {
    if (!glfwInit()) { ROS_FATAL("sim: glfwInit failed"); return false; }
    if (!gui_) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    win_ = glfwCreateWindow(1200, 800, "MuJoCo - autonomous pyramid", nullptr, nullptr);
    if (!win_) { ROS_FATAL("sim: cannot create a GLFW window (no display? try xvfb-run)"); return false; }
    glfwMakeContextCurrent(win_);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjv_defaultScene(&scn_cam_);
    mjr_defaultContext(&con_);
    mjv_makeScene(m_, &scn_, 2000);
    mjv_makeScene(m_, &scn_cam_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);
    cam_.type = mjCAMERA_FREE;
    cam_.lookat[0] = 0.15; cam_.lookat[1] = -0.1; cam_.lookat[2] = 0.1;
    cam_.distance = 1.1; cam_.azimuth = 130; cam_.elevation = -25;
    mjv_defaultCamera(&robot_cam_);
    robot_cam_.type = mjCAMERA_FIXED;
    robot_cam_.fixedcamid = cam_id_;
    glfwSetWindowUserPointer(win_, this);
    glfwSetCursorPosCallback(win_, [](GLFWwindow* w, double x, double y) { static_cast<SimNode*>(glfwGetWindowUserPointer(w))->onMouseMove(x, y); });
    glfwSetMouseButtonCallback(win_, [](GLFWwindow* w, int, int, int) { static_cast<SimNode*>(glfwGetWindowUserPointer(w))->onMouseButton(); });
    glfwSetScrollCallback(win_, [](GLFWwindow* w, double, double dy) { static_cast<SimNode*>(glfwGetWindowUserPointer(w))->onScroll(dy); });
    return true;
  }
  void onMouseButton() {
    btn_l_ = glfwGetMouseButton(win_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    btn_r_ = glfwGetMouseButton(win_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(win_, &lx_, &ly_);
  }
  void onMouseMove(double x, double y) {
    double dx = x - lx_, dy = y - ly_;
    lx_ = x; ly_ = y;
    if (!btn_l_ && !btn_r_) return;
    int w, h;
    glfwGetWindowSize(win_, &w, &h);
    bool shift = glfwGetKey(win_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    mjtMouse a = btn_r_ ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) : (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
    mjv_moveCamera(m_, a, dx / h, dy / h, &scn_, &cam_);
  }
  void onScroll(double dy) { mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, -0.05 * dy, &scn_, &cam_); }

  void draw() {
    mjrRect vp = {0, 0, 0, 0};
    glfwGetFramebufferSize(win_, &vp.width, &vp.height);
    mjr_setBuffer(mjFB_WINDOW, &con_);
    mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(vp, &scn_, &con_);
    glfwSwapBuffers(win_);
  }

  void publishCamera() {
    mjrRect vp = {0, 0, W_, H_};
    mjv_updateScene(m_, d_, &opt_, nullptr, &robot_cam_, mjCAT_ALL, &scn_cam_);
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    mjr_render(vp, &scn_cam_, &con_);
    std::vector<unsigned char> buf(static_cast<size_t>(W_) * H_ * 3);
    mjr_readPixels(buf.data(), nullptr, vp, &con_);
    mjr_setBuffer(mjFB_WINDOW, &con_);
    sensor_msgs::Image im;
    im.header.stamp = ros::Time(d_->time);
    im.header.frame_id = "color_cam";
    im.width = W_; im.height = H_; im.encoding = "rgb8"; im.step = W_ * 3;
    im.data.resize(buf.size());
    for (int r = 0; r < H_; ++r)  // OpenGL rows are bottom-up
      std::copy(buf.begin() + static_cast<size_t>(H_ - 1 - r) * W_ * 3, buf.begin() + static_cast<size_t>(H_ - r) * W_ * 3,
                im.data.begin() + static_cast<size_t>(r) * W_ * 3);
    img_pub_.publish(im);
    double fovy = m_->cam_fovy[cam_id_];
    double fy = (H_ / 2.0) / std::tan(fovy * M_PI / 360.0);
    sensor_msgs::CameraInfo ci;
    ci.header = im.header; ci.width = W_; ci.height = H_;
    ci.K = {fy, 0, W_ / 2.0, 0, fy, H_ / 2.0, 0, 0, 1};
    info_pub_.publish(ci);
  }

  // ---------- publishing ----------
  void publishClock() {
    rosgraph_msgs::Clock c;
    c.clock = ros::Time(d_->time);
    clock_pub_.publish(c);
  }
  void publishState() {
    ros::Time t(d_->time);
    sensor_msgs::JointState js;
    js.header.stamp = t;
    for (int i = 0; i < 6; ++i) {
      js.name.push_back(kJoints[i]);
      js.position.push_back(d_->qpos[qadr_[i]]);
      js.velocity.push_back(d_->qvel[vadr_[i]]);
      js.effort.push_back(d_->actuator_force[act_[i]]);
    }
    js_pub_.publish(js);
    sensor_msgs::Range r;
    r.header.stamp = t;
    r.header.frame_id = "ultrasonic_site";
    r.radiation_type = sensor_msgs::Range::ULTRASOUND;
    r.field_of_view = 0.05; r.min_range = 0.0; r.max_range = 3.0;
    double v = d_->sensordata[us_adr_];
    r.range = v >= 0 ? static_cast<float>(v) : std::numeric_limits<float>::infinity();  // -1 = no echo
    range_pub_.publish(r);
    geometry_msgs::PoseStamped ee;
    ee.header.stamp = t; ee.header.frame_id = "world";
    const mjtNum* p = d_->site_xpos + 3 * ee_site_;
    ee.pose.position.x = p[0]; ee.pose.position.y = p[1]; ee.pose.position.z = p[2];
    ee.pose.orientation.w = 1;
    ee_pub_.publish(ee);
    if (t.toSec() >= next_obj_) {
      next_obj_ = t.toSec() + 0.1;
      geometry_msgs::PoseArray pa;
      pa.header.stamp = t; pa.header.frame_id = "world";
      for (int b : obj_body_) {
        geometry_msgs::Pose po;
        po.position.x = d_->xpos[3 * b]; po.position.y = d_->xpos[3 * b + 1]; po.position.z = d_->xpos[3 * b + 2];
        po.orientation.w = d_->xquat[4 * b]; po.orientation.x = d_->xquat[4 * b + 1];
        po.orientation.y = d_->xquat[4 * b + 2]; po.orientation.z = d_->xquat[4 * b + 3];
        pa.poses.push_back(po);
      }
      obj_pub_.publish(pa);
    }
  }

  // ---------- ROS callbacks (no MuJoCo access) ----------
  void onCommand(const std_msgs::Float64MultiArrayConstPtr& m) {
    if (m->data.size() != 6) { ROS_WARN_THROTTLE(2, "sim: /arm/joint_command needs 6 values"); return; }
    std::lock_guard<std::mutex> l(cmd_mtx_);
    for (int i = 0; i < 6; ++i) cmd_[i] = m->data[i];
  }
  bool onWeld(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    auto prom = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto fut = prom->get_future();
    bool on = req.data;
    {
      std::lock_guard<std::mutex> l(req_mtx_);
      requests_.push([this, on, prom]() { prom->set_value(setWeld(on)); });
    }
    auto st = fut.wait_for(std::chrono::seconds(5));
    if (st != std::future_status::ready) { res.success = false; res.message = "timeout"; return true; }
    auto r = fut.get();
    res.success = r.first; res.message = r.second;
    return true;
  }
  void runRequests() {
    for (;;) {
      std::function<void()> f;
      {
        std::lock_guard<std::mutex> l(req_mtx_);
        if (requests_.empty()) return;
        f = requests_.front();
        requests_.pop();
      }
      f();
    }
  }

  // Simulation-assisted grasp: weld the dumbbell nearest to the ee site to base_hand, freezing the CURRENT relative pose.
  std::pair<bool, std::string> setWeld(bool on) {
    if (!on) {
      for (int w : weld_) if (w >= 0) d_->eq_active[w] = 0;
      return {true, "weld released"};
    }
    const mjtNum* pe = d_->site_xpos + 3 * ee_site_;
    int best = -1;
    double bd = 0.05;  // must be within 5 cm of the ee site
    for (size_t i = 0; i < obj_body_.size(); ++i) {
      int b = obj_body_[i];
      double dd = std::sqrt(std::pow(d_->xpos[3 * b] - pe[0], 2) + std::pow(d_->xpos[3 * b + 1] - pe[1], 2) +
                            std::pow(d_->xpos[3 * b + 2] - pe[2], 2));
      if (dd < bd) { bd = dd; best = static_cast<int>(i); }
    }
    if (best < 0 || weld_[best] < 0) return {false, "no dumbbell within 5 cm of the ee"};
    int b2 = obj_body_[best], b1 = hand_body_, e = weld_[best];
    mjtNum neg[4], rel[4], dp[3], pos[3];
    mju_negQuat(neg, d_->xquat + 4 * b1);
    mju_mulQuat(rel, neg, d_->xquat + 4 * b2);
    for (int k = 0; k < 3; ++k) dp[k] = d_->xpos[3 * b2 + k] - d_->xpos[3 * b1 + k];
    mju_rotVecQuat(pos, dp, neg);
    mjtNum* data = m_->eq_data + e * mjNEQDATA;
    for (int k = 0; k < 3; ++k) data[k] = 0;        // anchor (body2 frame)
    for (int k = 0; k < 3; ++k) data[3 + k] = pos[k];  // relpose position
    for (int k = 0; k < 4; ++k) data[6 + k] = rel[k];  // relpose quaternion
    data[10] = 1;                                   // torquescale
    d_->eq_active[e] = 1;
    return {true, "weld activated for " + obj_name_[best]};
  }

  ros::NodeHandle nh_, pnh_, cfg_;
  ros::Publisher clock_pub_, js_pub_, range_pub_, img_pub_, info_pub_, ee_pub_, obj_pub_, names_pub_;
  ros::Subscriber cmd_sub_;
  ros::ServiceServer weld_srv_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjvCamera cam_, robot_cam_;
  mjvOption opt_;
  mjvScene scn_, scn_cam_;
  mjrContext con_;
  GLFWwindow* win_ = nullptr;
  bool gui_ = true, btn_l_ = false, btn_r_ = false;
  double lx_ = 0, ly_ = 0, rtf_ = 1.0, cam_period_, state_period_, next_obj_ = 0;
  int W_, H_;
  int qadr_[6], vadr_[6], act_[6], cam_id_, ee_site_, us_site_, us_adr_, hand_body_;
  double lo_[6], hi_[6];
  unsigned char limited_[6];
  std::vector<int> obj_body_, weld_;
  std::vector<std::string> obj_name_;
  std::mutex cmd_mtx_, req_mtx_;
  double cmd_[6];
  std::queue<std::function<void()>> requests_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mujoco_sim");
  SimNode node;
  if (!node.init()) return 1;
  node.run();
  return 0;
}
