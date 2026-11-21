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
 *
 * VIEWER KEYS: Space = pause/resume, Backspace = reset, Esc = quit,
 *              left drag = rotate, right drag = pan, Shift+drag = other plane, wheel = zoom.
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

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
    if (rtf_ <= 0.0 || joint_rate_ <= 0.0 || render_rate_ <= 0.0) {
      throw std::runtime_error("realtime_factor, joint_states_rate and render_rate must be > 0");
    }

    loadModel();
    buildJointTable();
    resetState();

    joint_pub_ = nh.advertise<sensor_msgs::JointState>(js_topic, 10);
    cmd_sub_ = nh.subscribe(cmd_topic, 10, &MujocoSimNode::onCommand, this);
    reset_srv_ = nh.advertiseService("/sim/reset", &MujocoSimNode::onReset, this);

    if (render_ && !initViewer()) {
      ROS_WARN("Viewer could not be created: continuing headless.");
      render_ = false;
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
    const ros::WallDuration pub_period(1.0 / joint_rate_);
    const ros::WallDuration render_period(1.0 / render_rate_);
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

  // ---------------------------------------------------------------- viewer
  bool initViewer() {
    if (!glfwInit()) {
      ROS_ERROR("glfwInit failed");
      return false;
    }
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
    char text[160];
    std::snprintf(text, sizeof(text), "sim time %.2f s%s\n[Space] pause  [Backspace] reset  [Esc] quit",
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

  GLFWwindow* window_ = nullptr;
  bool viewer_ready_ = false;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  bool btn_left_ = false, btn_right_ = false;
  double last_x_ = 0.0, last_y_ = 0.0;
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
