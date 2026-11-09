// Collision-aware motion planning. The planner owns a PRIVATE copy of the MJCF (own mjModel/mjData, never the simulator's)
// and uses MuJoCo's collision pipeline to check every planned configuration against the believed scene.
#pragma once
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "autonomous_manipulator/trajectory_planner.hpp"

namespace am {

struct SceneObject {
  std::string color;
  Vec3 center;  // body origin of an upright dumbbell (ignored for the held object)
};

struct PlanContext {
  std::vector<SceneObject> objects;
  int held = -1;           // index in objects of the object carried by the gripper (-1 = none)
  double carry_dz = 0.017; // held object centre = ee - (0,0,carry_dz)
  int target = -1;         // object whose bar may touch the jaws (final grasp approach)
  double finger = 0.9;     // finger angle assumed during the motion
};

class MotionPlanner {
 public:
  MotionPlanner() = default;
  ~MotionPlanner();
  bool load(const std::string& model_path, double margin);
  bool inCollision(const JointVec& q, const PlanContext& ctx, std::string* why);
  // Checks a trajectory at <= 2 cm of end-effector motion (and every 50 ms) plus its end points.
  bool validate(const JointTrajectory& tr, const PlanContext& ctx, std::string* why);

 private:
  void setFree(int body_idx, const Vec3& p);
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qadr_[6] = {};
  std::vector<int> obj_body_;
  std::vector<std::string> obj_color_;
  std::vector<char> robot_body_, obj_flag_;
  std::set<std::pair<int, int>> static_pairs_;  // geom pairs touching in the neutral pose (model artefacts), ignored
  int jaw_up_ = -1, jaw_lo_ = -1, held_body_ = -1, target_body_ = -1;
};

}  // namespace am
