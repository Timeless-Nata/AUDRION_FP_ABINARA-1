// robot_model.hpp - MuJoCo-independent description of the robot (loaded from config/robot_model.txt).
//
// The file is produced by tools/export_robot_model.py from the MJCF. The core algorithms (kinematics,
// IK, collision checking) only ever see this structure, so they run unchanged for the simulation and
// for the real robot.
#pragma once
#include <array>
#include <string>
#include <vector>

#include "autonomous_manipulator/math_utils.hpp"

namespace arm {

constexpr int kArmDof = 4;       // joint_1..joint_4
constexpr int kModelDof = 5;     // + finger_active (finger_passive is coupled: q = coef * finger_active)

struct Body {
  std::string name;
  int parent = -1;       // index into RobotModel::bodies (-1 for the world)
  Pose fixed;            // pose of this body frame in the parent frame at joint value 0
  int dof = -1;          // index into the dof vector, -1 = no joint
  double coef = 0;       // joint value = coef * dof value (finger_passive: -1)
  Vec3 axis{0, 0, 1};    // joint axis in the body frame
  double lo = 0, hi = 0; // joint limits [rad]
};

// Convex collision hull, vertices/normals/edge directions all expressed in the frame of `body`.
struct Hull {
  std::string name;
  int body = 0;
  std::vector<Vec3> v;        // hull vertices
  std::vector<Vec3> normals;  // unique outward face normals
  std::vector<Vec3> edges;    // unique edge directions (unit)
  Vec3 center;                // bounding sphere
  double radius = 0;
};

struct Frame {
  std::string name;
  int body = 0;
  Pose pose;  // frame pose in the frame of `body`
};

class RobotModel {
 public:
  // Throws std::runtime_error with a readable message if the file is missing or malformed.
  static RobotModel loadFromFile(const std::string& path);

  std::string name;
  int ndof = 0;
  std::vector<Body> bodies;  // topological order, bodies[0] = world
  std::vector<Hull> hulls;
  std::vector<Frame> frames;

  int bodyIndex(const std::string& n) const;   // -1 if missing
  int frameIndex(const std::string& n) const;  // -1 if missing
  // bodies carrying dof 0..3 in order (the arm chain)
  std::array<int, kArmDof> armBodies() const;
};

}  // namespace arm
