// inverse_kinematics.hpp - collision-aware IK for the 4-DOF arm (yaw + 3 parallel pitch joints).
//
// 4 DOF => the tool can be placed at a 3D position AND a pitch (elevation of the approach axis); tool yaw
// follows the base and roll is fixed. The solver is numerical (damped least squares on position + pitch with
// the analytic Jacobian) started from the given seed AND from a grid of configurations, so it finds every
// distinct arm posture (elbow up / down, ...). Each solution is checked against the joint limits and, when a
// CollisionChecker is supplied, against link overlap; collision-free solutions are preferred.
#pragma once
#include <vector>

#include "autonomous_manipulator/collision_checker.hpp"
#include "autonomous_manipulator/kinematics.hpp"

namespace arm {

enum class IKStatus {
  Ok,               // a solution within limits (and collision-free if a checker is set) was found
  NoSolution,       // position/pitch not reachable within the joint limits
  AllInCollision    // reachable, but every solution makes links overlap (best colliding one is returned)
};

struct IKOptions {
  double pos_tol = 1e-4;          // [m]
  double pitch_tol = 1e-3;        // [rad]
  int max_iter = 100;
  double damping = 0.01;          // lambda of the damped least squares
  double max_step = 0.3;          // [rad] per iteration and joint
  double pitch_weight = 0.1;      // [m/rad] weight of the pitch error relative to position errors
  double joint_margin = 0.02;     // [rad] stay this far inside the joint limits
  bool seed_grid = true;          // also start from a grid of postures (finds elbow up/down etc.)
  double finger_angle = -0.7;     // finger_active angle assumed for the collision check (open gripper)
};

struct IKSolution {
  JointVec q{};
  double pos_err = 0;       // [m]
  double pitch_err = 0;     // [rad]
  bool collision_free = true;
  double depth = 0;         // deepest overlap [m] when not collision free
  std::string first_pair;   // "hullA <-> hullB" of the deepest overlap
};

class InverseKinematics {
 public:
  // `checker` may be nullptr (then only joint limits are enforced).
  InverseKinematics(const Kinematics& kin, const CollisionChecker* checker = nullptr) : kin_(kin), checker_(checker) {}

  // All distinct solutions within limits, collision-free ones first, then by distance to `seed`.
  std::vector<IKSolution> solveAll(Vec3 p, double pitch, const JointVec& seed, const IKOptions& opt = IKOptions()) const;

  IKStatus solve(Vec3 p, double pitch, const JointVec& seed, const IKOptions& opt, IKSolution& best) const;

  // Position only: tries the pitch candidates in order (e.g. {0, +5deg, -5deg, ...}), returns the first Ok one.
  IKStatus solvePosition(Vec3 p, const std::vector<double>& pitch_candidates, const JointVec& seed, const IKOptions& opt,
                         IKSolution& best, double* pitch_used = nullptr) const;

 private:
  bool refine(JointVec& q, Vec3 p, double pitch, const IKOptions& opt) const;
  const Kinematics& kin_;
  const CollisionChecker* checker_;
};

}  // namespace arm
