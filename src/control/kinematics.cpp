#include "autonomous_manipulator/kinematics.hpp"

#include <stdexcept>

namespace arm {

Kinematics::Kinematics(const RobotModel& model) : model_(&model), arm_(model.armBodies()), tool_frame_(model.frameIndex("tool")) {
  for (int i = 0; i < kArmDof; ++i) {
    lo_[i] = model.bodies[arm_[i]].lo;
    hi_[i] = model.bodies[arm_[i]].hi;
  }
}

std::vector<Pose> Kinematics::bodyPoses(const DofVec& dofs) const {
  std::vector<Pose> W(model_->bodies.size());
  for (size_t i = 0; i < model_->bodies.size(); ++i) {
    const Body& b = model_->bodies[i];
    Pose T = b.fixed;
    if (b.dof >= 0) T.R = T.R * rotAxis(b.axis, b.coef * dofs[b.dof]);
    W[i] = (b.parent >= 0) ? W[b.parent] * T : T;
  }
  return W;
}

Pose Kinematics::framePose(const std::string& name, const JointVec& q, double finger_active) const {
  const int f = model_->frameIndex(name);
  if (f < 0) throw std::runtime_error("unknown frame '" + name + "'");
  DofVec d{q[0], q[1], q[2], q[3], finger_active};
  const std::vector<Pose> W = bodyPoses(d);
  return W[model_->frames[f].body] * model_->frames[f].pose;
}

Pose Kinematics::toolPose(const JointVec& q) const {
  DofVec d{q[0], q[1], q[2], q[3], 0.0};
  const std::vector<Pose> W = bodyPoses(d);
  return W[model_->frames[tool_frame_].body] * model_->frames[tool_frame_].pose;
}

double Kinematics::pitchOf(const Pose& tool) {
  const Vec3 x = tool.R.col(0);
  return std::asin(clampd(x.z, -1.0, 1.0));
}

Jacobian Kinematics::toolJacobian(const JointVec& q) const {
  DofVec d{q[0], q[1], q[2], q[3], 0.0};
  const std::vector<Pose> W = bodyPoses(d);
  const Pose tool = W[model_->frames[tool_frame_].body] * model_->frames[tool_frame_].pose;
  Jacobian J{};
  for (int i = 0; i < kArmDof; ++i) {
    const Body& b = model_->bodies[arm_[i]];
    const Pose pre = W[b.parent] * b.fixed;  // joint frame before rotating (axis and origin do not move with the joint)
    const Vec3 w = pre.R * b.axis;
    const Vec3 v = cross(w, tool.p - pre.p);
    J.J[0][i] = v.x; J.J[1][i] = v.y; J.J[2][i] = v.z;
    J.J[3][i] = w.x; J.J[4][i] = w.y; J.J[5][i] = w.z;
  }
  return J;
}

JointVec Kinematics::pitchGradient(const JointVec& q) const {
  DofVec d{q[0], q[1], q[2], q[3], 0.0};
  const std::vector<Pose> W = bodyPoses(d);
  const Pose tool = W[model_->frames[tool_frame_].body] * model_->frames[tool_frame_].pose;
  const Vec3 x = tool.R.col(0);
  const double s = std::sqrt(std::fmax(1e-9, 1.0 - x.z * x.z));
  JointVec g{};
  for (int i = 0; i < kArmDof; ++i) {
    const Body& b = model_->bodies[arm_[i]];
    const Pose pre = W[b.parent] * b.fixed;
    g[i] = cross(pre.R * b.axis, x).z / s;
  }
  return g;
}

bool Kinematics::withinLimits(const JointVec& q, double margin) const {
  for (int i = 0; i < kArmDof; ++i)
    if (q[i] < lo_[i] + margin || q[i] > hi_[i] - margin) return false;
  return true;
}

JointVec Kinematics::clampToLimits(const JointVec& q, double margin) const {
  JointVec o = q;
  for (int i = 0; i < kArmDof; ++i) o[i] = clampd(q[i], lo_[i] + margin, hi_[i] - margin);
  return o;
}

}  // namespace arm
