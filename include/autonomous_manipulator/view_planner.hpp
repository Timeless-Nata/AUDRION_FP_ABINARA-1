// view_planner.hpp - arm poses from which the wrist camera looks at ONE dumbbell (pure C++17).
//
// The camera only validates a colour; where the dumbbell is comes from the HC-SR04 range and the arm pose. This
// module answers: "which joint angles put the camera `standoff` metres from the point `target`, looking at it with a
// downward pitch, optionally shifted sideways, without making links overlap?"
#pragma once
#include <string>

#include "autonomous_manipulator/inverse_kinematics.hpp"

namespace arm {

// Pinhole model of the MJCF camera "esp32cam" (looks along -z of its frame, +y up, +x right).
struct CameraModel {
  double fovy_deg = 60.0;
  int width = 640, height = 480;
  // Pixel of a world point; false if it is behind the camera.
  bool project(const Pose& cam, Vec3 p, double& u, double& v) const;
  Vec3 forward(const Pose& cam) const { return {-cam.R(0, 2), -cam.R(1, 2), -cam.R(2, 2)}; }
  Vec3 right(const Pose& cam) const { return {cam.R(0, 0), cam.R(1, 0), cam.R(2, 0)}; }
};

struct ViewParams {
  std::string camera_frame = "cam_esp32cam";
  double finger_angle = -0.7;   // gripper opening assumed in collision checks
  double joint_margin = 0.05;   // [rad] kept away from the joint limits (observation poses are near the q4 limit)
  double max_pos_error = 0.003; // [m] camera position error accepted
  double max_pitch_error_deg = 1.5;
  // What-if: extra tilt of the camera relative to its current mount [deg, negative = further down], rotating about the
  // joint axis through the camera origin. 0 = the camera as modelled in arm.xml.
  double camera_tilt_deg = 0.0;
};

struct ViewPose {
  JointVec q{};
  Pose cam;              // camera pose in the world
  Vec3 aim;              // point the optical axis passes through
  double standoff = 0;   // [m] distance camera -> aim along the optical axis
  double cam_pitch = 0;  // [rad] elevation of the optical axis (negative = looking down)
  double lateral = 0;    // [m] shift of the aim point, + = to the left of the radial direction as seen from the base
};

class ViewPlanner {
 public:
  ViewPlanner(const Kinematics& kin, const CollisionChecker& chk, const InverseKinematics& ik, const ViewParams& p = ViewParams())
      : kin_(kin), chk_(chk), ik_(ik), p_(p) {}

  // target: the dumbbell centre. cam_pitch: elevation of the optical axis [rad], negative = down.
  // lateral: the optical axis aims at target + lateral * (left unit vector), so the dumbbell appears shifted the opposite way.
  bool plan(Vec3 target, double standoff, double cam_pitch, double lateral, const JointVec& seed, ViewPose& out,
            std::string* why = nullptr) const;

  // camera pose including the optional what-if tilt
  Pose cameraPose(const JointVec& q) const;
  // elevation of the camera optical axis for given joints
  double cameraPitch(const JointVec& q) const;

 private:
  const Kinematics& kin_;
  const CollisionChecker& chk_;
  const InverseKinematics& ik_;
  ViewParams p_;
};

}  // namespace arm
