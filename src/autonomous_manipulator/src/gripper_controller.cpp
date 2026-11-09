#include "autonomous_manipulator/gripper_controller.hpp"

#include <cmath>

namespace am {

bool GripperController::open(double timeout_s) { return moveTo(open_, timeout_s); }
bool GripperController::close(double timeout_s) { return moveTo(closed_, timeout_s); }

// Command the fingers and wait until both have stopped moving (reached the target or blocked by the bar).
bool GripperController::moveTo(double angle, double timeout_s) {
  robot_.setFingerCommand(angle);
  ros::Rate rate(100.0);
  ros::Time t0 = ros::Time::now();
  int still = 0;
  while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout_s) {
    robot_.hold(0.0);
    auto v = robot_.fingerQd();
    still = (std::fabs(v[0]) < 0.02 && std::fabs(v[1]) < 0.02 && (ros::Time::now() - t0).toSec() > 0.3) ? still + 1 : 0;
    if (still >= 30) return true;
    robot_.hold(0.01);
    rate.sleep();
  }
  return ros::ok();
}

}  // namespace am
