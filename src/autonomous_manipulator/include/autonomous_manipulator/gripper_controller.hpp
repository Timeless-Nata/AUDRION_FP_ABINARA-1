#pragma once
#include "autonomous_manipulator/robot_interface.hpp"

namespace am {

// Both fingers are driven with the same angle (0 = closed ring around the bar, 0.9 = fully open mouth).
class GripperController {
 public:
  GripperController(RobotInterface& robot, double open_angle, double closed_angle)
      : robot_(robot), open_(open_angle), closed_(closed_angle) {}
  bool open(double timeout_s = 3.0);
  bool close(double timeout_s = 3.0);
  void setAngles(double open_angle, double closed_angle) { open_ = open_angle; closed_ = closed_angle; }
  double openAngle() const { return open_; }

 private:
  bool moveTo(double angle, double timeout_s);
  RobotInterface& robot_;
  double open_, closed_;
};

}  // namespace am
