// executor.hpp - plays a Trajectory on a robot through two small abstract interfaces.
//
// RobotInterface is the ONLY thing that differs between simulation and hardware:
//   simulation: ROS topics of the MuJoCo node;  hardware: the STM32 serial link (later phase).
// Nothing in the planner / executor knows about MuJoCo, ROS or the STM32.
#pragma once
#include <functional>
#include <string>

#include "autonomous_manipulator/trajectory.hpp"

namespace arm {

class RobotInterface {
 public:
  virtual ~RobotInterface() = default;
  // Latest measured arm joint angles. Hardware servos report nothing: return the last commanded values.
  virtual bool readJoints(JointVec& q) = 0;
  virtual void sendJoints(const JointVec& q) = 0;  // position command for joint_1..4 [rad]
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() = 0;               // [s]
  virtual void sleep(double seconds) = 0;
};

struct ExecOptions {
  double rate_hz = 100.0;
  double settle_tol = 0.004;       // [rad] all joints within this of the goal ...
  double settle_time = 0.25;       // [s]   ... for this long
  double settle_timeout = 3.0;     // [s]   give up after this
  double integral_gain = 0.0;      // [1/s] trims the command from measured error (simulation P-servo sag); 0 = off
  double trim_limit = 0.10;        // [rad] max |trim| per joint
  double max_tracking_error = 0.6; // [rad] abort if the measured joint is this far from the reference
};

struct ExecResult {
  bool ok = false;
  std::string message;
  JointVec final_q{};
  double max_error = 0;  // [rad] |goal - measured| after settling
};

// `cancel` (optional) is polled every cycle; returning true aborts the move and holds the current command.
ExecResult executeTrajectory(const Trajectory& traj, RobotInterface& robot, Clock& clock, const ExecOptions& opt = ExecOptions(),
                             const std::function<bool()>& cancel = nullptr);

}  // namespace arm
