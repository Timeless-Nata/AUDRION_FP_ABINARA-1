#include "autonomous_manipulator/executor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace arm {

ExecResult executeTrajectory(const Trajectory& traj, RobotInterface& robot, Clock& clock, const ExecOptions& opt,
                             const std::function<bool()>& cancel) {
  ExecResult res;
  if (traj.empty()) {
    res.message = "empty trajectory";
    return res;
  }
  const double dt = 1.0 / opt.rate_hz;
  JointVec trim{};
  JointVec q_ref = traj.start(), qd{}, meas{};

  auto step = [&](const JointVec& ref) {  // measure, update trim, command; returns false if tracking is lost
    if (!robot.readJoints(meas)) return true;  // no measurement: open loop
    for (int i = 0; i < kArmDof; ++i) {
      if (std::fabs(ref[i] - meas[i]) > opt.max_tracking_error) return false;
      trim[i] = std::min(opt.trim_limit, std::max(-opt.trim_limit, trim[i] + opt.integral_gain * (ref[i] - meas[i]) * dt));
    }
    return true;
  };
  auto command = [&](const JointVec& ref) {
    JointVec c;
    for (int i = 0; i < kArmDof; ++i) c[i] = ref[i] + trim[i];
    robot.sendJoints(c);
  };

  const double t0 = clock.now();
  for (;;) {
    const double t = clock.now() - t0;
    traj.sample(t, q_ref, qd);
    if (cancel && cancel()) {
      command(q_ref);
      res.message = "cancelled";
      return res;
    }
    if (!step(q_ref)) {
      res.message = "tracking error too large: the arm does not follow the command (blocked or collided?)";
      return res;
    }
    command(q_ref);
    if (t >= traj.duration()) break;
    clock.sleep(dt);
  }

  // settle on the goal
  const JointVec goal = traj.goal();
  double ok_since = -1, t_settle = clock.now();
  for (;;) {
    double err = 0;
    if (robot.readJoints(meas)) {
      for (int i = 0; i < kArmDof; ++i) err = std::max(err, std::fabs(goal[i] - meas[i]));
    }
    res.max_error = err;
    res.final_q = meas;
    step(goal);
    command(goal);
    const double now = clock.now();
    if (err <= opt.settle_tol) {
      if (ok_since < 0) ok_since = now;
      if (now - ok_since >= opt.settle_time) {
        res.ok = true;
        res.message = "ok";
        return res;
      }
    } else {
      ok_since = -1;
    }
    if (now - t_settle > opt.settle_timeout) {
      char b[160];
      std::snprintf(b, sizeof b, "did not settle: %.4f rad from the goal after %.1f s", err, opt.settle_timeout);
      res.message = b;
      return res;
    }
    clock.sleep(dt);
  }
}

}  // namespace arm
