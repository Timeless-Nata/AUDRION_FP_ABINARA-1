# Phase 3 - Kinematics and Motion Control

## Build and run
    cd ~/catkin_ws && catkin_make          # new: setup.py, 2 services (MoveToPose, MoveJoints)
    roslaunch autonomous_manipulator simulation.launch      # now also starts motion_control
    rosrun autonomous_manipulator test_kinematics.py        # offline checks (no sim needed)
    rosrun autonomous_manipulator test_reach_targets.py     # live checks against MuJoCo

## What was added
| File | Purpose |
|---|---|
| `src/autonomous_manipulator/kinematics.py` | FK, Jacobian, joint limits, closed-form IK + DLS polish, gravity model. Everything is **parsed from the MJCF** (no copied numbers). |
| `src/autonomous_manipulator/trajectory.py` | Quintic (min-jerk) joint trajectories with v/a limits; straight-line Cartesian trajectories with IK at every sample. |
| `src/autonomous_manipulator/motion_control.py` | Planning + validation + execution (gravity feed-forward, integral trim, settle detection). No ROS imports. |
| `scripts/motion_control_node.py` | ROS wrapper. |
| `srv/MoveToPose.srv`, `srv/MoveJoints.srv` | Interfaces. |
| `config/kinematics_config.yaml` | Limits, margins, gravity comp, tolerances. |

## ROS interface
- `/motion/move_to_pose` (MoveToPose): world-frame target, `MODE_POSE` (position + pitch from quaternion) or `MODE_POSITION` (position, pitch chosen near `prefer_pitch`), `linear` = straight line, `dry_run`.
- `/motion/move_joints` (MoveJoints), `/motion/go_home`, `/motion/stop`.
- `/motion/tool_pose` (PoseStamped, world): FK of the measured joints at 20 Hz.
- Commands go to `/arm/joint_command` and contain only joint_1..4 (gripper untouched).

    rosservice call /motion/move_to_pose "target: {header: {frame_id: world}, pose: {position: {x: 0.289, y: 0.0, z: 0.0305}}}
    mode: 1
    prefer_pitch: 0.0
    linear: true
    duration: 0.0
    dry_run: false"

## Conventions
- Tool point = MJCF site `grasp_center`. Tool frame = ROS convention: x approach, y left, z up. Identity = horizontal gripper along +x; **positive rotation about y pitches down**.
- 4 DOF => only position (3) + pitch (1) are controllable. Tool yaw follows the base; roll is fixed. For `MODE_POSE`, `orient_error` reports the part that cannot be realised (about 0.011 rad = the 3 mm lateral offset of `grasp_center`).
- Pitch = elevation of the tool x axis (0 horizontal, negative down).

## Findings about this robot (from the MJCF)
- joint_1 = yaw; joints 2-4 are three parallel pitch axes (planar 3R). Link lengths 0.149 / 0.211 m, grasp_center 0.095 m beyond joint_4.
- **All six dumbbells lie on one arc, r = 0.289 m from the base axis.** At z = 0.0305 the reachable tool pitch there is -49 deg .. +72 deg, so **top-down grasps are impossible; use a horizontal side grasp** (pitch about 0). Maximum horizontal reach at pick height is about 0.40 m.
- Gravity: position servos sag. Shoulder holds 0.185 N m at home (2.3 deg, about 1.6 cm at the tool) without compensation. Worst-case holding torque in the joint box is 0.595 N m against the 0.93 N m force limit (margin x1.56). A held dumbbell (0.03 kg) adds roughly 0.09 N m at the shoulder: set `payload_kg: 0.03` in Phase 5.
- Servos have a friction dead-band of about 4 mrad (frictionloss 0.02 / kp 4.65), which limits the best achievable accuracy to about 1-2 mm.

## Verification status (be aware)
- Verified here, offline: parsing, FK home pose, Jacobian vs finite differences (2e-7), gravity torque vs -dU/dq (2e-10), IK round trip (742/742, 0.002 mm), limit/unreachable handling, trajectory v/a limits, line straightness (0.03 mm), all six dumbbells reachable incl. 6 cm pre-grasp. The controller + node logic was exercised against a simulated servo plant (P-servo, gravity, friction): about 1 mm tool error with compensation, 35 mm without.
- **NOT verified here: anything inside real MuJoCo/ROS.** `test_reach_targets.py` does that: it compares my FK against the camera pose and HC-SR04 pose that MuJoCo publishes (catches axis/sign mistakes), reaches every dumbbell, and checks contacts via `/sim/contact_report`. Please run it and send me the output.
- Collision avoidance is limited to height checks (tool / elbow / wrist above the floor); real contacts are only reported by the Phase 2 contact monitor.
- IK covers the arm in front of the base axis (tool pointing away from the base); folded-back poses are out of scope.

## Real robot (Phase 7)
No servo feedback exists on hardware: set `gravity_comp` according to the real arm and `integral_gain: 0`.
