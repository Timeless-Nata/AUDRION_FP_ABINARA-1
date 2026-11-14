[INFO] [1794295774.486574330, 171.024000000]: [state] CLASSIFY_COLORS
[INFO] [1794295776.190694927, 172.718000000]: range probe: range 0.120 m -> hit (0.158, 0.313, z=0.074), 0.001 m from the colour blob
[INFO] [1794295776.190723390, 172.718000000]: candidate 'orange' at (0.158, 0.313): ultrasonic height 0.074 m -> upright object
[INFO] [1794295776.190742836, 172.718000000]: [state] SELECT_OBJECT
[INFO] [1794295776.190752754, 172.718000000]: select: slot 2 (level 1, orange) <- object 2 at (0.158, 0.313)
[INFO] [1794295776.190760905, 172.718000000]: [state] LOCALIZE_OBJECT
[WARN] [1794295777.653425338, 174.180000000]: localize: object is close to the base, viewed from x0.70 of the normal distance (position less accurate)
[INFO] [1794295778.394880269, 174.928000000]: localize: refined (0.1584,0.3132) -> (0.1552,0.2946)
[INFO] [1794295778.394914078, 174.928000000]: [state] CALCULATE_GRASP
[INFO] [1794295778.394938068, 174.928000000]: grasp plan: hip=1.098 XT=0.333 d_pre=0.030 tilt=0.30
[INFO] [1794295778.394947342, 174.928000000]: [state] EXECUTE_GRASP
[WARN] [1794295779.094083686, 175.632000000]: plan 'transit to pre-grasp' rejected by collision check: contact jaw_lower <-> dumbbell_6_orange (dist -0.000238)
[WARN] [1794295779.095462126, 175.632000000]: plan 'transit to pre-grasp' rejected by collision check: contact jaw_lower <-> dumbbell_6_orange (dist -0.000612)
[WARN] [1794295779.096941695, 175.634000000]: plan 'transit to pre-grasp' rejected by collision check: contact jaw_lower <-> dumbbell_6_orange (dist -0.000731)
[WARN] [1794295779.098300483, 175.636000000]: plan 'transit to pre-grasp' rejected by collision check: contact jaw_lower <-> dumbbell_6_orange (dist 0.000183)
[WARN] [1794295779.099790754, 175.638000000]: plan 'transit to pre-grasp' rejected by collision check: contact jaw_lower <-> dumbbell_6_orange (dist 0.000186)
[WARN] [1794295783.938231711, 180.476000000]: standoff: used +0 mm up, +40 mm back, tilt 0.40 (candidate 6)
[INFO] [1794295790.360647555, 186.898000001]: ultrasonic: range 0.0821 m -> bar-centre X 0.3450 (camera estimate 0.3329)
[INFO] [1794295790.360704982, 186.898000001]: ultrasonic: tracked object moved 12.1 mm radially to (0.1607, 0.3054)
[WARN] [1794295797.533401882, 194.070000001]: plan 'engage lift' rejected by collision check: contact jaw_upper <-> dumbbell_6_orange (dist -0.000117)
[WARN] [1794295797.545689108, 194.070000001]: plan 'engage lift' rejected by collision check: contact jaw_upper <-> dumbbell_6_orange (dist -0.000115)
[WARN] [1794295797.552565620, 194.088000001]: plan 'engage lift' rejected by collision check: contact jaw_upper <-> dumbbell_6_orange (dist -0.000115)
[WARN] [1794295797.556278124, 194.088000001]: plan 'engage lift' rejected by collision check: contact jaw_upper <-> dumbbell_6_orange (dist -0.000113)
[WARN] [1794295797.556297473, 194.088000001]: engage lift: jaw/bar contact is by design, lifting without the collision check


# autonomous_manipulator – ROS1 + MuJoCo pyramid stacking

Autonomous 3‑2‑1 pyramid (3 orange base, 2 green middle, 1 white top) built from the six dumbbells of your MJCF.
Sensors used for decisions: simulated colour camera + ultrasonic rangefinder + joint encoders/actuator torque.
Simulator ground truth is only printed in a validation report at the very end.

## 1. Directory structure (create it yourself, then copy the files in)

```
catkin_ws/
└── src/
    └── autonomous_manipulator/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── config/mission.yaml
        ├── launch/simulation.launch
        ├── models/arm_gripper_dumbbells.xml
        ├── msg/ObjectDetection.msg
        ├── msg/ObjectDetectionArray.msg
        ├── include/autonomous_manipulator/
        │   ├── math_utils.hpp
        │   ├── robot_model.hpp
        │   ├── geometry_utils.hpp
        │   ├── trajectory_planner.hpp
        │   ├── motion_planner.hpp
        │   ├── robot_interface.hpp
        │   └── gripper_controller.hpp
        ├── src/
        │   ├── forward_kinematics.cpp
        │   ├── inverse_kinematics.cpp
        │   ├── trajectory_planner.cpp
        │   ├── motion_planner.cpp
        │   ├── robot_interface.cpp
        │   ├── gripper_controller.cpp
        │   ├── mujoco_sim_node.cpp
        │   ├── perception_node.cpp
        │   └── mission_controller.cpp
        └── test/
            ├── test_kinematics.cpp
            └── test_mission_geometry.cpp
```

## 2. Dependencies (assumed: Ubuntu 20.04 + ROS Noetic, C++14)

```bash
sudo apt install ros-noetic-desktop-full libglfw3-dev libopencv-dev python3-catkin-tools build-essential
# MuJoCo 3.x: download the release tarball from https://github.com/google-deepmind/mujoco/releases and unpack, e.g.
mkdir -p ~/mujoco && tar -xzf mujoco-3.*-linux-x86_64.tar.gz -C ~/mujoco
export MUJOCO_DIR=~/mujoco/mujoco-3.x.x          # directory containing include/ and lib/
```

## 3. Build and run

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws && catkin_make -DMUJOCO_DIR=$MUJOCO_DIR        # or: catkin build --cmake-args -DMUJOCO_DIR=$MUJOCO_DIR
source ~/catkin_ws/devel/setup.bash

# offline sanity tests (no simulator needed) – run these FIRST
rosrun autonomous_manipulator test_kinematics
rosrun autonomous_manipulator test_mission_geometry

# the mission
roslaunch autonomous_manipulator simulation.launch          # gui:=false on a headless PC (needs xvfb-run for the GL context)
```
`roslaunch` starts `roscore` itself. The MuJoCo window opens, the arm scans the arc, then picks and stacks all six pieces.
A second MuJoCo viewer is not needed: `mujoco_sim_node` owns the only `mjModel/mjData` and also draws the window.

Debug helpers: `rqt_image_view /perception/debug_image`, `rostopic echo /mission/state`.

## 4. Honest status

| Item | Status |
|---|---|
| FK / analytic IK / quintic trajectories (`am_core`) | written, **compiled** (g++ ‑Wall ‑Wextra ‑Wpedantic), **unit-tested** offline: 10 338 FK→IK round trips, error 2e‑16 m; FK, camera pose and ultrasonic pose **cross-checked against an independent generic MJCF-tree evaluation** (Python) – match to 1e‑4 m |
| Reachability of every commanded pose (picks, stand-offs, 6 stack slots, hover/release/retract, 36 scan stations, verification views) | **audited offline** by `test_mission_geometry`: 0 problems |
| `mujoco_sim_node`, `perception_node`, `mission_controller`, `motion_planner`, `robot_interface`, `gripper_controller` | written and read through; brace/paren balanced; **NOT compiled and NOT run** (my sandbox has neither ROS nor MuJoCo nor network). Expect to fix a few compile errors on first build |
| Anything in MuJoCo (rendering, contacts, grasp, stacking, HSV thresholds, collision whitelist) | **UNTESTED**. Tuning will be needed – see §7 |

At start-up `mujoco_sim_node` compares my analytic FK with MuJoCo (end effector, camera, ultrasonic) and aborts with `ROS_FATAL` if they differ.

## 5. What I found in your model (and how the design follows from it)

* **The six dumbbells stand upright** (bar axis = world Z, 74 mm tall, plates 36×36×12 mm, 50 mm gap between the plates, bar radius 8 mm). A top‑down grasp is impossible; the grasp is a **horizontal, radial insertion** into the plate gap.
* **Correction of my earlier message:** the jaws do *not* open in the vertical plane. Both fingers rotate about the hand's Z axis (horizontal opening). Each finger is an 8‑box half ring; closed, the two halves form a **ring of inner radius 8.2 mm around a bar of radius 8.0 mm**, centred 1.0 cm in front of the finger pivot (= the `ee` site). So the `ee` site is the bar axis.
* The ring is only 16 mm tall (±8 mm) and fits the bar with 0.2 mm radial slack; finger joint limit is 0, so **the ring cannot squeeze the bar**. The physical grasp implemented here is therefore a **form‑closure "hook lift"**: the ring encloses the bar, and when the hand rises ~17 mm the ring's top face carries the top plate. It is *contact physics only* (no weld, no friction clamp). The hand must be level (±~1°) when the ring closes around the bar.
* With a level hand the arm can only reach radii 0.238–0.328 m at table height (wrist limit). The three nearest dumbbells (r = 0.26 m) force a **tilted (0.3 rad) stand‑off that is levelled during the last centimetres**.
* The camera (fovy 20°) is hand‑mounted and the `gripper_extension` box blocks the right ~⅓ of the image (columns > ~368 px). Perception masks those columns (`roi_u_max`), and scan poses aim the object at pixel column 170.

## 6. Architecture

| Node | Owns | Notes |
|---|---|---|
| `mujoco_sim_node` | the only `mjModel/mjData`, GL window | main thread steps/renders/publishes; ROS callbacks only store commands or queue closures run between steps; publishes `/clock` (all nodes use sim time) |
| `perception_node` | – | HSV segmentation → centroid of the lit **top face** → ray from camera pose (FK of measured joints) → plane z = 0.074 m. Frames taken while the arm moves are discarded |
| `mission_controller` | a *private* MuJoCo copy for collision checking | state machine, grasp/place planning, trajectories, verification |

Topics: `/clock`, `/joint_states` (pos, vel, effort = actuator force), `/ultrasonic/range`, `/color_cam/image_raw`, `/color_cam/camera_info`, `/arm/joint_command` (Float64MultiArray: hip, shoulder, arm, hand, finger1, finger2), `/perception/detections` (`ObjectDetectionArray`), `/perception/debug_image`, `/mission/state`, `/sim/ee_pose` and `/sim/debug/object_poses` (debug / validation only). Service: `/sim/grasp_weld` (`std_srvs/SetBool`, only used in `weld_assist` mode).

Use of the sensors: camera → colour and position (bearing from the pixel ray, ground-plane geometry for range). Ultrasonic → at the level pre‑grasp pose the beam (at plate height) must hit the top plate; its range corrects the insertion depth (±15 mm limit) – no echo or inconsistent range aborts the attempt. Joint torque → grasp verification.

State machine: INITIALIZATION → WAIT_FOR_SIMULATION → SCAN_WORKSPACE → CLASSIFY_COLORS → SELECT_OBJECT → LOCALIZE_OBJECT → CALCULATE_GRASP → EXECUTE_GRASP → VERIFY_GRASP → TRANSPORT_OBJECT → PLACE_OBJECT → VERIFY_PLACEMENT → (next) → MISSION_COMPLETE / MISSION_FAILED. Order: slots are filled level by level (3 orange, 2 green, 1 white).

Grasp sequence: open → transit to a safe pre‑grasp waypoint → descend to stand‑off → slide to level pre‑grasp (hand levels) → ultrasonic check/correction → slow final approach (bar enters the open ring) → close → small "engage" lift → full lift → **verify** (the shoulder torque must change by ≥ 40 % of m·g·lever, measured from actuator force) → raise → transport → lower → release → retract. A failed grasp opens the gripper, retreats, re‑scans and retries (`max_pick_retries`, then `MISSION_FAILED`); a failed placement verification stops the mission – it never continues after a failure.

Collision checking: every trajectory is sampled (≤ 2 cm or 50 ms) and checked with MuJoCo's collision pipeline on a private model against the believed scene (remaining dumbbells, placed pieces, the held piece). Contacts that already exist in the neutral pose of the model are whitelisted at start-up (check the log line "geom pairs ... whitelisted"). Jaw–bar contact is allowed only for the target during the grasp.

## 7. Choices you should know about

* **Colours:** pyramid = 3 orange / 2 green / 1 white (matches the six bodies in your XML; no recolouring). Change `level_colors` / `level_counts` in `config/mission.yaml`.
* **Stack location:** base row centred at (0.059, 0.335) m (radius 0.34 m, azimuth +80°): the only free sector beyond the object arc that all levels can reach with margin and where the arm does not sweep over the remaining dumbbells (screened offline). Slots: pitch 46 mm (10 mm gap), level‑2 plates overlap the base plates by 13 mm. Heights come from the object geometry (piece 74 mm). Everything is in `config/mission.yaml`.
* **Hip joint limit:** the outermost slots need hip ≈ 1.53 rad of the ±1.5708 limit (margin kept: 0.02 rad).
* **MJCF modification:** only an `<equality>` block of six **inactive** weld constraints (used solely if `grasp_mode: weld_assist`; the relative pose is frozen at activation, nothing teleports). Worldbody, actuators, sensors, defaults are byte-identical to yours. Default `grasp_mode: contact` = real contact grasping. `weld_assist` is a simulation‑assisted constraint and logs a loud warning.

## 8. Things most likely to need tuning on first run
1. HSV ranges (`perception/colors`) and `top_face_v_ratio`; look at `/perception/debug_image`.
2. `planner: ... rejected by collision check` messages: false positives from the neutral‑pose whitelist or `collision_margin` (2 mm).
3. Hook lift: if verification fails, increase `lift_engage`, check `grasp_z_offset`, or try `grasp_mode: weld_assist` to separate grasp problems from the rest.
4. Position‑actuator lag: lower `cart_speed_*` / `joint_vmax` if tracking errors appear.
5. The ultrasonic gate fails if the camera estimate is off by more than ~18 mm laterally (beam misses the plate).
