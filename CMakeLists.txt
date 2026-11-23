cmake_minimum_required(VERSION 3.0.2)
project(autonomous_manipulator)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_srvs std_msgs geometry_msgs message_generation)
find_package(glfw3 REQUIRED)   # sudo apt install libglfw3-dev

# ---- MuJoCo location -------------------------------------------------------
# Pass -DMUJOCO_DIR=/home/<you>/mujoco-3.x.y to catkin_make to choose explicitly.
# Otherwise the newest $HOME/mujoco* or $HOME/.mujoco/mujoco* directory that
# contains include/mujoco/mujoco.h is used.
if(NOT MUJOCO_DIR)
  file(GLOB _mj_candidates LIST_DIRECTORIES true
       "$ENV{HOME}/mujoco*" "$ENV{HOME}/.mujoco/mujoco*")
  list(SORT _mj_candidates)
  list(REVERSE _mj_candidates)
  foreach(_c ${_mj_candidates})
    if(EXISTS "${_c}/include/mujoco/mujoco.h")
      set(MUJOCO_DIR "${_c}" CACHE PATH "MuJoCo install directory (has include/ and lib/)")
      break()
    endif()
  endforeach()
endif()
if(NOT MUJOCO_DIR)
  message(FATAL_ERROR "MuJoCo not found. Re-run with -DMUJOCO_DIR=/path/to/mujoco-x.y.z "
                      "(the folder that contains include/mujoco/mujoco.h and lib/libmujoco.so).")
endif()
find_library(MUJOCO_LIB NAMES mujoco
             HINTS "${MUJOCO_DIR}/lib" "${MUJOCO_DIR}/build/lib"
             NO_DEFAULT_PATH)
if(NOT MUJOCO_LIB)
  message(FATAL_ERROR "libmujoco.so not found in ${MUJOCO_DIR}/lib or ${MUJOCO_DIR}/build/lib")
endif()
get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB}" DIRECTORY)
message(STATUS "MuJoCo: ${MUJOCO_DIR} (${MUJOCO_LIB})")

catkin_python_setup()   # Phase 3: python package src/autonomous_manipulator (kinematics, trajectory, motion_control); must precede generate_messages

add_message_files(FILES DetectedObject.msg DetectedObjectArray.msg)
add_service_files(FILES MoveToPose.srv MoveJoints.srv ValidateColor.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy sensor_msgs std_srvs std_msgs geometry_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS} "${MUJOCO_DIR}/include")

# ---- Phase 3a/3b: arm core (pure C++17, no ROS / MuJoCo) -------------------------------------------------------
add_library(arm_core STATIC
  src/control/robot_model.cpp
  src/control/kinematics.cpp
  src/control/collision_checker.cpp
  src/control/inverse_kinematics.cpp
  src/control/trajectory.cpp
  src/control/motion_planner.cpp
  src/control/executor.cpp
  src/control/color_classifier.cpp
  src/control/view_planner.cpp
  src/control/view_search.cpp)
target_include_directories(arm_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(arm_core_cli src/tools/arm_core_cli.cpp)
target_link_libraries(arm_core_cli arm_core)

add_executable(arm_core_test test/test_core.cpp)
target_link_libraries(arm_core_test arm_core)

add_executable(arm_motion_test test/test_motion.cpp)
target_link_libraries(arm_motion_test arm_core)

add_executable(arm_color_test test/test_color.cpp)
target_link_libraries(arm_color_test arm_core)

add_executable(arm_view_test test/test_view.cpp)
target_link_libraries(arm_view_test arm_core)

# C++ motion node (replaces scripts/motion_control_node.py; both offer /motion/move_to_pose and /motion/move_joints)
add_executable(motion_node src/control/motion_node.cpp)
add_dependencies(motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_node arm_core ${catkin_LIBRARIES})

# C++ colour-validation node (Phase 4b): finds a view that shows one colour; moves the arm through motion_node
add_executable(color_view_node src/control/color_view_node.cpp)
add_dependencies(color_view_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_view_node arm_core ${catkin_LIBRARIES})

add_executable(mujoco_sim_node src/simulation/mujoco_sim_node.cpp)
target_link_libraries(mujoco_sim_node ${catkin_LIBRARIES} ${MUJOCO_LIB} glfw)
set_target_properties(mujoco_sim_node PROPERTIES
  BUILD_RPATH "${MUJOCO_LIB_DIR}" INSTALL_RPATH "${MUJOCO_LIB_DIR}")

catkin_install_python(PROGRAMS scripts/test_joint_control.py scripts/perception_node.py scripts/test_perception.py scripts/test_kinematics.py scripts/test_reach_targets.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config mjcf meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
