cmake_minimum_required(VERSION 3.0.2)
project(autonomous_manipulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs geometry_msgs sensor_msgs rosgraph_msgs message_generation)
find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)

# ---- MuJoCo (pass -DMUJOCO_DIR=/path/to/mujoco-3.x.x  or  export MUJOCO_DIR=...) ----
set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo installation directory (contains include/ and lib/)")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS ${MUJOCO_DIR}/include /usr/local/include /usr/include)
find_library(MUJOCO_LIBRARY NAMES mujoco HINTS ${MUJOCO_DIR}/lib /usr/local/lib /usr/lib)
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo not found. Re-run with -DMUJOCO_DIR=/path/to/mujoco-3.x.x")
endif()
get_filename_component(MUJOCO_LIB_DIR ${MUJOCO_LIBRARY} DIRECTORY)
set(CMAKE_BUILD_RPATH ${MUJOCO_LIB_DIR})
set(CMAKE_INSTALL_RPATH ${MUJOCO_LIB_DIR})

add_message_files(FILES ObjectDetection.msg ObjectDetectionArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs sensor_msgs rosgraph_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${MUJOCO_INCLUDE_DIR})

# Pure C++ core (no ROS / MuJoCo): kinematics, IK, trajectories
add_library(am_core src/forward_kinematics.cpp src/inverse_kinematics.cpp src/trajectory_planner.cpp)

add_executable(mujoco_sim_node src/mujoco_sim_node.cpp)
target_link_libraries(mujoco_sim_node am_core ${catkin_LIBRARIES} ${MUJOCO_LIBRARY} glfw)
add_dependencies(mujoco_sim_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node am_core ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(perception_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(mission_controller src/mission_controller.cpp src/motion_planner.cpp src/gripper_controller.cpp src/robot_interface.cpp)
target_link_libraries(mission_controller am_core ${catkin_LIBRARIES} ${MUJOCO_LIBRARY})
add_dependencies(mission_controller ${PROJECT_NAME}_generate_messages_cpp)

# Stand-alone kinematics test:  rosrun autonomous_manipulator test_kinematics
add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics am_core)

# Offline reachability audit of every pose the mission commands:  rosrun autonomous_manipulator test_mission_geometry
add_executable(test_mission_geometry test/test_mission_geometry.cpp)
target_link_libraries(test_mission_geometry am_core)
