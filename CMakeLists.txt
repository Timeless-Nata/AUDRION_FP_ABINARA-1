cmake_minimum_required(VERSION 3.0.2)
project(autonomous_manipulator)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_srvs)
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

catkin_package(CATKIN_DEPENDS roscpp rospy sensor_msgs std_srvs)

include_directories(${catkin_INCLUDE_DIRS} "${MUJOCO_DIR}/include")

add_executable(mujoco_sim_node src/simulation/mujoco_sim_node.cpp)
target_link_libraries(mujoco_sim_node ${catkin_LIBRARIES} ${MUJOCO_LIB} glfw)
set_target_properties(mujoco_sim_node PROPERTIES
  BUILD_RPATH "${MUJOCO_LIB_DIR}" INSTALL_RPATH "${MUJOCO_LIB_DIR}")

catkin_install_python(PROGRAMS scripts/test_joint_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config mjcf meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
