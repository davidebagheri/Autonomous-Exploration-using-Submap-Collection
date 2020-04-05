# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "active_3d_planning_app_reconstruction: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iactive_3d_planning_app_reconstruction:/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg;-Iroscpp:/opt/ros/melodic/share/roscpp/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Imav_msgs:/home/davide/catkin_ws/src/mav_comm/mav_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_custom_target(_active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "active_3d_planning_app_reconstruction" "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(active_3d_planning_app_reconstruction
  "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_3d_planning_app_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_cpp(active_3d_planning_app_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_3d_planning_app_reconstruction
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages active_3d_planning_app_reconstruction_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp _active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_3d_planning_app_reconstruction_gencpp)
add_dependencies(active_3d_planning_app_reconstruction_gencpp active_3d_planning_app_reconstruction_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_3d_planning_app_reconstruction_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(active_3d_planning_app_reconstruction
  "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_3d_planning_app_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_eus(active_3d_planning_app_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_3d_planning_app_reconstruction
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages active_3d_planning_app_reconstruction_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus _active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_3d_planning_app_reconstruction_geneus)
add_dependencies(active_3d_planning_app_reconstruction_geneus active_3d_planning_app_reconstruction_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_3d_planning_app_reconstruction_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(active_3d_planning_app_reconstruction
  "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_3d_planning_app_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_lisp(active_3d_planning_app_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_3d_planning_app_reconstruction
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages active_3d_planning_app_reconstruction_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp _active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_3d_planning_app_reconstruction_genlisp)
add_dependencies(active_3d_planning_app_reconstruction_genlisp active_3d_planning_app_reconstruction_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_3d_planning_app_reconstruction_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(active_3d_planning_app_reconstruction
  "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_3d_planning_app_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_nodejs(active_3d_planning_app_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_3d_planning_app_reconstruction
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages active_3d_planning_app_reconstruction_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs _active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_3d_planning_app_reconstruction_gennodejs)
add_dependencies(active_3d_planning_app_reconstruction_gennodejs active_3d_planning_app_reconstruction_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_3d_planning_app_reconstruction_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(active_3d_planning_app_reconstruction
  "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_3d_planning_app_reconstruction
)

### Generating Services

### Generating Module File
_generate_module_py(active_3d_planning_app_reconstruction
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_3d_planning_app_reconstruction
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(active_3d_planning_app_reconstruction_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages active_3d_planning_app_reconstruction_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davide/catkin_ws/src/voxgraph_planner/active_3d_planning_app_reconstruction/msg/OdometryOffset.msg" NAME_WE)
add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py _active_3d_planning_app_reconstruction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(active_3d_planning_app_reconstruction_genpy)
add_dependencies(active_3d_planning_app_reconstruction_genpy active_3d_planning_app_reconstruction_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS active_3d_planning_app_reconstruction_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_3d_planning_app_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/active_3d_planning_app_reconstruction
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp roscpp_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET mav_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp mav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_3d_planning_app_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/active_3d_planning_app_reconstruction
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus roscpp_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()
if(TARGET mav_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus mav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_3d_planning_app_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/active_3d_planning_app_reconstruction
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp roscpp_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET mav_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp mav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_3d_planning_app_reconstruction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/active_3d_planning_app_reconstruction
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs roscpp_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()
if(TARGET mav_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs mav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_3d_planning_app_reconstruction)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_3d_planning_app_reconstruction\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/active_3d_planning_app_reconstruction
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py roscpp_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET mav_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py mav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(active_3d_planning_app_reconstruction_generate_messages_py nav_msgs_generate_messages_py)
endif()
