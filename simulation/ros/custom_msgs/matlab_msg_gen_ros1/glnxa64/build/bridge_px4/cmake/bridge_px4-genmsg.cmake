# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bridge_px4: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bridge_px4_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv" NAME_WE)
add_custom_target(_bridge_px4_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bridge_px4" "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(bridge_px4
  "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bridge_px4
)

### Generating Module File
_generate_module_cpp(bridge_px4
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bridge_px4
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bridge_px4_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bridge_px4_generate_messages bridge_px4_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv" NAME_WE)
add_dependencies(bridge_px4_generate_messages_cpp _bridge_px4_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bridge_px4_gencpp)
add_dependencies(bridge_px4_gencpp bridge_px4_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bridge_px4_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(bridge_px4
  "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bridge_px4
)

### Generating Module File
_generate_module_py(bridge_px4
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bridge_px4
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bridge_px4_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bridge_px4_generate_messages bridge_px4_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/simulation/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/bridge_px4/srv/TrajTransfer.srv" NAME_WE)
add_dependencies(bridge_px4_generate_messages_py _bridge_px4_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bridge_px4_genpy)
add_dependencies(bridge_px4_genpy bridge_px4_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bridge_px4_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bridge_px4)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bridge_px4
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bridge_px4_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bridge_px4)
  install(CODE "execute_process(COMMAND \"/home/lowjunen/.matlab/R2020b/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bridge_px4\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bridge_px4
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bridge_px4_generate_messages_py std_msgs_generate_messages_py)
endif()
