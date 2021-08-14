# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mavros_msgs: 3 messages, 3 services")

set(MSG_I_FLAGS "-Imavros_msgs:/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg;-Istd_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg;-Istd_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mavros_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv" ""
)

get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv" ""
)

get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv" ""
)

get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg" NAME_WE)
add_custom_target(_mavros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mavros_msgs" "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)
_generate_msg_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)
_generate_msg_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)

### Generating Services
_generate_srv_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)
_generate_srv_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)
_generate_srv_cpp(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
)

### Generating Module File
_generate_module_cpp(mavros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mavros_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mavros_msgs_generate_messages mavros_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_cpp _mavros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mavros_msgs_gencpp)
add_dependencies(mavros_msgs_gencpp mavros_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mavros_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)
_generate_msg_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)
_generate_msg_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)

### Generating Services
_generate_srv_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)
_generate_srv_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)
_generate_srv_py(mavros_msgs
  "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
)

### Generating Module File
_generate_module_py(mavros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mavros_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mavros_msgs_generate_messages mavros_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandBool.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/CommandTOL.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/ActuatorControl.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/srv/SetMode.srv" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/Thrust.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg/AttitudeTarget.msg" NAME_WE)
add_dependencies(mavros_msgs_generate_messages_py _mavros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mavros_msgs_genpy)
add_dependencies(mavros_msgs_genpy mavros_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mavros_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mavros_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mavros_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mavros_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mavros_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs)
  install(CODE "execute_process(COMMAND \"/home/lowjunen/.matlab/R2020b/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mavros_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mavros_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mavros_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mavros_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
