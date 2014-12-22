# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sb_msgs: 12 messages, 0 services")

set(MSG_I_FLAGS "-Isb_msgs:/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sb_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg" ""
)

get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_sb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_msgs" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)
_generate_msg_cpp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(sb_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sb_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sb_msgs_generate_messages sb_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_cpp _sb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_msgs_gencpp)
add_dependencies(sb_msgs_gencpp sb_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)
_generate_msg_lisp(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(sb_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sb_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sb_msgs_generate_messages sb_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_lisp _sb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_msgs_genlisp)
add_dependencies(sb_msgs_genlisp sb_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)
_generate_msg_py(sb_msgs
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(sb_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sb_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sb_msgs_generate_messages sb_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausout.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/gps.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/ServoCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Velocity.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/Pose.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/IMU.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/TurretCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/jausin.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/LidarNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/CarCommand.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/VisionNav.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(sb_msgs_generate_messages_py _sb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_msgs_genpy)
add_dependencies(sb_msgs_genpy sb_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sb_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sb_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sb_msgs_generate_messages_py std_msgs_generate_messages_py)
