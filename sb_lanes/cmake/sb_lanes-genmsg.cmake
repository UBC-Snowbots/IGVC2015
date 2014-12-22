# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sb_lanes: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isb_lanes:/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isb_msgs:/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sb_lanes_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg" NAME_WE)
add_custom_target(_sb_lanes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sb_lanes" "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sb_lanes
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_lanes
)

### Generating Services

### Generating Module File
_generate_module_cpp(sb_lanes
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_lanes
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sb_lanes_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sb_lanes_generate_messages sb_lanes_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg" NAME_WE)
add_dependencies(sb_lanes_generate_messages_cpp _sb_lanes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_lanes_gencpp)
add_dependencies(sb_lanes_gencpp sb_lanes_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_lanes_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sb_lanes
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_lanes
)

### Generating Services

### Generating Module File
_generate_module_lisp(sb_lanes
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_lanes
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sb_lanes_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sb_lanes_generate_messages sb_lanes_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg" NAME_WE)
add_dependencies(sb_lanes_generate_messages_lisp _sb_lanes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_lanes_genlisp)
add_dependencies(sb_lanes_genlisp sb_lanes_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_lanes_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sb_lanes
  "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_lanes
)

### Generating Services

### Generating Module File
_generate_module_py(sb_lanes
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_lanes
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sb_lanes_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sb_lanes_generate_messages sb_lanes_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dulluhan/Desktop/Snowbots/src/IGVC2015/sb_lanes/msg/Num.msg" NAME_WE)
add_dependencies(sb_lanes_generate_messages_py _sb_lanes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sb_lanes_genpy)
add_dependencies(sb_lanes_genpy sb_lanes_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sb_lanes_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_lanes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sb_lanes
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sb_lanes_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(sb_lanes_generate_messages_cpp sb_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_lanes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sb_lanes
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sb_lanes_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(sb_lanes_generate_messages_lisp sb_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_lanes)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_lanes\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sb_lanes
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sb_lanes_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(sb_lanes_generate_messages_py sb_msgs_generate_messages_py)
