# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hlbc: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ihlbc:/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hlbc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_custom_target(_hlbc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hlbc" "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" ""
)

get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_custom_target(_hlbc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hlbc" "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" "hlbc/TrajectoryPoint:hlbc/PathPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_custom_target(_hlbc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hlbc" "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" "hlbc/PathPoint"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc
)
_generate_msg_cpp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg;/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc
)
_generate_msg_cpp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc
)

### Generating Services

### Generating Module File
_generate_module_cpp(hlbc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hlbc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hlbc_generate_messages hlbc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_cpp _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_cpp _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_cpp _hlbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hlbc_gencpp)
add_dependencies(hlbc_gencpp hlbc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hlbc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc
)
_generate_msg_eus(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg;/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc
)
_generate_msg_eus(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc
)

### Generating Services

### Generating Module File
_generate_module_eus(hlbc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hlbc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hlbc_generate_messages hlbc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_eus _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_eus _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_eus _hlbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hlbc_geneus)
add_dependencies(hlbc_geneus hlbc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hlbc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc
)
_generate_msg_lisp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg;/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc
)
_generate_msg_lisp(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc
)

### Generating Services

### Generating Module File
_generate_module_lisp(hlbc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hlbc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hlbc_generate_messages hlbc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_lisp _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_lisp _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_lisp _hlbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hlbc_genlisp)
add_dependencies(hlbc_genlisp hlbc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hlbc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc
)
_generate_msg_nodejs(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg;/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc
)
_generate_msg_nodejs(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hlbc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hlbc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hlbc_generate_messages hlbc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_nodejs _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_nodejs _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_nodejs _hlbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hlbc_gennodejs)
add_dependencies(hlbc_gennodejs hlbc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hlbc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc
)
_generate_msg_py(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg;/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc
)
_generate_msg_py(hlbc
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc
)

### Generating Services

### Generating Module File
_generate_module_py(hlbc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hlbc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hlbc_generate_messages hlbc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/PathPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_py _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/Trajectory.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_py _hlbc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/chufankong/autoware.ai/src/autoware/common/hlbc/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(hlbc_generate_messages_py _hlbc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hlbc_genpy)
add_dependencies(hlbc_genpy hlbc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hlbc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hlbc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hlbc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hlbc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hlbc_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hlbc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hlbc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hlbc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hlbc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hlbc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hlbc_generate_messages_py std_msgs_generate_messages_py)
endif()
