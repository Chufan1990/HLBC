cmake_minimum_required(VERSION 3.10.0)
project(hlbc)

set(${PROJECT}}_VERSION 0.0.1)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall")
set(CMAKE_CXX_FLAGS_DEBUG
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall -O0 -g -ggdb"
)
set(CMAKE_CXX_FLAGS_RELEASE
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall -O5")

# set(ABSL_PROPAGATE_CXX_STD ON)

add_subdirectory(third_party)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             visualization_msgs
             roscpp
             std_msgs
             rospy
             message_generation
             autoware_msgs
             message_filters)

set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(Protobuf_DIR "${THIRDPARTY_DIR}/protobuf")
set(OSQP_DIR "${THIRDPARTY_DIR}/osqp")

find_package(osqp REQUIRED PATHS ${OSQP_DIR})

message(STATUS "osqp_VERSION : ${osqp_VERSION}")

# message(WARNING "CMAKE_SYSTEM_PREFIX_PATH: ${CMAKE_SYSTEM_PREFIX_PATH}")
# message(WARNING "CMAKE_SYSTEM_FRAMEWORK_PATH: ${CMAKE_SYSTEM_FRAMEWORK_PATH}")
# message(WARNING "CMAKE_SYSTEM_APPBUNDLE_PATH: ${CMAKE_SYSTEM_APPBUNDLE_PATH}")

# message(WARNING "Eigen3_DIR: ${Eigen3_DIR}")

# message(WARNING "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}") message(WARNING
# "CMAKE_FRAMEWORK_PATH: ${CMAKE_FRAMEWORK_PATH}") message(WARNING
# "CMAKE_APPBUNDLE_PATH: ${CMAKE_APPBUNDLE_PATH}")

# message(WARNING "PATH: ${PATH}")

# list(APPEND CMAKE_PREFIX_PATH "${Protobuf_DIR}")

# message(WARNING "Protobuf_DIR: ${Protobuf_DIR}")

# find_package(Protobuf 3.19.0 EXACT MODULE REQUIRED) message(WARNING
# "THIRDPARTY_DIR: ${THIRDPARTY_DIR}")

find_library(
  Protobuf_LIBRARIES protobuf
  PATHS ${Protobuf_DIR}
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH)

find_program(
  PROTOBUF_PROTOC_EXECUTABLE protoc
  PATHS ${Protobuf_DIR}
  PATH_SUFFIXES bin
  NO_DEFAULT_PATH)

find_path(
  Protobuf_INCLUDE_DIRS google
  PATHS ${Protobuf_DIR}
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)

if("${Protobuf_LIBRARIES}" STREQUAL "Protobuf_LIBRARIES-NOTFOUND")
  message(WARNING "Protobuf package not found")
else()
  message(STATUS "Protobuf include path : ${Protobuf_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries : ${Protobuf_LIBRARIES}")
endif()

find_package(gflags REQUIRED)

find_package(glog REQUIRED)

find_package(Eigen3 REQUIRED)

find_library(Ipopt_LIBRARIES ipopt PATHS /usr/local/lib /usr/lib)

if("${Ipopt_LIBRARIES}" STREQUAL "Ipopt_LIBRARIES-NOTFOUND")
  message(WARNING ${Ipopt_LIBRARIES})
else()
  message(STATUS "Ipopt libraries : ${Ipopt_LIBRARIES}")
endif()

# message(WARNING "Protobuf_LIBRARIES=${Protobuf_LIBRARIES}") message(WARNING

# "PROTOBUF_INCLUDE_DIRS=${PROTOBUF_INCLUDE_DIRS}") if(Eigen3_FOUND)
# message(WARNING "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}") message(WARNING
# "Eigen3_LIBRARIES=${Eigen3_LIBRARIES}") endif() message(WARNING
# "gflags_LIBRARIES=${gflags_LIBRARIES}") message(WARNING

# "gflags_INCLUDE_DIR=${gflags_INCLUDE_DIR}")

# message(WARNING "GFLAGS_LIBRARIES=${GFLAGS_LIBRARIES}")

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# To declare and build messages, services or actions from within this package,
# follow these steps: * Let MSG_DEP_SET be the set of packages whose message
# types you use in your messages/services/actions (e.g. std_msgs,
# actionlib_msgs, ...). * In the file package.xml: * add a build_depend tag for
# "message_generation" * add a build_depend and a exec_depend tag for each
# package in MSG_DEP_SET * If MSG_DEP_SET isn't empty the following dependency
# has been pulled in but can be declared for certainty nonetheless: * add a
# exec_depend tag for "message_runtime" * In this file (CMakeLists.txt): * add
# "message_generation" and every package in MSG_DEP_SET to find_package(catkin
# REQUIRED COMPONENTS ...) * add "message_runtime" and every package in
# MSG_DEP_SET to catkin_package(CATKIN_DEPENDS ...) * uncomment the add_*_files
# sections below as needed and list every .msg/.srv/.action file to be processed
# * uncomment the generate_messages entry below * add every package in
# MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder

file(
  GLOB_RECURSE ${PROJECT_NAME}_MESSAGE_SOURCE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg
  *.msg)

message(
  STATUS
    "${PROJECT_NAME}_MESSAGE_SOURCE_FILES = ${${PROJECT_NAME}_MESSAGE_SOURCE_FILES}"
)

add_message_files(DIRECTORY msg FILES ${${PROJECT_NAME}_MESSAGE_SOURCE_FILES})
# add_message_files(FILES PathPoint.msg) Generate services in the 'srv' folder
# add_service_files( FILES Service1.srv Service2.srv )

# Generate actions in the 'action' folder add_action_files( FILES Action1.action
# Action2.action )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

# Generate protobuf in the 'proto' folder file( GLOB_RECURSE PROTO_FILES
# RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/proto/" *.proto)

# get_filename_component(PARENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
# DIRECTORY)

add_subdirectory(proto)

get_filename_component(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR}
                       DIRECTORY)
list(APPEND PROTO_FLAGS -I${PROTO_INCLUDE_DIRS})

foreach(FIL ${PROTO_FILES})
  # get_filename_component(FILE_NAME ${FIL} NAME_WE)
  # get_filename_component(FILE_DIR ${FIL} DIRECTORY)
  # get_filename_component(FILE_PATH ${FIL} ABSOLUTE)

  # file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.h")

  # list(APPEND PROTO_SRCS
  # "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.cc") list(APPEND
  # PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.h")

  # message(WARNING "FILE_PATH=${FILE_PATH}")

  execute_process(
    COMMAND
      ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS}
      --cpp_out=${PROTO_META_BASE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/proto/${FIL})
endforeach()

# message(WARNING "GFLAGS_INCLUDE_DIR=${GFLAGS_INCLUDE_DIR}") message(WARNING
# "GFLAGS_LIBRARIES=${GFLAGS_LIBRARIES}") message(WARNING
# "GLOG_INCLUDE_DIR=${GLOG_INCLUDE_DIR}") message(WARNING
# "GLOG_LIBRARIES=${GLOG_LIBRARIES}")

# message(WARNING "PROTO_SRCS=${PROTO_SRCS}") message(WARNING
# "PROTO_HDRS=${PROTO_HDRS}")

# ##############################################################################
# Declare ROS dynamic reconfigure parameters ##
# ##############################################################################

# To declare and build dynamic reconfigure parameters within this package,
# follow these steps: * In the file package.xml: * add a build_depend and a
# exec_depend tag for "dynamic_reconfigure" * In this file (CMakeLists.txt): *
# add "dynamic_reconfigure" to find_package(catkin REQUIRED COMPONENTS ...) *
# uncomment the "generate_dynamic_reconfigure_options" section below and list
# every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options( cfg/DynReconf1.cfg cfg/DynReconf2.cfg )

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent

catkin_package(
  INCLUDE_DIRS
  include
  ${PROTO_META_BASE_DIR}
  LIBRARIES
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${Ipopt_LIBRARIES}
  CATKIN_DEPENDS
  geometry_msgs
  visualization_msgs
  roscpp
  rospy
  std_msgs
  message_runtime
  autoware_msgs
  DEPENDS
  GFLAGS
  EIGEN3)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROTO_META_BASE_DIR})

# Declare a C++ library add_library(${PROJECT_NAME} src/${PROJECT_NAME}/hlbc.cpp
# )

# target_link_libraries(Project PRIVATE library1 library2)
# get_target_property(TARGET_LIBRARIES Project LINK_LIBRARIES)
# message("Libraries at start") message(${TARGET_LIBRARIES})

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCE_FILES src/common/*.cpp
     src/control/*.cpp src/planning/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_PROTO_SOURCE_FILES
     ${PROTO_META_BASE_DIR}/autoagric/*.cc)

# use, i.e. don't skip the full RPATH for the build tree
# set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already (but later on when
# installing) set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) set(CMAKE_INSTALL_RPATH
# "${Protobuf_DIR}/lib")

# add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ##############################################################################
# PROTOBUF ##
# ##############################################################################
# add_library(protobuf::protobuf SHARED IMPORTED) set_property(TARGET
# protobuf::protobuf PROPERTY IMPORTED_LOCATION ${Protobuf_LIBRARIES})
# set_target_properties( protobuf::protobuf PROPERTIES
# INTERFACE_INCLUDE_DIRECTORIES ${Protobuf_INCLUDE_DIRS})

# ##############################################################################
# HLBC ##
# ##############################################################################

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCE_FILES}
                            ${${PROJECT_NAME}_PROTO_SOURCE_FILES})

# message(WARNING "LIBRARY_OUTPUT_DIRECTORY=${LIBRARY_OUTPUT_DIRECTORY}")

# set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 0.0.1 SOVERSION 1)

target_link_libraries(
  ${PROJECT_NAME}
  glog::glog
  absl::base
  absl::synchronization
  absl::strings
  absl::str_format
  osqp::osqp)

# ##############################################################################
# control_node ##
# ##############################################################################

add_executable(control_node src/nodes/control_node.cpp)

add_dependencies(control_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  control_node
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Ipopt_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  glog::glog
  osqp::osqp)

# ##############################################################################
# trajectory_visualizer_test ##
# ##############################################################################

add_executable(trajectory_visualizer_test
               src/nodes/trajectory_visualizer_test.cpp)

add_dependencies(trajectory_visualizer_test ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  trajectory_visualizer_test
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Ipopt_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  glog::glog)

# ##############################################################################
# discrete_points_trajectory_smoother_test ##
# ##############################################################################
# add_executable(discrete_points_trajectory_smoother_test
# src/nodes/discrete_points_trajectory_smoother_test.cpp)

# add_dependencies(discrete_points_trajectory_smoother_test
# ${catkin_EXPORTED_TARGETS})

# target_link_libraries( discrete_points_trajectory_smoother_test
# ${PROJECT_NAME} ${Protobuf_LIBRARIES} ${catkin_LIBRARIES} ${Ipopt_LIBRARIES}
# ${GFLAGS_LIBRARIES} glog::glog)

# ##############################################################################
# static_trajectory_loader_test ##
# ##############################################################################
add_executable(static_path_generator_test
               src/nodes/static_path_generator_test.cpp)

add_dependencies(static_path_generator_test ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(
  static_path_generator_test
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Ipopt_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  glog::glog)

# ##############################################################################
# piecewise_jerk_speed_problem_test ##
# ##############################################################################
add_executable(piecewise_jerk_speed_problem_test
               src/nodes/piecewise_jerk_speed_problem_test.cpp)

add_dependencies(piecewise_jerk_speed_problem_test ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(
  piecewise_jerk_speed_problem_test
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Ipopt_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  glog::glog)
# ##############################################################################
# Install ##
# ##############################################################################

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination catkin_install_python(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark libraries for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install( TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets ARCHIVE
# DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_GLOBAL_BIN_DESTINATION}) message(WARNING
# "CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}") message( WARNING
# "CATKIN_PACKAGE_SHARE_DESTINATION=${CATKIN_PACKAGE_SHARE_DESTINATION}" ) Mark
# cpp header files for installation install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE )

install(
  TARGETS ${PROJECT_NAME} control_node trajectory_visualizer_test
          static_path_generator_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Install project namespaced headers

install(
  DIRECTORY third_party/protobuf/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.cc" EXCLUDE
  PATTERN "*.cpp" EXCLUDE)

install(
  DIRECTORY third_party/protobuf/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  PATTERN ".svn" EXCLUDE)

# install( DIRECTORY ThirdParty/protobuf/bin/ DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} PATTERN ".svn" EXCLUDE)

install(
  DIRECTORY ${PROTO_META_BASE_DIR}/autoagric
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.cc" EXCLUDE
  PATTERN "*.cpp" EXCLUDE)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(
  DIRECTORY conf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf
  PATTERN ".svn" EXCLUDE)

install(
  DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  PATTERN ".svn" EXCLUDE)

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hlbc.cpp) if(TARGET
# ${PROJECT_NAME}-test) target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)
