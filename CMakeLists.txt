cmake_minimum_required(VERSION 3.0.2)
project(hlbc)

set(${PROJECT}}_VERSION 1.0.0)
# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall")
set(CMAKE_CXX_FLAGS_DEBUG
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall -O3 -g -ggdb"
)
set(CMAKE_CXX_FLAGS_RELEASE
    "-std=c++14 -march=native -pipe -Wextra -fopenmp -fPIC -pthread -Wall -O0")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             visualization_msgs
             roscpp
             rospy
             std_msgs
             message_generation
             autoware_msgs)

find_package(Protobuf 3.19 REQUIRED)

find_package(gflags REQUIRED)

find_package(glog REQUIRED)

find_package(Eigen3 REQUIRED)
# message(WARNING "Protobuf_LIBRARIES=${Protobuf_LIBRARIES}") message(WARNING
# "PROTOBUF_INCLUDE_DIRS=${PROTOBUF_INCLUDE_DIRS}")

# message(WARNING "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}") message(WARNING
# "EIGEN3_LIBRARIES=${EIGEN3_LIBRARIES}")

# message(WARNING "gflags_LIBRARIES=${gflags_LIBRARIES}") message(WARNING
# "gflags_INCLUDE_DIR=${gflags_INCLUDE_DIR}")

# message(WARNING "glog_LIBRARIES=${glog_LIBRARIES}") message(WARNING
# "glog_INCLUDE_DIR=${glog_INCLUDE_DIR}")

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

# Generate messages in the 'msg' folder add_message_files( FILES Message1.msg
# Message2.msg )

# Generate services in the 'srv' folder add_service_files( FILES Service1.srv
# Service2.srv )

# Generate actions in the 'action' folder add_action_files( FILES Action1.action
# Action2.action )

# Generate added messages and services with any dependencies listed here
# generate_messages( DEPENDENCIES std_msgs  # Or other packages containing msgs
# )

# Generate protobuf in the 'proto' folder
file(
  GLOB_RECURSE PROTO_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  *.proto)

get_filename_component(PARENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(PARENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)

# message(WARNING "PARENT_BINARY_DIR=${PARENT_BINARY_DIR}")

set(PROTO_META_BASE_DIR ${PARENT_BINARY_DIR})
list(APPEND PROTO_FLAGS -I${PARENT_SOURCE_DIR})

# message(WARNING "PROTO_FLAGS=${PROTO_FLAGS}")

foreach(FIL ${PROTO_FILES})
  get_filename_component(FILE_NAME ${FIL} NAME_WE)
  get_filename_component(FILE_DIR ${FIL} DIRECTORY)
  get_filename_component(FILE_PATH ${FIL} ABSOLUTE)

  # file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.cc"
  # "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.h")

  # list(APPEND PROTO_SRCS
  # "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.cc") list(APPEND
  # PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR}/${FILE_NAME}.pb.h")

  # message(WARNING "FILE_PATH=${FILE_PATH}")

  execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS}
                          --cpp_out=${PROTO_META_BASE_DIR} ${FILE_PATH})
endforeach()

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
# projects also need
catkin_package(
  INCLUDE_DIRS
  include
  ${PROTO_META_BASE_DIR}
  LIBRARIES
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  gflags
  glog
  CATKIN_DEPENDS
  geometry_msgs
  visualization_msgs
  roscpp
  rospy
  std_msgs
  message_runtime
  autoware_msgs
  # DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(
  include ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PROTO_META_BASE_DIR})

# Declare a C++ library add_library(${PROJECT_NAME} src/${PROJECT_NAME}/hlbc.cpp
# )

# target_link_libraries(Project PRIVATE library1 library2)
# get_target_property(TARGET_LIBRARIES Project LINK_LIBRARIES)
# message("Libraries at start") message(${TARGET_LIBRARIES})

# find_library( ${PROJECT_NAME}_PROTO_LIBRARIES_FOUND NAMES
# ${PROJECT_NAME}_PROTO_LIBRARIES PATHS ${CMAKE_PREFIX_PATH})

# message(WARNING "${${PROJECT_NAME}_PROTO_LIBRARIES_FOUND}")

# if(NOT "${${PROJECT_NAME}_PROTO_LIBRARIES_FOUND}" STREQUAL
# "${PROJECT_NAME}_PROTO_LIBRARIES_FOUND-NOTFOUND")

# message(WARNING "Removing ${${PROJECT_NAME}_PROTO_LIBRARIES_FOUND}")

# file(REMOVE ${PROJECT_NAME}_PROTO_LIBRARIES_FOUND)

# endif()

# add_library(${PROJECT_NAME}_PROTO_LIBRARIES ${PROTO_SRCS} ${PROTO_HDRS})

# target_link_libraries(${PROJECT_NAME}_PROTO_LIBRARIES ${PROTOBUF_LIBRARIES})

file(GLOB_RECURSE ${PROJECT_NAME}_SOURCE_FILES src/common/*.cpp
     src/control/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_PROTO_SOURCE_FILES
     ${CMAKE_CURRENT_BINARY_DIR}/*.cc)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCE_FILES}
                            ${${PROJECT_NAME}_PROTO_SOURCE_FILES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/nodes/control_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  gflags
  glog)
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
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install project namespaced headers
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
# install( EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake
# NAMESPACE ${PROJECT_NAME}:: DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION}/cmake/${PROJECT_NAME})

# include(CMakePackageConfigHelpers) write_basic_package_version_file(
# "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
# VERSION ${PROJECT}_VERSION COMPATIBILITY AnyNewerVersion)

# export( EXPORT ${PROJECT_NAME}Targets FILE
# "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}Targets.cmake"
# NAMESPACE ${PROJECT_NAME}::)

# configure_file( cmake/${PROJECT_NAME}Config.cmake
# "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake"
# COPYONLY)

# set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})

# install( EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake
# NAMESPACE ${PROJECT_NAME}:: DESTINATION ${ConfigPackageLocation})

# install( FILES cmake/${PROJECT_NAME}Config.cmake
# "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake"
# DESTINATION ${ConfigPackageLocation} COMPONENT Devel)

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
