cmake_minimum_required(VERSION 3.10.0)

# recursively traversal all protobuf source files
file(
  GLOB_RECURSE PROTO_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  *.proto)

# make proto file variable visiable in parent scope
set(PROTO_FILES
    ${PROTO_FILES}
    PARENT_SCOPE)

# include directoris for proto files
set(PROTO_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    PARENT_SCOPE)
