# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget qdldl::qdldlstatic qdldl::qdldl)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target qdldl::qdldlstatic
add_library(qdldl::qdldlstatic STATIC IMPORTED)

set_target_properties(qdldl::qdldlstatic PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/lin_sys/direct/qdldl/qdldl_sources/include"
)

# Create imported target qdldl::qdldl
add_library(qdldl::qdldl SHARED IMPORTED)

set_target_properties(qdldl::qdldl PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/lin_sys/direct/qdldl/qdldl_sources/include"
)

# Import target "qdldl::qdldlstatic" for configuration "Debug"
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
  IMPORTED_LOCATION_DEBUG "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a"
  )

# Import target "qdldl::qdldl" for configuration "Debug"
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_LOCATION_DEBUG "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so"
  IMPORTED_SONAME_DEBUG "libqdldl.so"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
