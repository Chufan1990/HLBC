# Install script for directory: /home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/out/libosqp.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osqp" TYPE FILE FILES
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/auxil.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/constants.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/error.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/glob_opts.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/lin_alg.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/osqp.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/osqp_configure.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/proj.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/scaling.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/types.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/util.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/cs.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/polish.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/lin_sys.h"
    "/home/chufankong/autoware.ai/src/autoware/common/hlbc/third_party/osqp/include/ctrlc.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/out/libosqp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake"
         "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/CMakeFiles/Export/lib/cmake/osqp/osqp-targets-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/osqp-config.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/src/cmake_install.cmake")
  include("/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/include/cmake_install.cmake")
  include("/home/chufankong/autoware.ai/src/autoware/common/hlbc/build/third_party/osqp/lin_sys/cmake_install.cmake")

endif()

