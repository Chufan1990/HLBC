#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qdldl::qdldlstatic" for configuration "Debug"
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libqdldl.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS qdldl::qdldlstatic )
list(APPEND _IMPORT_CHECK_FILES_FOR_qdldl::qdldlstatic "${_IMPORT_PREFIX}/lib/libqdldl.a" )

# Import target "qdldl::qdldl" for configuration "Debug"
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libqdldl.so"
  IMPORTED_SONAME_DEBUG "libqdldl.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS qdldl::qdldl )
list(APPEND _IMPORT_CHECK_FILES_FOR_qdldl::qdldl "${_IMPORT_PREFIX}/lib/libqdldl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
