# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_exemplo3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED exemplo3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(exemplo3_FOUND FALSE)
  elseif(NOT exemplo3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(exemplo3_FOUND FALSE)
  endif()
  return()
endif()
set(_exemplo3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT exemplo3_FIND_QUIETLY)
  message(STATUS "Found exemplo3: 0.0.0 (${exemplo3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'exemplo3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${exemplo3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(exemplo3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${exemplo3_DIR}/${_extra}")
endforeach()
