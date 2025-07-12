# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_radar_world_calib_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED radar_world_calib_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(radar_world_calib_FOUND FALSE)
  elseif(NOT radar_world_calib_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(radar_world_calib_FOUND FALSE)
  endif()
  return()
endif()
set(_radar_world_calib_CONFIG_INCLUDED TRUE)

# output package information
if(NOT radar_world_calib_FIND_QUIETLY)
  message(STATUS "Found radar_world_calib: 0.0.0 (${radar_world_calib_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'radar_world_calib' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${radar_world_calib_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(radar_world_calib_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${radar_world_calib_DIR}/${_extra}")
endforeach()
