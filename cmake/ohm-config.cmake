# Copyright (c) 2017
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# CMake package documentation (https://cmake.org/cmake/help/v3.5/manual/cmake-packages.7.html) shows how to generate
# cmake package config files. While a <pacakge>-config.cmake file can be generated and used directly, the documentation
# recommends providing a wrapper such as this file. The purpose is to ensure other depdencies are found first and bound
# as required. That is, this is a good place to find external dependencies for the project libraries.
# https://cmake.org/cmake/help/v3.5/manual/cmake-packages.7.html#creating-a-package-configuration-file

# Supports selection between OpenCL and CUDA implementations via find_package(ohm COMPONENTS [OpenCL,CUDA]). The
# selected library is defined in the variable OHM_GPU_LIBRARY with the supporting gputil library defined in
# OHM_GPUTIL_LIBRARY

# Configuration
include("${CMAKE_CURRENT_LIST_DIR}/ohm-packages.cmake")

# Include the generated configuration file.
include("${CMAKE_CURRENT_LIST_DIR}/ohm-config-targets.cmake")

set(OHM_BUILD_OPENCL @OHM_BUILD_OPENCL@)
set(OHM_BUILD_CUDA @OHM_BUILD_CUDA@)
set(OHM_BUILD_HEIGHTMAP_IMAGE @OHM_BUILD_HEIGHTMAP_IMAGE@)

function(register_target TARGET INCLUDES_VAR LIBRARIES_VAR)
  if(TARGET ${TARGET})
    # Resolve include directories
    get_target_property(TARGET_INCLUDE_DIRS ${TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    set(TARGET_INCLUDE_DIRS "${TARGET_INCLUDE_DIRS}" CACHE PATH "${TARGET} include directories")

    # Append to existing include directories
    list(APPEND ${INCLUDES_VAR} ${TARGET_INCLUDE_DIRS})
    list(REMOVE_DUPLICATES ${INCLUDES_VAR})
    # Propagate to caller
    set(${INCLUDES_VAR} ${${INCLUDES_VAR}} PARENT_SCOPE)

    # Resolve release and debug libraries.
    get_target_property(TARGET_LIBRARY_DEBUG ${TARGET} IMPORTED_LOCATION_DEBUG)
    get_target_property(TARGET_LIBRARY_RELEASE ${TARGET} IMPORTED_LOCATION_RELEASE)

    # For non-Windows, substitude RelWithDebInfo for release library. Can't for Windows as it
    # links the debug runtime so is more optimised debug than release with debug info.
    if(NOT WIN32 AND NOT TARGET_LIBRARY_RELEASE)
      get_target_property(TARGET_LIBRARY_RELEASE ${TARGET} IMPORTED_LOCATION_REL_WITH_DEB_INFO)
    endif(NOT WIN32 AND NOT TARGET_LIBRARY_RELEASE)

    # Try with no config specified.
    if(NOT TARGET_LIBRARY_RELEASE)
      get_target_property(TARGET_LIBRARY_RELEASE ${TARGET} IMPORTED_LOCATION)

      if(NOT TARGET_LIBRARY_RELEASE)
        get_target_property(TARGET_LIBRARY_RELEASE ${TARGET} IMPORTED_LOCATION_NOCONFIG)
      endif(NOT TARGET_LIBRARY_RELEASE)
    endif(NOT TARGET_LIBRARY_RELEASE)

    # Get any supporting libraries for the target.
    get_target_property(TARGET_SUPPORT_LIBRARIES ${TARGET} INTERFACE_LINK_LIBRARIES)

    # Extent the library list.
    set(TARGET_LIBRARIES)
    if(TARGET_LIBRARY_DEBUG)
      list(APPEND TARGET_LIBRARIES debug "${TARGET_LIBRARY_DEBUG}" optimized "${TARGET_LIBRARY_RELEASE}")
    else(TARGET_LIBRARY_DEBUG)

      set(TARGET_LIBRARIES "${TARGET_LIBRARY_RELEASE}")
    endif(TARGET_LIBRARY_DEBUG)

    list(APPEND ${LIBRARIES_VAR} ${TARGET_LIBRARIES})
    if(TARGET_SUPPORT_LIBRARIES)
      list(APPEND ${LIBRARIES_VAR} ${TARGET_SUPPORT_LIBRARIES})
    endif(TARGET_SUPPORT_LIBRARIES)

    # Export to caller.
    set(${LIBRARIES_VAR} ${${LIBRARIES_VAR}} PARENT_SCOPE)

  else(TARGET ${TARGET})
    message(SEND_ERROR "${TARGET} not found")
  endif(TARGET ${TARGET})
endfunction(register_target)

# Define the OHM library according to the selected component
if(ohm_FIND_COMPONENTS)
  foreach(_comp ${ohm_FIND_COMPONENTS})
    if(_comp EQUAL "OpenCL")
      set(OHM_GPUTIL_LIBRARY ohm::gputilocl)
      set(OHM_GPU_LIBRARY ohm::ohmocl)
    elseif(_comp EQUAL "CUDA")
      set(OHM_GPUTIL_LIBRARY ohm::gputilcuda)
      set(OHM_GPU_LIBRARY ohm::ohmcuda)
    endif()
  endforeach(_comp)
endif(ohm_FIND_COMPONENTS)

register_target(ohm::ohmtools OHM_INCLUDE_DIRS OHM_LIBRARIES)
register_target(ohm::ohmutil OHM_INCLUDE_DIRS OHM_LIBRARIES)
register_target(ohm::ohm OHM_INCLUDE_DIRS OHM_LIBRARIES)
register_target(ohm::slamio OHM_INCLUDE_DIRS OHM_LIBRARIES)

if(OHM_BUILD_CUDA)
  register_target(ohm::gputilcuda OHM_INCLUDE_DIRS OHM_LIBRARIES)
  register_target(ohm::ohmcuda OHM_INCLUDE_DIRS OHM_LIBRARIES)
  if(NOT DEFINED OHM_GPU_LIBRARY)
      set(OHM_GPUTIL_LIBRARY ohm::gputilcuda)
    set(OHM_GPU_LIBRARY ohm::ohmcuda)
  endif(NOT DEFINED OHM_GPU_LIBRARY)
endif(OHM_BUILD_CUDA)

if(OHM_BUILD_OPENCL)
  register_target(ohm::clu OHM_INCLUDE_DIRS OHM_LIBRARIES)
  register_target(ohm::gputilocl OHM_INCLUDE_DIRS OHM_LIBRARIES)
  register_target(ohm::ohmocl OHM_INCLUDE_DIRS OHM_LIBRARIES)
  if(NOT DEFINED OHM_GPU_LIBRARY)
   set(OHM_GPUTIL_LIBRARY ohm::gputilocl)
    set(OHM_GPU_LIBRARY ohm::ohmocl)
  endif(NOT DEFINED OHM_GPU_LIBRARY)
endif(OHM_BUILD_OPENCL)

# Packages required for ohmheightmapimage
if(OHM_BUILD_HEIGHTMAP_IMAGE)
  find_package(OpenGL REQUIRED)
  register_target(ohm::ohmheightmapimage OHM_INCLUDE_DIRS OHM_LIBRARIES)
endif(OHM_BUILD_HEIGHTMAP_IMAGE)
