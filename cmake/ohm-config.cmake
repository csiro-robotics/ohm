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

# Include the generated configuration file.
include("${CMAKE_CURRENT_LIST_DIR}/aproject-config-targets.cmake")

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

register_target(aproject::alib APROJECT_INCLUDE_DIRS APROJECT_LIBRARIES)
register_target(aproject::blib APROJECT_INCLUDE_DIRS APROJECT_LIBRARIES)
