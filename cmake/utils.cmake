# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# A collection of useful utility functions.

include(CMakeParseArguments)

#-------------------------------------------------------------------------------
# ohm_feature(<name> <help_string>
#   [FIND <package1> [<package2>...]]
#   [CONDITION <condition>]
# )
#
# Setup an OHM feature which can be enabled via either VCPKG or CMake with VCPKG_MANIFEST_FEATURES being authoritative.
#
# The feature NAME is mapped to a cmake option as 'OHM_FEATURE_$<NAME>' and to an equivalent VCPKG feature - an entry
# in the VCPKG_MANIFEST_FEATURES list - where the vcpkg feature name is the lower case NAME with underscores replaced by
# hyphens '-'. The CMake option defaults to on when the feature is present in VCPKG_MANIFEST_FEATURES.
#
# Additionally, the default state of the CMake option can be affected by the results of find_package() commands. When
# FIND is specified and all packages listed are located, then the CMake OHM_FEATURE_<NAME> option is enabled. Note that
# this does not guarantee that the library is located from the VCPKG packages, thus when using vcpkg, it's best to
# toggle features using VCPKG_MANIFEST_FEATURES exclusively.
#
# FIND - list of packages which, if found, will enable this feature implicitly.
#
# DEFAULT - a variable which determines the default state of the feature option if not explicitly enabled.
function(ohm_feature NAME HELP)
  cmake_parse_arguments(ARG "" "DEFAULT" "FIND" ${ARGN})
  set(_require_from_vcpkg No)   # Required from VCPKG feature set?
  set(_require_from_option No)  # Required from OHM_FEATURE_ option?
  set(_imply_from_package No)   # Package is available if needed, potentially implying the feature?

  string(TOLOWER "${NAME}" _vcpkg_name)
  # _ not allowed in vcpkg feature
  string(REPLACE "_" "-" _vcpkg_name "${_vcpkg_name}")

  # Check for VCPKG specified feature.
  list(FIND VCPKG_MANIFEST_FEATURES ${_vcpkg_name} _find_at)
  if(_find_at GREATER_EQUAL 0)
    # VCPKG feature explicitly enabled.
    set(_require_from_vcpkg Yes)
    set(_feature_source "vcpkg")
  endif(_find_at GREATER_EQUAL 0)

  # Check for build option enabled (from command line or cache).
  if(OHM_FEATURE_${NAME})
    set(_require_from_option Yes)
    set(_feature_source "option")
  endif(OHM_FEATURE_${NAME})

  # Determine default state.
  if(_require_from_vcpkg OR _require_from_option)

    # If explicitly required then enable by default.
    set(_default On)

  else(_require_from_vcpkg OR _require_from_option)

    # If not explicitly required then check the DEFAULT and FIND args.
    set(_default On)

    if(DEFINED ARG_DEFAULT)
      if(NOT ${ARG_DEFAULT})
        set(_default Off)
      endif(NOT ${ARG_DEFAULT})
      if(_default)
        set(_feature_source "default")
      endif(_default)
    endif(DEFINED ARG_DEFAULT)

    if(DEFINED ARG_FIND)
      # If enable by default, try to find packages.
      if(_default)
        foreach(pkg ${ARG_FIND}) # Check each package
          find_package(${pkg} QUIET)
          if(NOT ${pkg}_FOUND)
            # Missing package. Do not enable by default.
            set(_default Off)
            break()
          endif(NOT ${pkg}_FOUND)
        endforeach(pkg)
      endif(_default)
      if(_default)
        set(_feature_source "find_package")
      endif(_default)
    endif(DEFINED ARG_FIND)

  endif(_require_from_vcpkg OR _require_from_option)

  option(OHM_FEATURE_${NAME} "${HELP}" "${_default}")
  if(OHM_FEATURE_${NAME})
    message(STATUS "Enable feature ${NAME} from ${_feature_source}")
  endif(OHM_FEATURE_${NAME})
endfunction(ohm_feature)

#-------------------------------------------------------------------------------
# option_expr
# Add an option with the given expression determining the default option state.
function(option_cond VAR HELP EXPR)
  set(_DEFAULT OFF)
  if(${EXPR})
    set(_DEFAULT ON)
  endif(${EXPR})
  option(${VAR} "${HELP}" ${_DEFAULT})
endfunction(option_cond)

#-------------------------------------------------------------------------------
# show_properties(<GLOBAL           |
#                  DIRECTORY [dir]  |
#                  TARGET <target>  |
#                  SOURCE <source>  |
#                  INSTALL <file>   |
#                  TEST <test>      |
#                  CACHE <entry>    |
#                  VARIABLE>)
# Print all the properties and their values of the selected type. For example, target properties are listed by
# specifying show_properties(TARGET <target>), while global properties are listed using show_properties(GLOBAL)
function(show_properties TYPE)
  # Validate TYPE values requiring an argument.
  set(TARGETED_ITEMS CACHE INSTALL SOURCE TARGET TEST)
  list(FIND TARGETED_ITEMS ${TYPE} TYPE_INDEX)
  if(TYPE_INDEX GREATER -1)
    if("${ARGV1}" STREQUAL "")
      message(SEND_ERROR "show_properties(${TYPE} <item>) missing <item> argument.")
      return()
    endif("${ARGV1}" STREQUAL "")
  endif(TYPE_INDEX GREATER -1)

  execute_process(COMMAND ${CMAKE_COMMAND} --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
  # Convert command output into a CMake list
  string(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
  string(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")

  # Filter the property list.
  set(ITEM_PROPERTY_LIST)
  foreach(PROPERTY ${CMAKE_PROPERTY_LIST})
    # message("${PROPERTY}...")
    if(PROPERTY MATCHES ".*<CONFIG>.*")
      # Replace <CONFIG> with each of the build configs and add those to the list.
      foreach(CONFIG Debug;MinSizeRel;Release;RelWithDebInfo)
        string(REGEX REPLACE "(.*)<CONFIG>(.*)" "\\1${CONFIG}\\2" CONFIG_PROPERTY "${PROPERTY}")
        list(APPEND ITEM_PROPERTY_LIST "${CONFIG_PROPERTY}")
        # message("...${CONFIG_PROPERTY}")
      endforeach(CONFIG)
    else(PROPERTY MATCHES "$.*<CONFIG>^")
      list(APPEND ITEM_PROPERTY_LIST "${PROPERTY}")
      # message("...${PROPERTY}")
    endif(PROPERTY MATCHES ".*<CONFIG>.*")
  endforeach(PROPERTY)

  if("${ARGV1}" STREQUAL "")
    message("${TYPE} properties:")
  else("${ARGV1}" STREQUAL "")
    set(ITEM "${ARGV1}")
    message("${ITEM} properties:")
  endif("${ARGV1}" STREQUAL "")
  foreach(PROPERTY ${ITEM_PROPERTY_LIST})
    # message ("Checking ${prop}")
    get_property(PROPERTY_VALUE ${TYPE} ${ITEM} PROPERTY ${PROPERTY} SET)
    if(PROPERTY_VALUE)
      get_property(PROPERTY_VALUE ${TYPE} ${ITEM} PROPERTY ${PROPERTY})
      message ("  ${PROPERTY}: ${PROPERTY_VALUE}")
    endif(PROPERTY_VALUE)
  endforeach(PROPERTY)
endfunction(show_properties)

#-------------------------------------------------------------------------------
# show_target_properties(<target>)
# Print all the properties and their values for <target>. Only lists the properties which have been set for <target>.
function(show_target_properties TARGET)
  # https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake
  if(NOT TARGET ${TARGET})
    message("There is no target named '${TARGET}'")
    return()
  endif(NOT TARGET ${TARGET})

  show_properties(TARGET ${TARGET})
endfunction(show_target_properties)

#-------------------------------------------------------------------------------
# show_variables()
# List all variables and their values.
function(show_variables)
  get_cmake_property(VAR_NAMES VARIABLES)
  foreach (VAR ${VAR_NAMES})
      message("${VAR}: ${${VAR}}")
  endforeach(VAR)
endfunction(show_variables)
