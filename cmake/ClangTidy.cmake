# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Add clang-tidy support at a per target level.
#
# This script allows clang-tidy support to be added to individual targets which can then be build collectively.
# Additionally, this supports having multiple clang-tidy configurations to be selected and used dynamically.
#
# Available configurations are in clang-tidy yaml format (see 'clang-tidy -dump-config') using the file naming
# 'clang-tidy-<level>.yaml' where <level> is a meaningful work idnetifying the level of checing. These files are searched
# for in the 'tidy/' subdirectory directory alongside this script; specifically '${CMAKE_CURRENT_LIST_DIR}/tidy'.
#
# The configuration to actually use is selected by the CMake cache variable: CLANG_TIDY_LEVEL. This appears as a drop
# down list in visual CMake configuration tools such as ccmake and cmake-gui.
#
# Targets must be explicitly added for clang-tidy linting using the CMake function 'clang_tidy_target(<target>)'. This
# creates a target name '<target>-clang-tidy-<level>' where <level> matches the CLANG_TIDY_LEVEL. A target may also
# be create which build all registered clang-tidy targets by invoking 'clang_tidy_global()'. This creates a target
# named 'clang-tidy-<level>' which processes all the registered targets.
#
# As a special case the CLANG_TIDY_LEVEL may be set to 'all'. This causes clang-tidy targets to be created for all
# available 'clang-tidy-<level>.yaml files.

if(CLANG_TIDY_CMAKE_INCLUDED)
  return()
endif(CLANG_TIDY_CMAKE_INCLUDED)

set(CLANG_TIDY_CMAKE_INCLUDED YES)

# Look for clang-tidy
set(CLANG_TIDY_NAMES "clang-tidy")
set(CLANG_TIDY_START_VERSION 10)
set(CLANG_TIDY_END_VERSION 5)
set(CLANG_TIDY_SEARCH_VERSION ${CLANG_TIDY_START_VERSION})
while(CLANG_TIDY_SEARCH_VERSION GREATER_EQUAL ${CLANG_TIDY_END_VERSION})
  list(APPEND CLANG_TIDY_NAMES "clang-tidy-${CLANG_TIDY_SEARCH_VERSION}")
  list(APPEND CLANG_TIDY_NAMES "clang-tidy-${CLANG_TIDY_SEARCH_VERSION}.0")
  math(EXPR CLANG_TIDY_SEARCH_VERSION "${CLANG_TIDY_SEARCH_VERSION} - 1")
endwhile(CLANG_TIDY_SEARCH_VERSION GREATER_EQUAL ${CLANG_TIDY_END_VERSION})

find_program(CLANG_TIDY_EXE NAMES ${CLANG_TIDY_NAMES}
            DOC "Path to clang-tidy executable")

if(NOT CLANG_TIDY_EXE)
  message(STATUS "clang-tidy not found: clang-tidy directives will be ignored")
  function(clang_tidy_target)
  endfunction(clang_tidy_target)
  function(clang_tidy_global)
  endfunction(clang_tidy_global)
  return()
endif(NOT CLANG_TIDY_EXE)

#-------------------------------------------------------------------------------
# Enable generation of compile_commands.json for Makefiles and Ninja.
#-------------------------------------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakeParseArguments)

# find_package(Python3 COMPONENTS Interpreter)
find_package(PythonInterp REQUIRED)

# Set the path where clang tidy config files are found
set(CLANG_TIDY_CONFIG_PATH "${CMAKE_CURRENT_LIST_DIR}/tidy")

# Resolve the names of the available configurations.
file(GLOB CLANG_TIDY_CONFIG_FILES RELATIVE "${CLANG_TIDY_CONFIG_PATH}" "${CLANG_TIDY_CONFIG_PATH}/*clang-tidy-*")

set(CLANG_TIDY_CONFIGS "")
foreach(CLANG_TIDY_CONFIG_FILE ${CLANG_TIDY_CONFIG_FILES})
  string(REGEX REPLACE ".*clang-tidy-([^\.]*)(\.yaml)?" "\\1" CLANG_TIDY_CONFIG "${CLANG_TIDY_CONFIG_FILE}")
  if(CLANG_TIDY_CONFIG)
    list(APPEND CLANG_TIDY_CONFIGS ${CLANG_TIDY_CONFIG})
  endif(CLANG_TIDY_CONFIG)
endforeach(CLANG_TIDY_CONFIG_FILE)

if(NOT CLANG_TIDY_CONFIGS)
  message(FATAL_ERROR "No clang-tidy-<name>.yaml files found in ${CLANG_TIDY_CONFIG_PATH}")
endif(NOT CLANG_TIDY_CONFIGS)
set(CLANG_TIDY_CONFIGS ${CLANG_TIDY_CONFIGS} CACHE INTERNAL "Available clang-tidy configurations")

# Add 'file' config before we clear the cached target list.
list(APPEND CLANG_TIDY_CONFIGS file)

foreach(CONFIG ${CLANG_TIDY_CONFIGS})
  # Clear cache variable listing all -clang-tidy targets
  set(CLANG_TIDY_TARGETS_${CONFIG} "" CACHE INTERNAL "" FORCE)
  set(CLANG_TIDY_TARGETS_${CONFIG}-fix "" CACHE INTERNAL "" FORCE)
endforeach(CONFIG)

# Add the 'all' config after everything else.
list(APPEND CLANG_TIDY_CONFIGS all)

# Add 'file' as a config implying using the standard .clang-tidy file finding approach.
# set(CTT_DEBUG ON)

# Enable clang-tidy if it was found.
set(CLANG_TIDY_LEVEL "all" CACHE STRING "Defines the level of clang tidy checks to invoke.")
set_property(CACHE CLANG_TIDY_LEVEL PROPERTY STRINGS ${CLANG_TIDY_CONFIGS})

#-------------------------------------------------------------------------------
# Utility functions.
#-------------------------------------------------------------------------------

# _ctt_debug(message)
# Print a message if CTT_DEBUG is true.
function(_ctt_debug)
  if(CTT_DEBUG)
    message(${ARGN})
  endif(CTT_DEBUG)
endfunction(_ctt_debug)

function(__ctt_setup_target TARGET WORKING_DIRECTORY)
  cmake_parse_arguments(CTT "FIX" "CONFIG_LEVEL;BUILD_PATH" "" ${ARGN})
  if(CTT_CONFIG_LEVEL)
    if(NOT CTT_CONFIG_LEVEL STREQUAL "file")
      set(CONFIG_ARG "-config-file=${CLANG_TIDY_CONFIG_PATH}/clang-tidy-${CTT_CONFIG_LEVEL}.yaml")
    endif(NOT CTT_CONFIG_LEVEL STREQUAL "file")
  endif(CTT_CONFIG_LEVEL)

  if(CTT_BUILD_PATH)
    set(CTT_BUILD_PATH "-p=${CTT_BUILD_PATH}")
  endif(CTT_BUILD_PATH)

  # Convert CTT_FIX to either be empty or '-fix' when the argument is present. Also set CTT_SUFFIX to '-fix' to the in
  # the latter case.
  if(CTT_FIX)
    set(CTT_FIX "-fix")
    set(CTT_SUFFIX "-fix")
    set(CTT_)
  else(CTT_FIX)
    set(CTT_FIX)
    set(CTT_SUFFIX)
  endif(CTT_FIX)

  set(CTT_TARGET_NAME ${TARGET}-clang-tidy-${CTT_CONFIG_LEVEL}${CTT_SUFFIX})
  add_custom_target(${CTT_TARGET_NAME}
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    COMMAND
      "${PYTHON_EXECUTABLE}" "${CLANG_TIDY_CONFIG_PATH}/wrap-clang-tidy.py"
      "-clang-tidy-binary=${CLANG_TIDY_EXE}"
      ${CONFIG_ARG} ${CTT_BUILD_PATH} ${CTT_UNPARSED_ARGUMENTS} ${CTT_FIX}
  )
  set_target_properties(${CTT_TARGET_NAME} PROPERTIES FOLDER clang-tidy)

  # Experiment: add a target per source file in order to allow multi-thread compilation across the board.
  # Requires the CTT_TARGET_NAME target have no command.
  # Using run-clang-tidy.py allows a single project to use multiple threads, but this has two problems:
  # 1. When the build system (ninja or make) also runs multiple threads per target, then we have multiple threads
  #   spawning multiple threads which takes longer over all.
  # 2. Output is some times missed (bug)
  #
  # Early experiments shows the build system threading is faster than run-clang-tidy: using ohm analysis as an example
  # run-clang-tidy is ~34s, using the approach below is ~18s.
  # foreach(SOURCE_FILE ${CTT_UNPARSED_ARGUMENTS})
  #   get_filename_component(SOURCE_FILE_RELATIVE "${SOURCE_FILE}" REALPATH BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
  #   string(REGEX REPLACE "[/ \t]" "_" SOURCE_FILE_RELATIVE "${SOURCE_FILE_RELATIVE}")
  #   # set(SOURCE_TARGET_NAME ${SOURCE_FILE_RELATIVE}-${CTT_CONFIG_LEVEL}${CTT_SUFFIX})
  #   set(SOURCE_TARGET_NAME ${CTT_TARGET_NAME}-${SOURCE_FILE_RELATIVE})
  #   # message("SOURCE_TARGET_NAME: ${SOURCE_TARGET_NAME}")
  #   add_custom_target(${SOURCE_TARGET_NAME}
  #     WORKING_DIRECTORY "${WORKING_DIRECTORY}"
  #     COMMAND
  #       "${PYTHON_EXECUTABLE}" "${CLANG_TIDY_CONFIG_PATH}/wrap-clang-tidy.py"
  #       "-clang-tidy-binary=${CLANG_TIDY_EXE}"
  #       ${CONFIG_ARG} ${CTT_BUILD_PATH} "${SOURCE_FILE}" ${CTT_FIX}
  #   )
  #   add_dependencies(${CTT_TARGET_NAME} ${SOURCE_TARGET_NAME})
  # endforeach(SOURCE_FILE)

  # Track the added clang-tidy target in the cache.
  set(CTT_TARGETS ${CLANG_TIDY_TARGETS_${CTT_CONFIG_LEVEL}${CTT_SUFFIX}})
  list(APPEND CTT_TARGETS)
  list(APPEND CTT_TARGETS ${CTT_TARGET_NAME})
  set(CLANG_TIDY_TARGETS_${CTT_CONFIG_LEVEL}${CTT_SUFFIX} "${CTT_TARGETS}" CACHE INTERNAL "" FORCE)
endfunction(__ctt_setup_target TARGET WORKING_DIRECTORY)

# clang_tidy_target(
#   target
#   [EXCLUDE file1 file2 ...]
#   [EXCLUDE_MATCHES regex1 regex2 ...]
#   [INCLUDE_HEADERS]
# )
#
# Add clang tidy support for a target.
#
# This function has two ways to apply clang-tidy support:
# - Using CXX_CLANG_TIDY property on the target
# - Using a custom, post build step.
#
# CXX_CLANG_TIDY provides better integration, but is only supported for Unix Makefiles and Ninja based CMake generators.
# For these generators, a compile_commands.json file is generated and provided to clang-tidy, ensuring clang-tidy runs
# with the same build flags. In this mode the optional EXCLUDE, EXCLUDE_MATCHES and INCLUDE_HEADERS arguments are
# ignored.
#
# Other generators use a custom build step and the build flags may differ. For increased compatibility, the build step
# implementation extracts the SOURCES and INCLUDE_DIRECTORIES properties to derive the target files and include paths.
#
# The SOURCES are filtered first filtered to remove all header files, unless INCLUDE_HEADERS was passed, then to remove
# any file specified after EXCLUDES (exact match), while EXCLUDE_MATCHES removes items using regular expressions.
# Header files are matched using the regular expression ".*\.(h|hpp|hxx|inl)".
#
# Note by default EXCLUDE_MATCHES is setup to exclude header files and files of the forms "file.in.extention" and
# "file.extention.in". The '.in' files are assumed to be used with configure_file().
#
# The include directories are filtered for explict directories, $<BUILD_INTERFACE:xxx> and
# $<TARGET_PROPERTY:target,INCLUDE_DIRECTORIES> generator expressions, adding both to the resulting include directory
# list.
#
# Finally, this mode of operation also considered the C++ standard using the CXX_STANDARD target property.
function(clang_tidy_target TARGET)
  if(CMAKE_GENERATOR MATCHES "Visual Studio")
    return()
  endif(CMAKE_GENERATOR MATCHES "Visual Studio")
# CXX_CLANG_TIDY doesn't work with -config="multi line string"
  # if(NOT CMAKE_VERSION VERSION_LESS 3.6)
  #   # From CMake 3.6+, Ninja and Makefiles have in-built clang-tidy support.
  #   # We assume support if compile_commands.json has been generated.
  #   if(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  #     set_target_properties(${TARGET} PROPERTIES
  #       CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-p;${CMAKE_BINARY_DIR};-config=${CLANG_CONFIG}"
  #     )
  #     return()
  #   endif(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  # endif(NOT CMAKE_VERSION VERSION_LESS 3.6)

  # if(NOT EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  #   message(SEND_ERROR "No compile_commands.json in ${CMAKE_BINARY_DIR}")
  # endif(NOT EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")

  # Configure as a post build step
  if(NOT TARGET ${TARGET})
    message(error "${TARGET} is not defined as a target. Cannot configure clang-tidy for ${TARGET}")
    return()
  endif(NOT TARGET ${TARGET})

  cmake_parse_arguments(CTT
    "INCLUDE_HEADERS;DEBUG"
    ""
    "EXCLUDE;EXCLUDE_MATCHES"
    ${ARGN})

  # Get target sources and include directories.
  get_target_property(TARGET_SOURCES ${TARGET} SOURCES)

  if(NOT CTT_INCLUDE_HEADERS)
    set(HEADER_REX ".*\\.(h|hpp|hxx)")
    if(CTT_EXCLUDE_MATCHES)
      list(APPEND CTT_EXCLUDE_MATCHES "${HEADER_REX}")
    else(CTT_EXCLUDE_MATCHES)
      set(CTT_EXCLUDE_MATCHES ".*\\.in($|\\..*)" "${HEADER_REX}")
    endif(CTT_EXCLUDE_MATCHES)
  endif(NOT CTT_INCLUDE_HEADERS)

  # Filter target sources by EXCLUDE and EXCLUDE_MATCHES arguments
  _ctt_debug("TARGET_SOURCES: ${TARGET_SOURCES}")
  set(SOURCES)
  foreach(SRC ${TARGET_SOURCES})
    set(ADD_SRC TRUE)

    # Check exclude patterns.
    if(CTT_EXCLUDE)
      foreach(EXCLUDE ${CTT_EXCLUDE})
        if(SRC STREQUAL EXCLUDE)
          set(ADD_SRC FALSE)
          _ctt_debug("EXCLUDE ${SRC}")
          continue()
        endif(SRC STREQUAL EXCLUDE)
      endforeach(EXCLUDE)
    endif(CTT_EXCLUDE)

    # Check exclude expressions.
    if(CTT_EXCLUDE_MATCHES)
      foreach(EXCLUDE ${CTT_EXCLUDE_MATCHES})
        if(SRC MATCHES "${EXCLUDE}")
          set(ADD_SRC FALSE)
          _ctt_debug("EXCLUDE_MATCHES ${SRC}")
          continue()
        endif(SRC MATCHES "${EXCLUDE}")
      endforeach(EXCLUDE)
    endif(CTT_EXCLUDE_MATCHES)

    if(ADD_SRC)
      _ctt_debug("Add ${SRC}")
      get_filename_component(SRC "${SRC}" ABSOLUTE BASE_DIR ${CMAKE_CURRENT_LIST_DIR})
      list(APPEND SOURCES "${SRC}")
    endif(ADD_SRC)
  endforeach(SRC)

  _ctt_debug("SOURCES: ${SOURCES}")

  if(NOT SOURCES)
    message(SEND_ERROR "Target ${TARGET} has no post filter source files for clang-tidy")
    return()
  endif(NOT SOURCES)

  # We have a source list. We can get the compile flags from compile_commands.json if it exists.
  if("${CLANG_TIDY_LEVEL}" STREQUAL "all")
    foreach(LEVEL ${CLANG_TIDY_CONFIGS})
      if(NOT "${LEVEL}" STREQUAL "all")
        __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" CONFIG_LEVEL "${LEVEL}"
          BUILD_PATH "${CMAKE_BINARY_DIR}" ${SOURCES}
        )
        if(CLANG_APPLY_REPLACEMENTS_EXE)
          __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" FIX CONFIG_LEVEL "${LEVEL}"
            BUILD_PATH "${CMAKE_BINARY_DIR}" ${SOURCES}
          )
        endif(CLANG_APPLY_REPLACEMENTS_EXE)
      endif(NOT "${LEVEL}" STREQUAL "all")
    endforeach(LEVEL)
  else("${CLANG_TIDY_LEVEL}" STREQUAL "all")
    __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" CONFIG_LEVEL "${CLANG_TIDY_LEVEL}"
      BUILD_PATH "${CMAKE_BINARY_DIR}" ${SOURCES}
    )
    if(CLANG_APPLY_REPLACEMENTS_EXE)
      __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" FIX CONFIG_LEVEL "${CLANG_TIDY_LEVEL}"
        BUILD_PATH "${CMAKE_BINARY_DIR}" ${SOURCES}
      )
    endif(CLANG_APPLY_REPLACEMENTS_EXE)
  endif("${CLANG_TIDY_LEVEL}" STREQUAL "all")
endfunction(clang_tidy_target)

# Setup a single target which makes all the other clang-tidy targets.
function(__ctt_add_clang_tidy_global CONFIG_NAME CONFIG_LEVEL)
  if(NOT CLANG_TIDY_TARGETS_${CONFIG_LEVEL})
    message("No clang-tidy targets have been added for ${CONFIG_LEVEL}")
    return()
  endif(NOT CLANG_TIDY_TARGETS_${CONFIG_LEVEL})

  add_custom_target(clang-tidy-${CONFIG_LEVEL})
  add_dependencies(clang-tidy-${CONFIG_NAME} ${CLANG_TIDY_TARGETS_${CONFIG_LEVEL}})
endfunction(__ctt_add_clang_tidy_global)

# Setup a single target which makes all the other clang-tidy targets.
function(clang_tidy_global)
  if(CMAKE_GENERATOR MATCHES "Visual Studio")
    return()
  endif(CMAKE_GENERATOR MATCHES "Visual Studio")

  if("${CLANG_TIDY_LEVEL}" STREQUAL "all")
    set(ADDED_SOMETHING FALSE)
    foreach(LEVEL ${CLANG_TIDY_CONFIGS})
      if(NOT "${LEVEL}" STREQUAL "all")
        __ctt_add_clang_tidy_global(${LEVEL} ${LEVEL})
        if(CLANG_APPLY_REPLACEMENTS_EXE)
          __ctt_add_clang_tidy_global(${LEVEL}-fix ${LEVEL}-fix)
        endif(CLANG_APPLY_REPLACEMENTS_EXE)
        set(ADDED_SOMETHING TRUE)
      endif(NOT "${LEVEL}" STREQUAL "all")
    endforeach(LEVEL)

    if(NOT ADDED_SOMETHING)
    message("No clang-tidy targets have been added")
  endif(NOT ADDED_SOMETHING)

  else("${CLANG_TIDY_LEVEL}" STREQUAL "all")
    __ctt_add_clang_tidy_global(${CLANG_TIDY_LEVEL} ${CLANG_TIDY_LEVEL})
    if(CLANG_APPLY_REPLACEMENTS_EXE)
      __ctt_add_clang_tidy_global(${CLANG_TIDY_LEVEL}-fix ${CLANG_TIDY_LEVEL}-fix)
    endif(CLANG_APPLY_REPLACEMENTS_EXE)
  endif("${CLANG_TIDY_LEVEL}" STREQUAL "all")
endfunction(clang_tidy_global)
