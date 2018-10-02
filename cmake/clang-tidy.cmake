# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Add clang-tidy support at a per target level. Typical usage:
# - Optionally set CLANG_TIDY_PROJECT_PREFIX to define the prefix for the <PREFIX>_CLANG_TIDY option
#   - Default is "PROJECT"
# - include this file
# - Optionally set CTT_DEBUG on/true to enable debug messages.
# - For each target, call clang_tidy_target(<target>)

#-------------------------------------------------------------------------------
# Enable generation of compile_commands.json for Makefiles and Ninja.
#-------------------------------------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakeParseArguments)

# set(CTT_DEBUG ON)

#-------------------------------------------------------------------------------
# Setup clang-tidy CMake option. Uses the value of CLANG_TIDY_PROJECT_PREFIX as a prefix, defaulting to "PROJECT".
#-------------------------------------------------------------------------------
if(NOT CLANG_TIDY_PROJECT_PREFIX)
  set(CLANG_TIDY_PROJECT_PREFIX PROJECT)
endif(NOT CLANG_TIDY_PROJECT_PREFIX)

# Look for clang-tidy
find_program(CLANG_TIDY_EXE
             NAMES
              "clang-tidy"
              "clang-tidy-7.0"
              "clang-tidy-6.0"
              "clang-tidy-5.0"
             DOC "Path to clang-tidy executable")
# Enable clang-tidy if it was found.
set(CLANG_TIDY_INIT OFF)
if(CLANG_TIDY_EXE)
  set(CLANG_TIDY_INIT Targets)
endif(CLANG_TIDY_EXE)
# Create the CMake option.
set(${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY ${CLANG_TIDY_INIT} CACHE STRING "Enable clang-tidy. OFF: disabled. Post-build: as a post build step. Targets: explicit build targets named <target>-clang-tidy.")
set_property(CACHE ${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY PROPERTY STRINGS OFF Post-build Targets)

#-------------------------------------------------------------------------------
# Error checking.
#-------------------------------------------------------------------------------
if(${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY)
  if(NOT CLANG_TIDY_EXE)
    message(ERROR "clang-tidy executable not found.")
  endif(NOT CLANG_TIDY_EXE)
endif(${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY)

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

# _ctt_target_includes(var target [SYSTEM] [PROCESSED target1 target2 ...] [PREPEND flag1 flag2])
#
# Expand the include directives of TARGET into VAR. Directories are expanded as clang[-tidy] flags in either form:
#   -I<include_dir>
#   -isystem <include_dir>
#
# The former is used for local include paths, the latter for non-local/system includes; that is source you have no
# control over and should not generate clang-tidy errors.
#
# The default is the -I<include_dir> form, unless one of the following conditions are met:
# - SYSTEM is passed as an argument
# - TARGET is an IMPORT target (IMPORTED property is true)
# - A recursive call is made where a previous condition was met
#
# A recursive call will add the SYSTEM argument when the target has the following generator expression:
# - $<TARGET_PROPERTY:target,INTERFACE_INCLUDE_DIRECTORIES>
# We assume the target is an import target of some sort.
#
# The function handles and uses four generator expressions:
# - $<BUILD_INTERFACE:dir>
# - $<INSTALL_INTERFACE:dir>
# - $<TARGET_PROPERTY:target,INCLUDE_DIRECTORIES>
# - $<TARGET_PROPERTY:target,INTERFACE_INCLUDE_DIRECTORIES>
#
# BUILD_INTERFACE is enabled only for any non IMPORT target, while INSTALL_INTERFACE is enabled for all IMPORT targets.
#
# The PROCESSED argument is for internal use, tracking which targets have already been processed to avoid cycles.
#
# PREPEND is used to append existing flags to the final result and is used to collate results.
function(_ctt_target_includes VAR TARGET)
  _ctt_debug("_ctt_target_includes(${VAR} ${TARGET})")
  cmake_parse_arguments(CTI "SYSTEM" "" "PREPEND;PROCESSED" ${ARGN})

  # Return if TARGET already processed.
  if(CTI_PROCESSED AND ${TARGET} IN_LIST CTI_PROCESSED)
    return()
  endif(CTI_PROCESSED AND ${TARGET} IN_LIST CTI_PROCESSED)

  # Add to processed targets so as not to reprocess.
  list(APPEND CTI_PROCESSED ${TARGET})

  # - Get target include directories.
  get_target_property(IMPORTED_TARGET ${TARGET} IMPORTED)

  _ctt_debug("${TARGET} import? ${IMPORTED_TARGET}")

  if(IMPORTED_TARGET)
    get_target_property(TARGET_INCLUDES ${TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    # get_target_property(TARGET_LINK_LIBRARIES ${TARGET} LINK_LIBRARIES)
    set(CTI_SYSTEM TRUE) # Force system include pattern.
    set(ACTIVE_INTERFACE INSTALL)
  else(IMPORTED_TARGET)
    get_target_property(TARGET_INCLUDES ${TARGET} INCLUDE_DIRECTORIES)
    # get_target_property(TARGET_LINK_LIBRARIES ${TARGET} LINK_LIBRARIES)
    set(ACTIVE_INTERFACE BUILD)
  endif(IMPORTED_TARGET)

  # Propagate use of system pattern.
  set(SYSTEM_ARG)
  if(CTI_SYSTEM)
    set(SYSTEM_ARG SYSTEM)
  endif(CTI_SYSTEM)

  # Resolve generator expressions:
  # - BUILD_INTERFACE or INSTALL_INTERFACE for INTERFACE targets.
  #   - Select by value of ACTIVE_INTERFACE
  # - $<TARGET_PROPERTY:target,INCLUDE_DIRECTORIES> by recursing.
  # - $<TARGET_PROPERTY:target,INTERFACE_INCLUDE_DIRECTORIES> by recursing with INTERFACE argument.
  _ctt_debug("${TARGET} INCLUDES: ${TARGET_INCLUDES}")

  set(INCLUDE_FLAGS)
  if(TARGET_INCLUDES)
    foreach(INC ${TARGET_INCLUDES})
      set(ADD_INC NO)
      # Either BUILD_INTERFACE or INSTALL_INTERFACE depending on ${ACTIVE_INTERFACE}
      if(INC MATCHES "^\\$<${ACTIVE_INTERFACE}_INTERFACE:.*>$")
        string(REGEX REPLACE "^\\$<${ACTIVE_INTERFACE}_INTERFACE:(.*)>$" "\\1" INC ${INC})
        set(ADD_INC YES)
      elseif(INC MATCHES "^\\$<TARGET_PROPERTY:.*,(INCLUDE_DIRECTORIES|INTERFACE_INCLUDE_DIRECTORIES)>$")
        # TARGET_PROPERTY. Test for INCLUDE_DIRECTORIES or INTERFACE_INCLUDE_DIRECTORIES
        string(REGEX REPLACE "^\\$<TARGET_PROPERTY:(.*),(.*)>$" "\\1" SUBTARGET ${INC})
        _ctt_target_includes(${TARGET}_INCLUDE_FLAGS ${SUBTARGET} ${SYSTEM_ARG} PROCESSED ${CTI_PROCESSED} PREPEND ${CTI_PREPEND})
        set(INCLUDE_FLAGS ${INCLUDE_FLAGS} ${${TARGET}_INCLUDE_FLAGS})
      elseif(NOT INC MATCHES "^\\$<.*>$")
        set(ADD_INC YES)
      endif()

      if(ADD_INC)
        # - Set command line include directory prefix:
        #   - "-I" for local include directories
        #   - "-isystem " for imported include directories (note the trailing space)
        if(NOT CTI_SYSTEM)
          list(APPEND INCLUDE_FLAGS "-I${INC}")
        else(NOT CTI_SYSTEM)
          list(APPEND INCLUDE_FLAGS "-isystem")
          list(APPEND INCLUDE_FLAGS "${INC}")
        endif(NOT CTI_SYSTEM)
      endif(ADD_INC)
    endforeach(INC)
  endif(TARGET_INCLUDES)
  _ctt_debug("INCLUDE_FLAGS(${TARGET}): ${INCLUDE_FLAGS}")

  # Propagate working variables up a scope
  if(INCLUDE_FLAGS)
    if(NOT CTI_PREPEND)
      unset(CTI_PREPEND)
    endif(NOT CTI_PREPEND)
    set(${VAR} ${CTI_PREPEND} ${INCLUDE_FLAGS} PARENT_SCOPE)
  endif(INCLUDE_FLAGS)
endfunction(_ctt_target_includes)


# _ctt_collect_link_targets(var target [PROCESSED target1 target2 ...])
#
# Collect all link dependencies for target including chaining dependencies.
#
# The PROCESSED argument is for internal use, tracking which targets have already been processed to avoid cycles.
function(_ctt_collect_link_targets VAR TARGET)
  _ctt_debug("_ctt_collect_link_targets(${VAR} ${TARGET})")

  cmake_parse_arguments(CLT "" "" "PROCESSED" ${ARGN})

  # Return if TARGET already processed.
  if(CLT_PROCESSED AND ${TARGET} IN_LIST CLT_PROCESSED)
    return()
  endif(CLT_PROCESSED AND ${TARGET} IN_LIST CLT_PROCESSED)

  # Add to processed targets so as not to reprocess.
  list(APPEND CLT_PROCESSED ${TARGET})

  get_target_property(IMPORTED_TARGET ${TARGET} IMPORTED)
  if(IMPORTED_TARGET)
    get_target_property(TARGET_LINK_LIBRARIES ${TARGET} INTERFACE_LINK_LIBRARIES)
  else(IMPORTED_TARGET)
    get_target_property(TARGET_LINK_LIBRARIES ${TARGET} LINK_LIBRARIES)
  endif(IMPORTED_TARGET)

  # _ctt_debug("... ${TARGET}: ${TARGET_LINK_LIBRARIES}")

  # Recurse on link dependencies. For any dependency which is a target, CMake automatically adds the target's
  # INCLUDE_DIRECTORIES or INTERFACE_INCLUDE_DIRECTORIES for compilation.
  if(TARGET_LINK_LIBRARIES)
    foreach(LIB ${TARGET_LINK_LIBRARIES})
      if(TARGET ${LIB})
        _ctt_collect_link_targets(${VAR} ${LIB} PROCESSED ${CLT_PROCESSED})
      endif(TARGET ${LIB})
    endforeach(LIB)
  endif(TARGET_LINK_LIBRARIES)

  set(${VAR} ${TARGET} ${${VAR}} PARENT_SCOPE)
endfunction(_ctt_collect_link_targets)

# _ctt_include_link_targets(var target1 target2 ...)
# Generate include flags for clang-tidy for the listed targets.
function(_ctt_include_link_targets VAR)
  set(${VAR})
  _ctt_debug("_ctt_include_link_targets(${VAR} ${ARGN})")
  foreach(TARGET ${ARGN})
    _ctt_target_includes(${TARGET}_INC ${TARGET})
    set(${VAR} ${${VAR}} ${${TARGET}_INC})
  endforeach(TARGET)
  set(${VAR} ${${VAR}} PARENT_SCOPE)
endfunction(_ctt_include_link_targets)

function(__ctt_setup_target TARGET WORKING_DIRECTORY)
  if(${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY STREQUAL "Post-build")
    add_custom_command(TARGET ${TARGET}
      POST_BUILD
      WORKING_DIRECTORY "${WORKING_DIRECTORY}"
      COMMAND "${CLANG_TIDY_EXE}" ${ARGN}
    )
  else()
    add_custom_target(${TARGET}-clang-tidy
      WORKING_DIRECTORY "${WORKING_DIRECTORY}"
      COMMAND "${CLANG_TIDY_EXE}" ${ARGN}
    )
    set_target_properties(${TARGET}-clang-tidy PROPERTIES FOLDER clang-tidy)
  endif()
endfunction(__ctt_setup_target TARGET WORKING_DIRECTORY)

# clang_tidy_target(
#   target
#   [EXCLUDE file1 file2 ...]
#   [EXCLUDE_MATCHES regex1 regex2 ...]
#   [INCLUDE_HEADERS]
# )
#
# Add clang tidy support for a target. Requires a clang-tidy executable be found and the
# ${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY option be enabled.
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
# Header files are matched using the regular expression ".*\.(h|hpp|hxx|inl)"
#
# The include directories are filtered for explict directories, $<BUILD_INTERFACE:xxx> and
# $<TARGET_PROPERTY:target,INCLUDE_DIRECTORIES> generator expressions, adding both to the resulting include directory
# list.
#
# Finally, this mode of operation also considered the C++ standard using the CXX_STANDARD target property.
function(clang_tidy_target TARGET)
  if(NOT ${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY)
    return()
  endif(NOT ${CLANG_TIDY_PROJECT_PREFIX}_CLANG_TIDY)

  if(NOT CMAKE_VERSION VERSION_LESS 3.6)
    # From CMake 3.6+, Ninja and Makefiles have in-built clang-tidy support.
    # We assume support if compile_commands.json has been generated.
    if(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
      set_target_properties(${TARGET} PROPERTIES
        CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-p;${CMAKE_BINARY_DIR}"
      )
      return()
    endif(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
  endif(NOT CMAKE_VERSION VERSION_LESS 3.6)

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
      set(CTT_EXCLUDE_MATCHES "${HEADER_REX}")
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
      list(APPEND SOURCES "${SRC}")
    endif(ADD_SRC)
  endforeach(SRC)

  _ctt_debug("SOURCES: ${SOURCES}")

  if(NOT SOURCES)
    message(SEND_ERROR "Target ${TARGET} has no post filter source files for clang-tidy")
    return()
  endif(NOT SOURCES)

  # We have a source list. We can get the compile flags from compile_commands.json if it exists.
  if(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")
    __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" -p "${CMAKE_BINARY_DIR}" ${SOURCES})
    return()
  endif(EXISTS "${CMAKE_BINARY_DIR}/compile_commands.json")

  # No compile_commands.json. We have to resolve the include directories and C++ standard.
  # This won't exactly match the command line, but should be good enough.
  # This path is primarily for Windows.

  # Resolve include directories
  _ctt_collect_link_targets(INCLUDE_TARGETS ${TARGET})
  _ctt_include_link_targets(INCLUDES ${INCLUDE_TARGETS})

  _ctt_debug("${TARGET} INCLUDES: ${INCLUDES}")

  # Handle special Qt generated file directories (as system to disable linting)
  get_target_property(AUTOGEN_INCLUDES ${TARGET} AUTOGEN_BUILD_DIR)
  if(AUTOGEN_INCLUDES)
    _ctt_debug("${TARGET} AUTOGEN_INCLUDES: ${AUTOGEN_INCLUDES}")
    foreach(INC ${AUTOGEN_INCLUDES})
      # Add as system to avoid errors.
      list(APPEND INCLUDES -system ${INC})
    endforeach(INC)
  else(AUTOGEN_INCLUDES)
    # Use defult autogen include patterns...
    if(CMAKE_AUTOMOC OR CMAKE_AUTORCC OR CMAKE_AUTOUIC)
      list(APPEND INCLUDES -isystem "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_autogen/include")
      list(APPEND INCLUDES -isystem "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_autogen/include_Debug")
      list(APPEND INCLUDES -isystem "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_autogen/include_MinSizeRel")
      list(APPEND INCLUDES -isystem "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_autogen/include_Release")
      list(APPEND INCLUDES -isystem "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_autogen/include_RelWithDebInfo")
    endif(CMAKE_AUTOMOC OR CMAKE_AUTORCC OR CMAKE_AUTOUIC)
  endif(AUTOGEN_INCLUDES)

  _ctt_debug("FINAL --- ${TARGET} INCLUDES: ${INCLUDES}")

  # Work out the C++ standard.
  get_target_property(CXXSTD ${TARGET} CXX_STANDARD)
  if(CXXSTD AND NOT CXXSTD LESS 11)
    set(CXXSTD "--std=c++${CXXSTD}")
  endif(CXXSTD AND NOT CXXSTD LESS 11)
  _ctt_debug("CXXSTD: ${CXXSTD}")

  # Setup custom clang-tidy build step.
  __ctt_setup_target(${TARGET} "${CMAKE_CURRENT_LIST_DIR}" ${SOURCES} -- ${INCLUDES} ${CXXSTD})
endfunction(clang_tidy_target)
