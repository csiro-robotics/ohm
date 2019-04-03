# Copyright (c) 2019
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# This script adds memory leak tracking to any exectuable target built using GCC or Clang via Address Sanitizer.
#
# Oustanding memory allocations are reported at the end of the run to stderr.
#
# Expected usage is as follows, written as a CMake script (with comments);
#
# # Find the package containing this script
# find_package(ras_cmake)
# # Import this script
# include(LeakTrack)
# # Define a CMake option() to toggle leak tracking. Make sure you enable this as required.
# option(MY_PROJECT_LEAK_TRACK "Enable memory leak tracking?" OFF)
# # Define your exectuable target. Also works with cuda_add_executable()
# add_executable(myprogram)
# # Conditionally enable leak tracking when MY_PROJECT_LEAK_TRACK is true
# leak_track_target_enable(myprogram CONDITION MY_PROJECT_LEAK_TRACK)
# # For a CUDA based program, ensure you disable protect_shadow_gap checking:
# leak_track_default_options(myprogram CONDITION MY_PROJECT_LEAK_TRACK "protect_shadow_gap=0")
#
#
# This enables all leak reporting for myprogram. Optionally, you can add additional ASAN options by passing additional
# arguments to leak_track_default_options(). See Overview resource below.
#
# There can also be spurrious memory leaks reported, or leaks in 3rd party libraries which cannot be dealt with. These
# can be suppressed from the leak report by adding:
# leak_track_suppress(myprogram CONDITION MY_PROJECT_LEAK_TRACK
#     symbol1
#     symbol2
#     symbol3
# )
# Where 'symbol#' matches part of the reported callstack symbols.
#
# If leak tracking is supported using AddressSanitizer, then the CMake cache variable LEAK_TRACK_WITH_ASAN will be set
# ON.
#
# Memory leak tracking may also be forcibly enabled vai the CMake option LEAK_TRACK_FORCE. This is intended as a global,
# consistent option in multi-project environments.
#
# Note that these directives are ignored when not building with GCC or Clang.
#
# The following links provide some much needed information about using libasan, althought the documentation is scarce:
# - Overview: https://github.com/google/sanitizers/wiki/AddressSanitizer
# - Weak symbols for in code overrides: https://chromium.googlesource.com/chromium/src/build/+/master/sanitizers/sanitizer_options.cc

include(CMakeParseArguments)

set(LEAK_TRACK_WITH_ASAN OFF CACHE STRING "Using libasan for leak tracking" FORCE)
mark_as_advanced(LEAK_TRACK_WITH_ASAN)

option(LEAK_TRACK_FORCE "Force memory leak tracking to be enabled regardless of condition checks?" OFF)

# Setup: find libasan
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  find_library(ASAN_LIBRARAY NAMES asan libasan.so libasan.so.4)
  if(ASAN_LIBRARAY)
    set(LEAK_TRACK_WITH_ASAN ON CACHE STRING "Using libasan for leak tracking" FORCE)
  else(ASAN_LIBRARY)
    message("Unable to find libasan. Leak tracking unavailable.")
    set(TARGET_ENABLE_ASAN_ONCE TRUE)
  endif(ASAN_LIBRARAY)
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")

# Utility function to extend a property on a target by adding a list of items.
#
# _extend_target_property(<target> <property> [FIRST] value1 [value2 ...])
#
# The FIRST keyword forces the new values to appear first in the property value list.
function(_leak_track_extend_target_property TARGET PROPERTY)
  cmake_parse_arguments(ETP "FIRST" "" "" ${ARGN})
  get_target_property(INITIAL_VALUE ${TARGET} ${PROPERTY})

  if(ETP_FIRST)
    set(MODIFIED_VALUE ${ETP_UNPARSED_ARGUMENTS})
    if(INITIAL_VALUE)
      list(APPEND MODIFIED_VALUE ${INITIAL_VALUE})
    endif(INITIAL_VALUE)
  else(ETP_FIRST)
    if(INITIAL_VALUE)
      set(MODIFIED_VALUE ${INITIAL_VALUE})
      list(APPEND MODIFIED_VALUE ${ETP_UNPARSED_ARGUMENTS})
    else(INITIAL_VALUE)
      set(MODIFIED_VALUE ${ETP_UNPARSED_ARGUMENTS})
    endif(INITIAL_VALUE)
  endif(ETP_FIRST)

  set_target_properties(${TARGET} PROPERTIES ${PROPERTY} "${MODIFIED_VALUE}")
endfunction(_leak_track_extend_target_property TARGET PROPERTY CONTENT)

# Add inline source to be built as part of a target.
#
# _leak_track_add_source(<target> <filename> <source_code>)
function(_leak_track_add_source TARGET FILENAME SOURCE)
  # Create the source in a dummy file initially, then marshal to the actual target location if different.
  # This prevents redundant compilation.

  # Define the output directory.
  set(SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_leak")

  # Write to an intermediate file
  file(WRITE "${SOURCE_DIR}/${FILENAME}.in" "${SOURCE}")

  # See if the intermediate file differs from the intended target file.
  execute_process(COMMAND
    ${CMAKE_COMMAND} -E compare_files
      "${SOURCE_DIR}/${FILENAME}.in"
      "${SOURCE_DIR}/${FILENAME}"
      OUTPUT_VARIABLE STDOUT_LOG
      ERROR_VARIABLE STDERR_LOG
      RESULT_VARIABLE NEEDS_UPDATE)
  if(NEEDS_UPDATE)
    execute_process(COMMAND
      ${CMAKE_COMMAND} -E copy
        "${SOURCE_DIR}/${FILENAME}.in"
        "${SOURCE_DIR}/${FILENAME}")
  endif(NEEDS_UPDATE)

  target_sources(${TARGET} PRIVATE "${SOURCE_DIR}/${FILENAME}")
endfunction(_leak_track_add_source)

# Enable libasan for a target. The target must be an exectuable or this command is ignored.
#
# leak_track_target_enable_asan(<target> [todo-options] [todo-suppress])
#
# This adds the required address santisiation parameters. This process also supports adding Address Sanitizer options
# and supressions.
function(leak_track_target_enable_asan TARGET)
  get_target_property(TARGET_TYPE ${TARGET} TYPE)

  if(TARGET_TYPE STREQUAL "EXECUTABLE")
    # Extend the library linking properties directly to ensure that libasan ends up first. This is a requirement for asan.
    # Publically link the asan library and add the compiler options, while privately linking the asan_config library to
    # configure asan properties.
    _leak_track_extend_target_property(${TARGET} LINK_LIBRARIES FIRST ${ASAN_LIBRARAY})# asan_config)
    _leak_track_extend_target_property(${TARGET} COMPILE_OPTIONS -fsanitize=address -fno-omit-frame-pointer)
  else(TARGET_TYPE STREQUAL "EXECUTABLE")
    message("Cannot enable memory leak tracking for ${TARGET}: only executables can have leak tracking enabled")
  endif(TARGET_TYPE STREQUAL "EXECUTABLE")
endfunction(leak_track_target_enable_asan TARGET)

# Check for a CONDITION keywork argument and evaluate it.
#
# _leak_track_check_condition(<variable> <unparsed_args> ${ARGN})
#
# Typical usage is:
#   _leak_track_check_condition(CONDITION_OK _ARGN ${ARGN})
#   if(NOT CONDITION_OK)
#     return()
#   endif(NOT CONDITION_OK)
#
# This extracts the CONDITION keyword argument and sets the variable (CONDITION_OK) if that argument expression
# evaluates to true. The variable is also set to true if there is no CONDITION argument.
#
# Unused values from ${ARGN} are written to ${UNPARSED_VARIABLE}
function(_leak_track_check_condition VARIABLE UNPARSED_VARIABLE)
  cmake_parse_arguments(LTC "" "CONDITION" "" ${ARGN})
  set(${VARIABLE} ON PARENT_SCOPE)
  if(LTC_CONDITION AND NOT LEAK_TRACK_FORCE)
    # Have a condition to check
    if(${LTC_CONDITION})
      set(${VARIABLE} ON PARENT_SCOPE)
    else(${LTC_CONDITION})
    set(${VARIABLE} OFF PARENT_SCOPE)
    endif(${LTC_CONDITION})
  endif(LTC_CONDITION AND NOT LEAK_TRACK_FORCE)
  set(${UNPARSED_VARIABLE} ${LTC_UNPARSED_ARGUMENTS} PARENT_SCOPE)
endfunction(_leak_track_check_condition)

# Suppress a list of memory leaks for a target.
#
# leak_track_suppress(<target> suppress1 [suppress2...])
#
# To suppress memory leaks, the function is provided a list of symbols from which to ignore memory leaks. These symbols
# can be matched anywhere in the stack to be ignored.
function(leak_track_suppress TARGET)
  if(NOT LEAK_TRACK_WITH_ASAN)
    return()
  endif(NOT LEAK_TRACK_WITH_ASAN)

  _leak_track_check_condition(CONDITION_OK _ARGN ${ARGN})
  if(NOT CONDITION_OK)
    return()
  endif(NOT CONDITION_OK)

  set(SUPPRESS_LIST)

  foreach(ARG ${_ARGN})
    set(SUPPRESS_LIST "${SUPPRESS_LIST}leak:${ARG}\\n")
  endforeach(ARG)

  set(SUPPRESS_SOURCE "
#ifdef __cplusplus
extern \"C\"
{
#endif // __cplusplus
const char *__lsan_default_suppressions()
{
  static const char *kDefaultSuppressions = \"${SUPPRESS_LIST}\";
  return kDefaultSuppressions;
}
#ifdef __cplusplus
}
#endif // __cplusplus
")
  _leak_track_add_source(${TARGET} asanSuppress.cpp "${SUPPRESS_SOURCE}")
endfunction(leak_track_suppress TARGET)

# Suppress a list of memory leaks for a target.
#
# leak_track_suppress(<target> [CONDITION condition] suppress1 [suppress2...])
#
# To suppress memory leaks, the function is provided a list of symbols from which to ignore memory leaks. These symbols
# can be matched anywhere in the stack to be ignored.
function(leak_track_default_options TARGET)
  if(NOT LEAK_TRACK_WITH_ASAN)
    return()
  endif(NOT LEAK_TRACK_WITH_ASAN)

  _leak_track_check_condition(CONDITION_OK _ARGN ${ARGN})
  if(NOT CONDITION_OK)
    return()
  endif(NOT CONDITION_OK)

  set(OPTION_LIST)

  foreach(ARG ${_ARGN})
    set(OPTION_LIST "${OPTION_LIST}${ARG} ")
  endforeach(ARG)

  set(OPTIONS_SOURCE "
#ifdef __cplusplus
extern \"C\"
{
#endif // __cplusplus
const char *__asan_default_options()
{
  static const char *kDefaultOptions = \"${OPTION_LIST}\";
  return kDefaultOptions;
}
#ifdef __cplusplus
}
#endif // __cplusplus
")
  _leak_track_add_source(${TARGET} asanOptions.cpp "${OPTIONS_SOURCE}")
endfunction(leak_track_default_options TARGET)

# Enable memory leak checking for a target.
#
# leak_track_target_enable(<target> [CONDITION condition])
#
# Optionally specify a CONDITION to check, only enabling leak tracking if this passes. Typically this would be the name
# of a CMake option() variable.
function(leak_track_target_enable TARGET)
  _leak_track_check_condition(CONDITION_OK _ARGN ${ARGN})
  if(NOT CONDITION_OK)
    return()
  endif(NOT CONDITION_OK)

  if(LEAK_TRACK_WITH_ASAN)
    leak_track_target_enable_asan(${TARGET})
  endif(LEAK_TRACK_WITH_ASAN)
endfunction(leak_track_target_enable)
