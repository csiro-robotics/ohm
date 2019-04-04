# Copyright (c) 2018
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# A collection of useful utility functions.

include(CMakeParseArguments)

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
