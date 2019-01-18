include(CMakeParseArguments)

set(_TEXT_FILE_RESOURCE_PY "${CMAKE_CURRENT_LIST_DIR}/TextFileResource.py")

# text_file_resource(
#     textFile resourceName
#     [FILELIST varname]
#     [TYPE filetype]
#     [INCLUDE_PATTERN pattern]
#     [COMMENT_PATTERN pattern]
#     [NO_IMPORT]
#     [MINIFY]
#     [BINARY]
#     [ECHO]
#     [PATHS path1 path2 ...]
# )
#
# Take a text file and generate a C++ header and source file containing a string variable with the file content.
# Can optionally expand include/import statements either inferred by file type or by using a regular expression match.
# This essentially will paste "included" file contents at the include location
#
# The resource files are generated in the CMAKE_CURRENT_BINARY_DIR as follows:
# - Generates a .h and a .cpp file for the TEXT_FILE
# - Generated file names are based on TEXT_FILE:
#   - Remove directory and extension from TEXT_FILE.
#   - Append 'Resource'
# - Generated files define a string variable named RESOURCE_NAME.
#
# The FILELIST argument may be used to track the generated files. The two generated files are appended to the specified
# variable.
#
# For example:
#   text_file_resource(resources/text.txt EmbeddedText FILELIST RESOURCE_FILES)
# This generates the following:
# - textResource.h and textResource.cpp in CMAKE_CURRENT_BINARY_DIR.
# - Define the variable EmbeddedText containing the content of resource/text.txt.
# - RESOURCE_FILES has the following appended to it: ${CMAKE_CURRENT_BINARY_DIR}/textResource.h and
#   ${CMAKE_CURRENT_BINARY_DIR}/textResource.cpp
#
# Include/import statement expansion is supported when the IMPORT argument is given. The include pattern is normally
# inferred by file type; for example, the include pattern for a C/C++ file is:
#   #include <file_path>
# for which we use the following regular expression:
#     ^[ \t]*#include[ \t]+[<"](.*)[>"].*$
#
# The include pattern may be overridden by specifying INCLUDE_PATTERN pattern where pattern is a a regular expression.
# This expression must have a capture (1) which isolates the included file path.
#
# Such expressions are located and replaced by the contents for <file_path>. We search for the
# file_path first in the same directy as the origin file, but also on the given PATHS. This behaviour may be disabled
# by adding the NO_IMPORT argument.
#
# Note that string literals are limited to less than 2^16 characters. Longer content may be handled by the following
# options:
# - MINIFY : strips leading and trailing whitespace and removes 'comment' strings (see COMMENT_PATTERN).
# - BINARY : inserts the text as an array of bytes instead of as a string.
#
# Note that MINIFY will yield shorter strings, but they may still be too larger. BINARY mode is always invoked when
# the string to be written is too long.
#
# MINIFY supports comment stripping, normally identified by the file type. The comment regular expression may be
# overridden by specifying COMMENT_PATTERN pattern.
#
# The resource type is normally inferred by file extension, but may also be explicitly selected. The following
# table identifies the supported types and the extentions.
#
# Type    | Language  | Extensions
# ------- | --------- | ----------
# c       | C         | c, h
# c#      | C#        | cs
# cmake   | CMake     | cmake, CMakeLists.txt
# cpp     | C++       | cpp, cxx, h, hpp, hxx
# cuda    | CUDA      | cu
# java    | Java      | java
# opencl  | OpenCL    | cl
# python  | Python    | py
#
# Include/import example:
#   text_file_resource(gpucode.cl GpuCode
#       PATHS ${CMAKE_CURRENT_LIST_DIR} anotherPath
#   )
# The INCLUDE_PATTERN matches patterns such as '#include "somefile.h"' with the capture extracting 'somefile.h'.
# This will be replaced with the contents of 'somefile.h' resolved from one of the PATHS.
#
# Finally, the command line executed may be shown by adding ECHO.
#
# FIXME: strings are limited to 2^16 - 1 characters. Provide a workaround.
function(text_file_resource TEXT_FILE RESOURCE_NAME)
  # Requires python to do the conversion.
  find_package(PythonInterp 3 REQUIRED)

  cmake_parse_arguments(TFR "BINARY;MINIFY;NO_IMPORT;ECHO" "COMMENT_PATTERN;INCLUDE_PATTERN;FILELIST;TYPE" "PATHS" ${ARGN})

  # Prepare command line arguments.
  set(_ARGS)
  if(TFR_BINARY)
    list(APPEND ARGS "--mode=binary")
  endif()
  if(TFR_MINIFY)
    list(APPEND ARGS "--minify=on")
  endif()
  if(TFR_NO_IMPORT)
    list(APPEND ARGS "--no-import")
  endif()
  if(TFR_INCLUDE_PATTERN)
    list(APPEND ARGS "--include-regex=\"${TFR_INCLUDE_PATTERN}\"")
  endif()
  if(TFR_COMMENT_PATTERN)
    list(APPEND ARGS "--comment-regex=\"${TFR_COMMENT_PATTERN}\"")
  endif()
  if(TFR_TYPE)
    list(APPEND ARGS "--style=${TFR_TYPE}")
  endif()

  foreach(PATH ${TFR_PATHS})
    list(APPEND ARGS "-I\"${PATH}\"")
  endforeach(PATH)

  get_filename_component(BASE_FILE "${TEXT_FILE}" NAME_WE)
  string(APPEND BASE_FILE Resource)

  if(TFR_FILELIST)
    list(APPEND ${TFR_FILELIST}
      "${CMAKE_CURRENT_BINARY_DIR}/${BASE_FILE}.cpp"
      "${CMAKE_CURRENT_BINARY_DIR}/${BASE_FILE}.h")
    set(${TFR_FILELIST} ${${TFR_FILELIST}} PARENT_SCOPE)
    # message("...${TFR_FILELIST}: ${CMAKE_CURRENT_BINARY_DIR}/${RESOURCE_NAME}.cpp ${CMAKE_CURRENT_BINARY_DIR}/${RESOURCE_NAME}.h")
  endif(TFR_FILELIST)

  if(TFR_ECHO)
    message("${PYTHON_EXECUTABLE} ${_TEXT_FILE_RESOURCE_PY} ${TEXT_FILE} ${CMAKE_CURRENT_BINARY_DIR}/${BASE_FILE} --name=${RESOURCE_NAME} ${ARGS}")
  endif(TFR_ECHO)

  # Python 3 available. Use python version.
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" "${_TEXT_FILE_RESOURCE_PY}" "${TEXT_FILE}" "${CMAKE_CURRENT_BINARY_DIR}/${BASE_FILE}" --name=${RESOURCE_NAME} ${ARGS}
    OUTPUT_VARIABLE PROCESS_OUTPUT ERROR_VARIABLE PROCESS_OUTPUT
    RESULT_VARIABLE EXIT_CODE
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")

  if(NOT EXIT_CODE EQUAL 0)
    message(SEND_ERROR "text_file_resource() failed (${ERROR_CODE})")
    message("${PROCESS_OUTPUT}")
  elseif(PROCESS_OUTPUT)
  message("text_file_resource():")
  message("${PROCESS_OUTPUT}")
  endif()
endfunction(text_file_resource)
