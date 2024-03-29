# Copyright (c) 2017
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

include(CMakeParseArguments)

find_package(Doxygen)

# doxygen_create(
#   [DOXYFILE doxyfile.in]
#   [PROJECT name]
#   [VERSION version]
#   [OUTPUT_DIR dir]
#   [CSS style.css]
#   [PUBLISHER name]
#   [PUBLISHER_ID id.publisher]
#   [PROJECT_ID id.publisher.project]
#   [EXAMPLE_PATHS path1 path2 ...]
#   [IMAGE_PATHS path1 path2 ...]
#   [EXCLUDE_PATHS path1 path2 ...]
#   PATHS path1 path2 ...
# )
#
# Create a custom target which builds the doxygen documentation.
#
# The function configures the Doxyfile, optionally specified by DOXYFILE, by converting various arguments to this
# function into the equivalent Doxyfile variables. By default the DOXYFILE is cmake/doxyfile.in.
# Note a full description of the Doxyfile variables and their function is beyond the scope of this documentation. See
# doxyfile documentation or comments in a Doxyfile generated by Doxywizard for more information.
#
# The function arguments are mapped as follows:
#
# Argument      | Doxygen Variable      | Description
# ------------- | --------------------- | ---------------------------------------------------------------------------------
# DOXYFILE      | N/A                   | The input doxyfile to configure using configure_file.
# PROJECT       | PROJECT_NAME          | Name of the project.
# VERSION       | PROJECT_NUMBER        | Project version number, in the form <major>.<minor>[.<patch>]
# OUTPUT_DIR    | HTML_OUPTUT           | HTML output directory under the build tree. Default is 'html'.
# CSS           | HTML_STYLESHEET       | Style sheet to style the documentation pages with.
# PUBLISHER     | DOCSET_PUBLISHER_NAME | Publisher name.
# PUBLISHER_ID  | DOCSET_PUBLISHER_ID   | Publisher ID in the form org.publisher
# PROJECT_ID    | DOCSET_BUNDLE_ID      | Project ID in the form org.publisher.project. Also maps to QHP_NAMESPACE, ECLIPSE_DOC_ID
# EXAMPLE_PATHS | EXAMPLE_PATH          | Where to find source files for @example tags.
# IMAGE_PATHS   | IMAGE_PATH            | Where to file images for @image tags.
# EXCLUDE_PATHS | EXCLUDE               | Exclude these directories from Doxygen parsing.
# PATHS         | INPUT                 | Directories to parse source for Doxygen comments. RECURSIVE is assumed on.
function(doxygen_create)
  cmake_parse_arguments(DGEN
    ""
    "PROJECT;VERSION;OUTPUT_DIR;CSS;PUBLISHER;PUBLISHER_ID;PROJECT_ID;DOXYFILE"
    "EXAMPLE_PATHS;IMAGE_PATHS;EXCLUDE_PATHS;PATHS"
    ${ARGN})

  # Doxygen configuration.
  # Short project name.
  if(DGEN_PROJECT)
    set(DOXYGEN_PROJECT_NAME "${DGEN_PROJECT}")
  else(DGEN_PROJECT)
    set(DOXYGEN_PROJECT_NAME "${CMAKE_PROJECT_NAME}")
  endif(DGEN_PROJECT)
  # Project version
  set(DOXYGEN_PROJECT_VERSION ${DGEN_VERSION})
  # Paths to example sources used with the @example tag.
  if(DGEN_EXAMPLE_PATHS)
    set(DOXYGEN_EXAMPLE_PATH "${DGEN_EXAMPLE_PATHS}")
  endif(DGEN_EXAMPLE_PATHS)
  # Paths to images included with the @image tag.
  if(DGEN_IMAGE_PATHS)
    set(DOXYGEN_IMAGE_PATH "${DGEN_IMAGE_PATHS}")
  endif(DGEN_IMAGE_PATHS)
  # Output subdirectory for generated documentation.
  if(DGEN_OUTPUT_DIR)
    set(DOXYGEN_HTML_OUTPUT ${DGEN_OUTPUT_DIR})
  else(DGEN_OUTPUT_DIR)
    set(DOXYGEN_HTML_OUTPUT html)
  endif(DGEN_OUTPUT_DIR)
  # CSS file used to style the documentation pages. Setting nothing here used the default Doxygen style.
  if(DGEN_CSS)
    set(DOXYGEN_CSS ${DGEN_CSS})
  endif(DGEN_CSS)
  # Publisher and project details.
  if(DGEN_PUBLISHER_ID)
    set(DOXYGEN_PUBLISHER_ID ${DOXYGEN_PUBLISHER_ID})
  endif(DGEN_PUBLISHER_ID)
  if(DGEN_PUBLISHER)
    set(DOXYGEN_PUBLISHER "${DOXYGEN_PUBLISHER}")
  endif(DGEN_PUBLISHER)

  if(DGEN_PROJECT_ID)
    set(DOXYGEN_PROJECT_ID ${DGEN_PROJECT_ID})
  endif(DGEN_PROJECT_ID)

  # Input directories:
  set(DOXYGEN_INPUT_DIRS)
  foreach(dir ${DGEN_PATHS})
    get_filename_component(dir ${dir} ABSOLUTE)
    set(DOXYGEN_INPUT_DIRS "${DOXYGEN_INPUT_DIRS} \\\n                         ${dir}")
  endforeach(dir)

  # Exclude directories
  set(DOXYGEN_EXCLUDE_DIRS)
  foreach(dir ${DGEN_EXCLUDE_PATHS})
    get_filename_component(dir ${dir} ABSOLUTE)
    set(DOXYGEN_EXCLUDE_DIRS "${DOXYGEN_EXCLUDE_DIRS} \\\n                         ${dir}")
  endforeach(dir)

  # Configure the doxyfile with the variables set above.
  if(NOT DGEN_DOXYFILE)
    set(DGEN_DOXYFILE "cmake/doxyfile.in")
  endif(NOT DGEN_DOXYFILE)
  configure_file("${DGEN_DOXYFILE}" "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
  get_filename_component(DOXYFILE_PATH "cmake/doxyfile.in" ABSOLUTE)

  # Setup the Doxygen target.
  add_custom_target(${DGEN_PROJECT}-doc
    "${DOXYGEN_EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/doxyfile"
    DEPENDS
      "${DOXYFILE_PATH}"
      "${CMAKE_CURRENT_BINARY_DIR}/doxyfile"
    )
  add_dependencies(${DGEN_PROJECT} ${DGEN_PROJECT}-doc)
endfunction(doxygen_create)

# doxygen_install(source destination)
#
# Setup an install command to install generated Doxygen documentation. The source specifies the directory under the
# build tree where to find the Doxygen pages. This relates to the OUTPUT_DIR option given to doxygen_create(). The
# destination specifies the subdirectory for the documentation pages and should generatelly match the project name.
function(doxygen_install dir dest)
  install(DIRECTORY ${dir} DESTINATION share/${dest} COMPONENT Devel)
endfunction(doxygen_install)
