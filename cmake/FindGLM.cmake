
# FindGLM (OpenGL Mathematics library)
# --------
#
# try to find include files.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``glm::glm``
#  Defined if the system has OpenGL Mathematics headers.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   GLM_INCLUDE_DIR, where to find glm/glm.hpp, etc.
#   GLM_FOUND, If false, do not try to use GLM.

# Try config file first. This particular file is really only for supporting older GLM such as found on Ubuntu 18.04.
# Newer GLM with config.cmake files is stringly recommended.
find_package(glm QUIET CONFIG)
if(GLM_FOUND)
  return()
endif(GLM_FOUND)

find_path(GLM_INCLUDE_DIR NAMES glm/glm.hpp
          PATHS $ENV{GLM_ROOT_PATH}/include ${GLM_ROOT_PATH}/include
          /usr/local/include
          )

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLM REQUIRED_VARS GLM_INCLUDE_DIR)

if (GLM_FOUND)
  if(NOT TARGET glm::glm)
    add_library(glm::glm INTERFACE IMPORTED)
    set_target_properties(glm::glm PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${GLM_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(
  GLM_INCLUDE_DIR
  )
