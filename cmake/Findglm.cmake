
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
#   GLM_INCLUDE_DIRS, where to find glm/glm.hpp, etc.
#   glm_FOUND, If false, do not try to use GLM.
#   glm::glm, import target

# Try config file first. This particular file is really only for supporting older GLM such as found on Ubuntu 18.04.
# Newer GLM with config.cmake files is strongly recommended.
find_package(glm QUIET CONFIG)
if(NOT glm_FOUND)
  find_path(GLM_INCLUDE_DIRS NAMES glm/glm.hpp
            PATHS $ENV{GLM_ROOT_PATH}/include ${GLM_ROOT_PATH}/include
            /usr/local/include
            )

  mark_as_advanced(
      GLM_INCLUDE_DIRS
    )
endif()

# Note: glmConfig.cmake on Ubuntu 18.04 does not provide a target glm::glm.
# 20.04 does.
if(NOT TARGET glm::glm)
  add_library(glm::glm INTERFACE IMPORTED)
  set_target_properties(glm::glm PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GLM_INCLUDE_DIRS}"
  )
else()
  get_target_property(GLM_INCLUDE_DIRS glm::glm INTERFACE_INCLUDE_DIRECTORIES)
endif()

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(glm REQUIRED_VARS GLM_INCLUDE_DIRS)
