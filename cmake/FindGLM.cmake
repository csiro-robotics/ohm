
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
# ``GLM``
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

find_path(GLM_INCLUDE_DIR NAMES glm/glm.hpp
          PATHS $ENV{GLM_ROOT_PATH}/include ${GLM_ROOT_PATH}/include
          /usr/local/include
          )

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLM REQUIRED_VARS GLM_INCLUDE_DIR)

if (GLM_FOUND)
  if(NOT TARGET GLM)
    add_library(GLM UNKNOWN IMPORTED)
    set_target_properties(GLM PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${GLM_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(
  GLM_INCLUDE_DIR
  )
