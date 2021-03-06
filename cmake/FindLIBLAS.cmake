# This module searches liblas and defines
# LIBLAS_LIBRARIES - link libraries
# LIBLAS_RUNTIME_LIBRARIES - runtime binaries (DLLs)
# LIBLAS_FOUND, if false, do not try to link
# LIBLAS_INCLUDE_DIR, where to find the headers
#
# $LIBLAS_ROOT is an environment variable that would

set(LL_HEADER liblas/liblas.hpp)
set(LL_HEADER_SUFFIX include)
set(LL_LIB liblas las)
set(LL_LIB_DEBUG)
set(LL_SHARED)
set(LL_SHARED_DEBUG)

foreach(LLIB ${LL_LIB})
  list(APPEND LL_LIB_DEBUG ${LLIB}d)
endforeach(LLIB)

foreach(LLIB ${LL_LIB})
  list(APPEND LL_SHARED ${LLIB}${CMAKE_SHARED_LIBRARY_SUFFIX})
endforeach(LLIB)

foreach(LLIB ${LL_LIB_DEBUG})
  list(APPEND LL_SHARED_DEBUG ${LLIB}${CMAKE_SHARED_LIBRARY_SUFFIX})
endforeach(LLIB)

# Target the C API if COMPONENTS specified as "capi"
if(LIBLAS_FIND_COMPONENTS STREQUAL "capi")
  set(LL_HEADER liblas/capi/liblas.h)
  set(LL_HEADER_SUFFIX include)
  set(LL_LIB liblas_c las_c)
else(LIBLAS_FIND_COMPONENTS STREQUAL "capi")
endif(LIBLAS_FIND_COMPONENTS STREQUAL "capi")

find_path(LIBLAS_INCLUDE_DIR ${LL_HEADER} HINTS ENV LIBLAS_ROOT PATH_SUFFIXES ${LL_HEADER_SUFFIX})

find_library(LIBLAS_LIBRARY_DEBUG NAMES ${LL_LIB_DEBUG} HINTS ENV LIBLAS_ROOT PATH_SUFFIXES lib)
find_library(LIBLAS_LIBRARY_RELEASE NAMES ${LL_LIB} HINTS ENV LIBLAS_ROOT PATH_SUFFIXES lib)

find_file(LIBLAS_RUNTIME_DEBUG NAMES ${LL_SHARED_DEBUG} HINTS ENV LIBLAS_ROOT PATH_SUFFIXES bin)
find_file(LIBLAS_RUNTIME_RELEASE NAMES ${LL_SHARED} HINTS ENV LIBLAS_ROOT PATH_SUFFIXES bin)

if(LIBLAS_LIBRARY_DEBUG)
  list(APPEND LIBLAS_LIBRARIES debug ${LIBLAS_LIBRARY_DEBUG})
  if(LIBLAS_LIBRARY_RELEASE)
    list(APPEND LIBLAS_LIBRARIES optimized ${LIBLAS_LIBRARY_RELEASE})
  endif(LIBLAS_LIBRARY_RELEASE)
else(LIBLAS_LIBRARY_DEBUG)
  list(APPEND LIBLAS_LIBRARIES ${LIBLAS_LIBRARY_RELEASE})
endif(LIBLAS_LIBRARY_DEBUG)

if(LIBLAS_RUNTIME_DEBUG)
  list(APPEND LIBLAS_RUNTIME_LIBRARIES debug ${LIBLAS_RUNTIME_DEBUG})
  if(LIBLAS_RUNTIME_RELEASE)
    list(APPEND LIBLAS_RUNTIME_LIBRARIES optimized ${LIBLAS_RUNTIME_RELEASE})
  endif(LIBLAS_RUNTIME_RELEASE)
else(LIBLAS_RUNTIME_DEBUG)
  list(APPEND LIBLAS_RUNTIME_LIBRARIES ${LIBLAS_RUNTIME_RELEASE})
endif(LIBLAS_RUNTIME_DEBUG)

if(LIBLAS_RUNTIME_DEBUG)
  get_filename_component(RUNTIME_DIR "${LIBLAS_RUNTIME_DEBUG}" DIRECTORY)
  list(APPEND LIBLAS_RUNTIME_DIRS "${RUNTIME_DIR}")
endif(LIBLAS_RUNTIME_DEBUG)
if(LIBLAS_RUNTIME_RELEASE)
  get_filename_component(RUNTIME_DIR "${LIBLAS_RUNTIME_RELEASE}" DIRECTORY)
  list(APPEND LIBLAS_RUNTIME_DIRS "${RUNTIME_DIR}")
endif(LIBLAS_RUNTIME_RELEASE)
if (LIBLAS_RUNTIME_DIRS)
  list(REMOVE_DUPLICATES LIBLAS_RUNTIME_DIRS)
endif(LIBLAS_RUNTIME_DIRS)
set(LIBLAS_RUNTIME_DIRS "${LIBLAS_RUNTIME_DIRS}" CACHE PATH "LIBLAS runtime directories")


# handle the QUIETLY and REQUIRED arguments and set LIBLAS_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBLAS REQUIRED_VARS LIBLAS_LIBRARIES LIBLAS_INCLUDE_DIR)
if(LIBLAS_RUNTIME_LIBRARIES)
  set(LIBLAS_RUNTIME_LIBRARIES ${LIBLAS_RUNTIME_LIBRARIES} CACHE PATH "LIBBLAS runtime libraries")
endif(LIBLAS_RUNTIME_LIBRARIES)

mark_as_advanced(LIBLAS_INCLUDE_DIR LIBLAS_LIBRARIES LIBLAS_RUNTIME_LIBRARIES)
