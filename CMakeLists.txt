cmake_minimum_required(VERSION 3.5)
project(ohm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Ensure -fPIC is added.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Read version from package.xml
file(READ package.xml ohm_VERSION)
string(REGEX MATCH "\\<version\\>(.*)\\</version\\>" ohm_VERSION ${ohm_VERSION})
string(REGEX REPLACE "\\<version\\>(.*)\\</version\\>" "\\1" ohm_VERSION ${ohm_VERSION})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Setup clang-tidy
set(CLANG_TIDY_PROJECT_PREFIX OHM)
include(clang-tidy)

# Allow the use of folders to group targets in supporting environments.
# For example Visual Studio solution folders.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(CONF Debug;Release;MinSizeRel;RelWithDebInfo)
  string(TOUPPER ${CONF} CONFU)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/lib")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/bin")
endforeach(CONF)

# Manage compiler warnings.
# Use CMAKE_MODULE_PATH and include(warnings) if warnings.cmake is moved.
include(compilerSetup)

# Configure installation prefixes.
# set(OHM_PREFIX_PACKAGE lib/cmake/${CMAKE_PROJECT_NAME})
set(OHM_PREFIX_PACKAGE share/${CMAKE_PROJECT_NAME}/cmake) # For catkin interoperation
# Prefix for include directories.
set(OHM_PREFIX_INCLUDE include/ohm)

# Look for OpenCL and CUDA to setup GPU build options
find_package(CUDA QUIET)
find_package(OpenCL QUIET)

set(OHM_BUILD_CUDA_DEFAULT NO)
set(OHM_BUILD_OPENCL_DEFAULT NO)

if(CUDA_FOUND)
  set(OHM_BUILD_CUDA_DEFAULT YES)
endif(CUDA_FOUND)
if(OPENCL_FOUND)
  set(OHM_BUILD_OPENCL_DEFAULT YES)
endif(OPENCL_FOUND)

# Build options
option(OHM_BUILD_DOXYGEN "Build doxygen documentation?" Off)

option(OHM_EMBED_GPU_CODE "Embed GPU code into the executable (OpenCL)? Otherwise GPU code files are loaded from disk (better for development)." ON)

set(OHM_OPENCL_VER 1.2 CACHE STRING "Select the OpenCL runtime API to use.")
set_property(CACHE OHM_OPENCL_VER PROPERTY STRINGS 1.2 2.0 2.1 2.2)

set(OHM_OPENCL_STD 1.2 CACHE STRING "Select the default OpenCL standard to compile GPU code against. Also runtime configurable. [max] uses the reported device allowing for 2.x requires features.")
set_property(CACHE OHM_OPENCL_STD PROPERTY STRINGS 1.2 2.0 max)

# Identify any exteions required to compile with OpenCL 2.0 std.
set(OHM_OPENCL_2_FEATURES "" CACHE STRING "Features required for OpenCL 2.x. Fallback to 1.2 otherwise.")

# Initialse OHM_THREADS based on whether we can find TBB or not.
if(NOT DEFINED OHM_THREADS)
  set(TBB_THREADS_DEFAULT Off)
  find_package(TBB QUIET)
  if(TBB_FOUND)
    set(TBB_THREADS_DEFAULT On)
  else(TBB_FOUND)
    message(STATUS "Intel Threading Building Blocks not available. Building without threads.")
  endif(TBB_FOUND)
else(NOT DEFINED OHM_THREADS)
  set(TBB_THREADS_DEFAULT ${OHM_THREADS})
endif(NOT DEFINED OHM_THREADS)

option(OHM_PROFILE "Enable timing information messages for some queries." Off)
option(OHM_TES_DEBUG "Enable visual debuging code?" Off)
option(OHM_THREADS "Enable CPU threading (using Thread Building Blocks)? Not advisable for onboard compilation." ${TBB_THREADS_DEFAULT})
option(OHM_UNIT_TESTS "Build ohm units tests?" On)
option(OHM_VALIDATION "Enable various validation tests in the occupancy map code. Has some performance impact." Off)
option(OHM_BUILD_CUDA "Build ohm library and utlities for CUDA?" ${OHM_BUILD_CUDA_DEFAULT})
option(OHM_BUILD_OPENCL "Build ohm library and utlities for OpenCL?" ${OHM_BUILD_OPENCL_DEFAULT})
option(OHM_BUILD_HEIGHTMAPUTIL "Build the heightmap to image conversion library and utility?" ON)

# Configure OHM_GPU as a value for configure_file
set(OHM_GPU_OPENCL 1)
set(OHM_GPU_CUDA 2)
set(OHM_GPU 0)

# Configure use of 3rd-Eye-Scene visual debugging (use for development only)
if(OHM_TES_DEBUG)
  find_package(3es)
else(OHM_TES_DEBUG)
  # Include path to 3esservermarcos.h which effectively disables all 3es macros.
  set(3ES_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/3rdparty/3es-core")
endif(OHM_TES_DEBUG)

# Resolve which point cloud library to use.
# TODO: Migrate to PDAL only once Windows developer setup instructions for PDAL are resolved.
find_package(PDAL QUIET)
find_package(LIBLAS QUIET)
find_package(LASZIP QUIET)

set(OHM_CLOUD_IO_LIBRARY auto CACHE STRING "Select point cloud IO library to use:  auto (prefer pdal), liblas, pdal or none")
set_property(CACHE OHM_CLOUD_IO_LIBRARY PROPERTY STRINGS auto liblas pdal none)

message(STATUS "Cloud IO Library: ${OHM_CLOUD_IO_LIBRARY}")

# Select the cloud io library to use.
set(CLOUD_IO_LIB ${OHM_CLOUD_IO_LIBRARY})
if(CLOUD_IO_LIB STREQUAL "auto")
  if(PDAL_FOUND)
    set(CLOUD_IO_LIB "pdal")
  elseif(LIBLAS_FOUND)
    set(CLOUD_IO_LIB "liblas")
    # libLAS uses static boost libraries.
    if(MSVC)
      set(Boost_USE_STATIC_LIBS   ON)
      set(Boost_USE_MULTITHREADED ON)
    endif(MSVC)
  else()
    message(SEND_ERROR "Unable to local a point cloud IO library [pdal, liblas].")
  endif()
  message(STATUS "Cloud IO auto selected: ${CLOUD_IO_LIB}")
endif(CLOUD_IO_LIB STREQUAL "auto")

if(OHM_BUILD_OPENCL)
  add_subdirectory(clu)
endif(OHM_BUILD_OPENCL)
add_subdirectory(gputil)

add_subdirectory(ohmutil)
add_subdirectory(slamio)
add_subdirectory(ohm)
add_subdirectory(ohmgpu)

if(OHM_BUILD_HEIGHTMAPUTIL)
  # Experimental
  add_subdirectory(ohmheightmaputil)
endif(OHM_BUILD_HEIGHTMAPUTIL)

add_subdirectory(ohmtools)
# add_subdirectory(utils)

if(OHM_UNIT_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif(OHM_UNIT_TESTS)

export(EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-targets.cmake"
  NAMESPACE ohm::
)

install(EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  FILE ${CMAKE_PROJECT_NAME}-config-targets.cmake
  NAMESPACE ohm::
  DESTINATION ${OHM_PREFIX_PACKAGE}
)

# Setup import scripts.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
  VERSION ${ohm_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Installation of the package configuration file.
configure_file(cmake/${CMAKE_PROJECT_NAME}-packages.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake")
configure_file(cmake/${CMAKE_PROJECT_NAME}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" @ONLY)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
  DESTINATION ${OHM_PREFIX_PACKAGE}
  COMPONENT Devel)

# Install MSVC runtime libraries. This will also affect the CPack installation.
include(InstallRequiredSystemLibraries)

# Doxygen setup.
if(OHM_BUILD_DOXYGEN)
  # Include Doxygen helper functions. This also finds the Doxygen package.
  include(doxygen)

  set(DOXYGEN_DIRS
    gputil
    ohm
    ohmtools
    ohmutil
    slamio
  )

  set(DOXYGE_EXCLUDE_DIRS
    gputil/cl
    gputil/cuda
    ohm/cl
    ohm/gpu
    ohm/private
  )

  if(OHM_GPU EQUAL OHM_GPU_OPENCL)
    list(APPEND DOXYGEN_DIRS clu)
  endif(OHM_GPU EQUAL OHM_GPU_OPENCL)

  if(DOXYGEN_FOUND)
    # Create a target to build the documentation.
    # Here we also setup various documentation variables passed through to the doxyfile configuration.
    # Each named argument below describes the Doxygen variable it sets.
    doxygen_create(
      # DOXYFILE cmake/doxyfile.in  # Doxyfile to configure.
      PROJECT ${CMAKE_PROJECT_NAME} # PROJECT_NAME
      VERSION ${ohm_VERSION}   # PROJECT_NUMBER
      OUTPUT_DIR html               # HTML_OUPTUT
      # CSS <style>.css             # HTML_STYLESHEET
      PUBLISHER "CSIRO"             # DOCSET_PUBLISHER_NAME
      PUBLISHER_ID au.csiro         # DOCSET_PUBLISHER_ID
      PROJECT_ID au.csiro.ohm       # DOCSET_BUNDLE_ID, QHP_NAMESPACE, ECLIPSE_DOC_ID
      PATHS                         # INPUT (RECURSIVE is on)
        ${DOXYGEN_DIRS}
      EXCLUDE_PATHS                 # EXCLUDE
        ${DOXYGE_EXCLUDE_DIRS}
      # Where to find source code examples.
      # EXAMPLE_PATHS <paths>        # EXAMPLE_PATH
      # Where to find images.
      # IMAGE_PATHS <paths>          # IMAGE_PATH
    )

    # Setup installation of the generated documentation: source, destination.
    doxygen_install(${CMAKE_CURRENT_BINARY_DIR}/html ohm)
  endif(DOXYGEN_FOUND)
endif(OHM_BUILD_DOXYGEN)
