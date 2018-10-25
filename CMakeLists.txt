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

# Build options
option(OHM_BUILD_DOXYGEN "Build doxygen documentation?" OFF)

option(OHM_EMBED_GPU_CODE "Embed GPU code into the executable (OpenCL)? Otherwise GPU code files are loaded from disk (better for development)." ON)

set(OHM_OPENCL_VER 2.0 CACHE STRING "Select the OpenCL runtime API to use.")
set_property(CACHE OHM_OPENCL_VER PROPERTY STRINGS 1.2 2.0 2.1 2.2)

# cl_khr_int64_base_atomics, cl_khr_int64_extended_atomics
set(OHM_OPENCL_STD max CACHE STRING "Select the OpenCL standard to compile GPU code against. [max] uses the reported device allowing for 2.x requires features.")
set_property(CACHE OHM_OPENCL_STD PROPERTY STRINGS 1.2 2.0 max)

set(OHM_OPENCL_2_FEATURES "cl_khr_int64_base_atomics;cl_khr_int64_extended_atomics" CACHE STRING "Features required for OpenCL 2.x. Fallback to 1.2 otherwise.")

option(OHM_PROFILE "Enable timing information messages for some queries." Off)
option(OHM_TES_DEBUG "Enable visual debuging code?" OFF)
option(OHM_THREADS "Enable CPU threading (using Thread Building Blocks)? Not advisable for onboard compilation." Off)
option(OHM_UNIT_TESTS "Build ohm units tests?" ON)
option(OHM_VALIDATION "Enable various validation tests in the occupancy map code. Has some performance impact." OFF)

set(OHM_GPU_API "OpenCL" CACHE STRING "Select the GPU acceleration API.")
set_property(CACHE OHM_GPU_API PROPERTY STRINGS CUDA OpenCL)

# Migrate to gputil library variable.
set(GPU_ACCEL ${OHM_GPU_API})

# Configure OHM_GPU as a value for configure_file
set(OHM_GPU_OPENCL 1)
set(OHM_GPU_CUDA 2)
set(OHM_GPU 0)

if(OHM_GPU_API STREQUAL "OpenCL")
  set(OHM_GPU ${OHM_GPU_OPENCL})
elseif(OHM_GPU_API STREQUAL "CUDA")
  include(cudautil)
  nvcc_setup(OHM)
  set(OHM_GPU ${OHM_GPU_CUDA})
endif()

# Configure use of 3rd-Eye-Scene visual debugging (use for development only)
if(OHM_TES_DEBUG)
  find_package(3es)
else(OHM_TES_DEBUG)
  # Include path to 3esservermarcos.h which effectively disables all 3es macros.
  set(3ES_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/3rdparty/3es-core")
endif(OHM_TES_DEBUG)

# libLAS uses static boost libraries.
if(MSVC)
  set(Boost_USE_STATIC_LIBS   ON)
  set(Boost_USE_MULTITHREADED ON)
endif(MSVC)

if(OHM_GPU_API STREQUAL "OpenCL")
  add_subdirectory(clu)
endif(OHM_GPU_API STREQUAL "OpenCL")
add_subdirectory(gputil)
add_subdirectory(ohmutil)
add_subdirectory(slamio)
add_subdirectory(ohm)
add_subdirectory(ohmtools)

add_subdirectory(utils)

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
configure_file(cmake/${CMAKE_PROJECT_NAME}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" COPYONLY)
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
