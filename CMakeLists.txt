cmake_minimum_required(VERSION 3.5)
project(ohm)

set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Read version from package.xml
file(READ package.xml ohm_VERSION)
string(REGEX MATCH "\\<version\\>(.*)\\</version\\>" ohm_VERSION ${ohm_VERSION})
string(REGEX REPLACE "\\<version\\>(.*)\\</version\\>" "\\1" ohm_VERSION ${ohm_VERSION})

if(APPLE)
  add_definitions("-Wno-logical-op-parentheses")
endif(APPLE)

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

# set(ConfigPackageLocation lib/cmake/${CMAKE_PROJECT_NAME})
set(ConfigPackageLocation share/${CMAKE_PROJECT_NAME}/cmake) # For catkin interoperation

# For Visual Studio, force PDB based debug information even in release builds.
# This has a small impact on the size of the of the DLLs, but provides debug information for release mode crashes.
# The PDBs can be kept for debugging specific releases, but do not need to be shipped as part of the runtime, unless
# shipping an SDK. The last point is to address MSVC linked warnings which are impossible to suppress without providing
# PDB files.
if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi")
  set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "/debug ${CMAKE_MODULE_LINKER_FLAGS_RELEASE}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "/debug ${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/debug ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
endif(MSVC)

option(OHM_VALIDATION "Enable various validation tests in the occupancy map code. Has some performance impact." OFF)
option(OHM_THREADS "Enable CPU threading (using Thread Building Blocks)? Not advisable for onboard compilation." Off)
option(OHM_PROFILE "Enable timing information messages for some queries." Off)

option(OHM_TES_DEBUG "Enable visual debuging code?" OFF)
option(OHM_UNIT_TESTS "Build ohm units tests?" ON)

option(OHM_EMBED_GPU_CODE "Embed GPU code into the executable (OpenCL)? Otherwise GPU code files are loaded from disk (better for development)." ON)
set(OHM_OPENCL_STD 1.2 CACHE STRING "Select the OpenCL standard to compile GPU code against.")
set_property(CACHE OHM_OPENCL_STD PROPERTY STRINGS 1.2 2.0)

set(OHM_GPU_API "OpenCL" CACHE STRING "Select the GPU acceleration API.")
set_property(CACHE OHM_GPU_API PROPERTY STRINGS CUDA OpenCL)

# Migrate to gputil library variable.
set(GPU_ACCEL ${OHM_GPU_API})

if(OHM_GPU STREQUAL "CUDA")
  include(cudautil)
  nvcc_setup(OHM)
endif(OHM_GPU STREQUAL "CUDA")

if(OHM_TES_DEBUG)
  find_package(3es)
else(OHM_TES_DEBUG)
  add_library(3es::3es-core INTERFACE IMPORTED)
  set_target_properties(3es::3es-core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/3rdparty/3es-core")
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

# Setup import scripts.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
  VERSION ${ohm_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Installation of the package configuration file.
configure_file(cmake/${CMAKE_PROJECT_NAME}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" COPYONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config-version.cmake"
  DESTINATION ${ConfigPackageLocation}
  COMPONENT Devel)
