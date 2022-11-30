cmake_minimum_required(VERSION 3.10)
project(ohm)

# Set CMAKE_CXX_STANDARD to the minimum if not externally set - e.g., from command line.
# We'll set it in such a way that it remains it the Cache and stays set.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14 CACHE INTERNAL "C++ standard")
endif(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Ensure -fPIC is added.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Read version from package.xml
file(READ package.xml ohm_VERSION)
string(REGEX MATCH "\\<version\\>(.*)\\</version\\>" ohm_VERSION ${ohm_VERSION})
string(REGEX REPLACE "\\<version\\>(.*)\\</version\\>" "\\1" ohm_VERSION ${ohm_VERSION})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(utils)

# Work out if we are using VCPKG.
set(OHM_VCPKG FALSE)
if(CMAKE_TOOLCHAIN_FILE MATCHES ".*vcpkg\.cmake;?.*")
  set(OHM_VCPKG TRUE)
endif(CMAKE_TOOLCHAIN_FILE MATCHES ".*vcpkg\.cmake;?.*")

# Setup core ohm build options.
# Default to building shared libs.
set(OHM_BUILD_SHARED_DEFAULT OFF)
if(DEFINED BUILD_SHARED_LIBS)
  # But use BUILD_SHARED_LIBS as the default if it's specified (like from the command line).
  set(OHM_BUILD_SHARED_DEFAULT ${BUILD_SHARED_LIBS})
endif(DEFINED BUILD_SHARED_LIBS)
option(OHM_BUILD_SHARED "Build shared ohm libraries?" ${OHM_BUILD_SHARED_DEFAULT})
# For details on OHM_BUILD_SHARED_SUPPRESS_WARNINGS, see usage in compilerSetup.cmake
option(OHM_BUILD_SHARED_SUPPRESS_WARNINGS
  "Suppress warnings relating the DLL export symbols when building shared? Disables MSC warnings C4251 and C4275" ON)

option(OHM_PROFILE "Enable timing information messages for some queries." Off)
option(OHM_TES_DEBUG "Enable visual debuging code?" Off)
option(OHM_VALIDATION "Enable various validation tests in the occupancy map code. Has some performance impact." Off)
# OHM_FEATURE_CUDA is found in OhmCuda.cmake
option(OHM_LEAK_TRACK "Enable memory leak tracking?" OFF)

# Build options; what extensions are we going to build. These options are authoritative and flow down from here.
# For CUDA and OpenCL, we only need one, and we prefer CUDA based on what we find. However, vcpkg can guarantee we'll
# find one, so it because a dependent option.
option(OHM_BUILD_DOXYGEN "Build doxygen documentation?" Off)

ohm_feature(EIGEN "Use Eigen for more signficiant linear algebra algorithms (e.g., eigen decomposition)?" FIND Eigen3)
ohm_feature(HEIGHTMAP "Build heightmap library?")
ohm_feature(HEIGHTMAP_IMAGE "Build heightmap image conversion?" FIND OpenGL GLEW glfw3)
ohm_feature(PDAL "Build with PDAL reader support?" FIND pdal)
ohm_feature(THREADS "Enable CPU threading (using Thread Building Blocks)?" FIND tbb)
ohm_feature(TEST "Build unit tests?" FIND googletest)

# include CUDA config here to prime OHM_FEATURE_CUDA as we prefer CUDA over OpenCL.
# OHM_FEATURE_CUDA is in OhmCuda.cmake
include(OhmCuda)
set(NEED_OPENCL OFF)
if(NOT OHM_FEATURE_CUDA)
  set(NEED_OPENCL ON)
endif(NOT OHM_FEATURE_CUDA)
ohm_feature(OPENCL "Build with OpenCL support?" FIND OpenCL DEFAULT NEED_OPENCL)

# # Windows vcpkg does not provide CUDA.
# add_vcpkg_feature_if("cuda" "OHM_CUDA AND NOT WIN32")

# Migrate ohm specific setting for building shared libraries to the CMake global option.
set(BUILD_SHARED_LIBS ${OHM_BUILD_SHARED})
if(BUILD_SHARED_LIBS)
  message(STATUS "Building shared ohm libraries.")
endif(BUILD_SHARED_LIBS)

# Setup clang-tidy
include(ClangTidy)

# Allow the use of folders to group targets in supporting environments.
# For example Visual Studio solution folders.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_DEBUG_POSTFIX _d)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(CONF Debug;Release;MinSizeRel;RelWithDebInfo)
  string(TOUPPER ${CONF} CONFU)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/lib")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/bin")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFU} "${CMAKE_BINARY_DIR}/bin")
endforeach(CONF)

# Add memory leak tracking support.
include(LeakTrack)

# Configure installation prefixes.
# set(OHM_PREFIX_PACKAGE lib/cmake/${CMAKE_PROJECT_NAME})
set(OHM_PREFIX_PACKAGE share/${CMAKE_PROJECT_NAME}/cmake) # For catkin interoperation
# Prefix for include directories.
set(OHM_PREFIX_INCLUDE include/ohm)

# General, required library
find_package(glm CONFIG REQUIRED)

if(OHM_FEATURE_OPENCL)
  find_package(OpenCL)
endif(OHM_FEATURE_OPENCL)
if(OpenCL_FOUND)
  option(OHM_EMBED_GPU_CODE "Embed GPU code into the executable (OpenCL)? Otherwise GPU code files are loaded from disk (better for development)." ON)

  set(OHM_OPENCL_SDK_VER 1.2 CACHE STRING "Select the OpenCL runtime API to use.")
  set_property(CACHE OHM_OPENCL_SDK_VER PROPERTY STRINGS 1.2 2.0 2.1 2.2 3.0)

  set(OHM_OPENCL_STD 2.0 CACHE STRING "Select the default OpenCL standard to compile GPU code against. Also runtime configurable. [max] uses the reported device allowing for 2.x requires features.")
  set_property(CACHE OHM_OPENCL_STD PROPERTY STRINGS 1.2 2.0 3.0 max)
endif(OpenCL_FOUND)

# Identify any exteions required to compile with OpenCL 2.0 std.
set(OHM_OPENCL_2_FEATURES "" CACHE STRING "Features required for OpenCL 2.x. Fallback to 1.2 otherwise.")

# Look for CUDA to setup CUDA build options and work out which CMake CUDA paradigm we'll use.
if(OHM_FEATURE_CUDA)
  if(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
    find_package(CUDAToolkit REQUIRED)
  else(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
    # Need to find CUDA for deprecated project configuration
    find_package(CUDA REQUIRED)
  endif(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
endif(OHM_FEATURE_CUDA)

if(OHM_FEATURE_EIGEN)
  find_package(Eigen3 CONFIG REQUIRED)
endif(OHM_FEATURE_EIGEN)

# Manage compiler warnings.
# Use CMAKE_MODULE_PATH and include(warnings) if warnings.cmake is moved.
include(compilerSetup)

# Now setup CUDA after knowing if we are building shared libraries or not.
cuda_setup()

# Setup default memory leak tracking suppressions and options (GCC/Clang AddressSanitizer).
set(OHM_LEAK_SUPPRESS_OCL_INIT
  # OpenCL memory allocation overrides
  "__interceptor_malloc"
  "__interceptor_calloc"
  # Intel OpenCL leaks
  "libigdrcl"
  "llvm::FPPassManager::runOnFunction"
  "llvm::SmallVectorBase"
  "libigc"
  "basic_string<char, std::char_traits<char>, std::allocator<char> >::_M_mutate"
)

set(OHM_LEAK_SUPPRESS_TBB_INIT
  "libtbb"
)

set(OHM_LEAK_SUPPRESS_CUDA_INIT
)

set(OHM_ASAN_OPTIONS_CUDA_INIT "protect_shadow_gap=0")

set(OHM_LEAK_SUPPRESS_OCL "${OHM_LEAK_SUPPRESS_OCL_INIT}" CACHE STRING "Memory leak suppression list for OpenCL")
set(OHM_LEAK_SUPPRESS_TBB "${OHM_LEAK_SUPPRESS_TBB_INIT}" CACHE STRING "Memory leak suppression list for Intel Threading Building Blocks")
set(OHM_LEAK_SUPPRESS_CUDA "${OHM_LEAK_SUPPRESS_CUDA_INIT}" CACHE STRING "Memory leak suppression list for CUDA")
set(OHM_ASAN_OPTIONS_CUDA "${OHM_ASAN_OPTIONS_CUDA_INIT}" CACHE STRING "Address Sanitizer options for CUDA")

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

add_subdirectory(logutil)

if(OHM_FEATURE_OPENCL)
  add_subdirectory(clu)
endif(OHM_FEATURE_OPENCL)
if(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)
  add_subdirectory(gputil)
endif(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)

add_subdirectory(ohmutil)
add_subdirectory(slamio)
add_subdirectory(ohm)
if(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)
  add_subdirectory(ohmgpu)
endif(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)
if(OHM_FEATURE_HEIGHTMAP)
  add_subdirectory(ohmheightmap)
endif(OHM_FEATURE_HEIGHTMAP)

if(OHM_FEATURE_HEIGHTMAP_IMAGE)
  # Experimental
  add_subdirectory(ohmheightmapimage)
endif(OHM_FEATURE_HEIGHTMAP_IMAGE)

add_subdirectory(ohmtools)
add_subdirectory(ohmapp)
add_subdirectory(utils)

if(OHM_FEATURE_TEST)
  enable_testing()
  add_subdirectory(tests)
endif(OHM_FEATURE_TEST)

clang_tidy_global()

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
  "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-version.cmake"
  VERSION ${ohm_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Installation of the package configuration file.
configure_file(cmake/${CMAKE_PROJECT_NAME}-packages.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake")
configure_file(cmake/${CMAKE_PROJECT_NAME}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake" @ONLY)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-packages.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-version.cmake"
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
    clu/3rdparty
    gputil/cl
    gputil/cuda
    ohm/cl
    ohm/gpu
    ohm/private
    ohmgpu/private
    ohmutil/3rdparty
    slamio/miniply
    slamio/pdal
  )

  if(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)
    list(APPEND DOXYGEN_DIRS ohmgpu)
  endif(OHM_FEATURE_CUDA OR OHM_FEATURE_OPENCL)
  if(OHM_FEATURE_OPENCL)
    list(APPEND DOXYGEN_DIRS clu)
  endif(OHM_FEATURE_OPENCL)

  list(APPEND DOXYGEN_DIRS docs)

  if(DOXYGEN_FOUND)
    # Create a target to build the documentation.
    # Here we also setup various documentation variables passed through to the doxyfile configuration.
    # Each named argument below describes the Doxygen variable it sets.
    doxygen_create(
      # DOXYFILE cmake/doxyfile.in  # Doxyfile to configure.
      PROJECT ${CMAKE_PROJECT_NAME} # PROJECT_NAME
      VERSION ${ohm_VERSION}        # PROJECT_NUMBER
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
