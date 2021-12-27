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

# Setup clang-tidy
include(ClangTidy)

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

# Add memory leak tracking support.
include(LeakTrack)

# Configure installation prefixes.
# set(OHM_PREFIX_PACKAGE lib/cmake/${CMAKE_PROJECT_NAME})
set(OHM_PREFIX_PACKAGE share/${CMAKE_PROJECT_NAME}/cmake) # For catkin interoperation
# Prefix for include directories.
set(OHM_PREFIX_INCLUDE include/ohm)

# General, required library
find_package(GLM REQUIRED)

# Look for OpenCL to setup OpenCL build options.
find_package(OpenCL QUIET)
# Look for CUDA to setup CUDA build options and work out which CMake CUDA paradigm we'll use.
include(OhmCuda)
if(OHM_BUILD_CUDA)
  if(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
    find_package(CUDAToolkit REQUIRED)
  else(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
    # Need to find CUDA for deprecated project configuration
    find_package(CUDA REQUIRED)
  endif(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
endif(OHM_BUILD_CUDA)

set(OHM_BUILD_OPENCL_DEFAULT NO)

if(OPENCL_FOUND)
  set(OHM_BUILD_OPENCL_DEFAULT YES)
endif(OPENCL_FOUND)

# Build options
option(OHM_BUILD_DOXYGEN "Build doxygen documentation?" Off)

option(OHM_EMBED_GPU_CODE "Embed GPU code into the executable (OpenCL)? Otherwise GPU code files are loaded from disk (better for development)." ON)

set(OHM_OPENCL_SDK_VER 1.2 CACHE STRING "Select the OpenCL runtime API to use.")
set_property(CACHE OHM_OPENCL_SDK_VER PROPERTY STRINGS 1.2 2.0 2.1 2.2)

set(OHM_OPENCL_STD 2.0 CACHE STRING "Select the default OpenCL standard to compile GPU code against. Also runtime configurable. [max] uses the reported device allowing for 2.x requires features.")
set_property(CACHE OHM_OPENCL_STD PROPERTY STRINGS 1.2 2.0 max)

# Identify any exteions required to compile with OpenCL 2.0 std.
set(OHM_OPENCL_2_FEATURES "" CACHE STRING "Features required for OpenCL 2.x. Fallback to 1.2 otherwise.")

# Helper macro to Find TBB prefering TBBConfig, falling back to FindTBB. TBB_LIBRARIES will be defined in both cases.
# TBB_INCLUDE_DIRS will be empty for TBBConfig, and populated for FindTBB.
macro(find_tbb)
  # Try find TBB config file.
  find_package(TBB QUIET CONFIG)
  if(TBB_FOUND)
    set(OHM_TBB_CONFIG TRUE CACHE INTERNAL "Using TBB via config.cmake?" FORCE)
    message(STATUS "TBB found using TBBConfig.cmake")
    if(NOT TBB_LIBRARIES)
      if(TBB_IMPORTED_TARGETS)
        set(TBB_LIBRARIES "${TBB_IMPORTED_TARGETS}")
      elseif(TARGET tbb::tbb_static)
        set(TBB_LIBRARIES tbb::tbb_static tbb::tbbmalloc_static)
      elseif(TARGET tbb::tbb)
        set(TBB_LIBRARIES tbb::tbb tbb::tbbmalloc)
      else()
        message(FATAL_ERROR "Could not resolve tbb libraries")
      endif()
    endif(NOT TBB_LIBRARIES)
  else(TBB_FOUND)
    set(OHM_TBB_CONFIG FALSE CACHE INTERNAL "Using TBB via config.cmake?" FORCE)
    # No TBB config file. Try FindTBB.cmake
    find_package(TBB QUIET)
    if(TBB_FOUND)
      message(STATUS "TBB found using FindTBB.cmake")
    endif(TBB_FOUND)
  endif(TBB_FOUND)
endmacro(find_tbb)

# Initialse OHM_THREADS based on whether we can find TBB or not.
if(NOT DEFINED OHM_THREADS)
  set(TBB_THREADS_DEFAULT Off)
  find_tbb()
  if(TBB_FOUND)
    set(TBB_THREADS_DEFAULT On)
  else(TBB_FOUND)
    message(STATUS "Intel Threading Building Blocks not available. Building without threads.")
  endif(TBB_FOUND)
else(NOT DEFINED OHM_THREADS)
  set(TBB_THREADS_DEFAULT ${OHM_THREADS})
endif(NOT DEFINED OHM_THREADS)

# Default to building shared libs.
set(OHM_BUILD_SHARED_DEFAULT ON)
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
option(OHM_THREADS "Enable CPU threading (using Thread Building Blocks)? Not advisable for onboard compilation." ${TBB_THREADS_DEFAULT})
option(OHM_UNIT_TESTS "Build ohm units tests?" On)
option(OHM_VALIDATION "Enable various validation tests in the occupancy map code. Has some performance impact." Off)
# OHM_BUILD_CUDA is found in OhmCuda.cmake
option(OHM_BUILD_OPENCL "Build ohm library and utlities for OpenCL?" ${OHM_BUILD_OPENCL_DEFAULT})
option(OHM_LEAK_TRACK "Enable memory leak tracking?" OFF)

# Manage compiler warnings.
# Use CMAKE_MODULE_PATH and include(warnings) if warnings.cmake is moved.
include(compilerSetup)

# Migrate ohm specific setting for building shared libraries to the CMake global option.
set(BUILD_SHARED_LIBS ${OHM_BUILD_SHARED})
if(BUILD_SHARED_LIBS)
  message(STATUS "Building shared ohm libraries.")
endif(BUILD_SHARED_LIBS)

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

# Resolve which point cloud library to use.
# TODO: Migrate to PDAL only once Windows developer setup instructions for PDAL are resolved.
find_package(PDAL QUIET)
find_package(LIBLAS QUIET)
find_package(LASZIP QUIET)

set(WITH_PDAL_INIT OFF)
if(PDAL_FOUND)
  if(PDAL_VERSION VERSION_GREATER_EQUAL 1.7)
    set(WITH_PDAL_INIT ON)
  endif(PDAL_VERSION VERSION_GREATER_EQUAL 1.7)
endif(PDAL_FOUND)
option(WITH_PDAL "Build with PDAL support?" ${WITH_PDAL_INIT})

find_package(Eigen3 QUIET)
option(WITH_EIGEN "Use Eigen for more signficiant linear algebra algorithms (e.g., eigen decomposition)?" ${EIGEN3_FOUND})
# Rename WITH_EIGEN for use in OhmConfig.h
# We follow CMake paradigms here using WITH_XXX, but in code we prefix with OHM to avoid aliasing with other libraries.
set(OHM_WITH_EIGEN ${WITH_EIGEN})

# Heightmap image libraries (optional)
find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(glfw3 QUIET)
set(OHM_HM_IMG_LIBS_FOUND NO)
if(OpenGL_FOUND AND GLEW_FOUND AND GLM_FOUND AND glfw3_FOUND)
  set(OHM_HM_IMG_LIBS_FOUND YES)
endif(OpenGL_FOUND AND GLEW_FOUND AND GLM_FOUND AND glfw3_FOUND)
option(OHM_BUILD_HEIGHTMAP_IMAGE "Build ohmheightmapimage library to support heightmap to image conversion?" ${OHM_HM_IMG_LIBS_FOUND})

if(OHM_BUILD_OPENCL)
  add_subdirectory(clu)
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)
  add_subdirectory(gputil)
endif(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)

add_subdirectory(ohmutil)
add_subdirectory(slamio)
add_subdirectory(ohm)
if(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)
  add_subdirectory(ohmgpu)
endif(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)
add_subdirectory(ohmheightmap)

if(OHM_BUILD_HEIGHTMAP_IMAGE)
  # Experimental
  add_subdirectory(ohmheightmapimage)
endif(OHM_BUILD_HEIGHTMAP_IMAGE)

add_subdirectory(ohmtools)
add_subdirectory(ohmapp)
add_subdirectory(utils)

if(OHM_UNIT_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif(OHM_UNIT_TESTS)

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

  if(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)
    list(APPEND DOXYGEN_DIRS ohmgpu)
  endif(OHM_BUILD_CUDA OR OHM_BUILD_OPENCL)
  if(OHM_BUILD_OPENCL)
      list(APPEND DOXYGEN_DIRS clu)
  endif(OHM_BUILD_OPENCL)

   list(APPEND DOXYGEN_DIRS docs)

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
