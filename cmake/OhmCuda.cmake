
# OHM_BUILD_CUDA_DEFAULT is used to initialise the OHM_BUILD_CUDA option. We have to work out if CUDA is present first.
# How we do so depends on the CMake version.
set(OHM_BUILD_CUDA_DEFAULT OFF) # Initialisation value for OHM_BUILD_CUDA

# CMake 3.10 introduces a native way of configuring CUDA. However, this lacked certain features such as CUDA
# architecture selection, which made it useful only for local builds (not for distribution). 3.18 adds CUDA architecture
# selection as well has having other CUDA features stabilised, making 3.18 the minimum preferred version.
# We prefer the deprecated CUDA setup for versions before 3.18.
set(OHM_USE_DEPRECATED_CMAKE_CUDA_DEFAULT ON) # Initialisation value for OHM_USE_DEPRECATED_CMAKE_CUDA
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
  set(OHM_USE_DEPRECATED_CMAKE_CUDA_DEFAULT OFF)
endif(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)

# Configure an option for forcibly using the deprecated CUDA project configuration.
option(OHM_USE_DEPRECATED_CMAKE_CUDA
  "Use deprecated CUDA project setup? Disabling requires CMake 3.10+ but the CUDA architecture cannot be set until 3.18."
  ${OHM_USE_DEPRECATED_CMAKE_CUDA_DEFAULT})

# Find if CUDA is available.
if(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
  # Newer way of finding CUDA.
  include(CheckLanguage)
  # Check if CUDA is available.
  check_language(CUDA)
  set(OHM_CUDA_VERSION "${CMAKE_CUDA_COMPILER_VERSION}")
  # Note: CMake 3.11 enabled CMAKE_CUDA_COMPILER on check_language(CUDA). Otherwise it's not propertly set.
  # Another argument for why we perfer a later version for new CUDA project configuration.
  if(CMAKE_VERSION VERSION_LESS 3.11)
    message("WARNING: Cannot automatically check for CUDA using CMake 3.10 unless OHM_USE_DEPRECATED_CMAKE_CUDA is ON.")
  endif(CMAKE_VERSION VERSION_LESS 3.11)
  if(CMAKE_CUDA_COMPILER)
    # CUDA compiler is available. Set OHM_BUILD_CUDA_DEFAULT so that we initialse OHM_BUILD_CUDA to ON by default.
    set(OHM_BUILD_CUDA_DEFAULT ON)
  endif(CMAKE_CUDA_COMPILER)
else(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
  find_package(CUDA)
  if(CUDA_FOUND)
    set(OHM_CUDA_VERSION "${CUDA_VERSION}")
    set(OHM_BUILD_CUDA_DEFAULT ON)
  endif(CUDA_FOUND)
endif(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)

# Set the option for building ohm with CUDA based on whether or not we found it.
option(OHM_BUILD_CUDA "Build ohm library and utlities for CUDA?" ${OHM_BUILD_CUDA_DEFAULT})

# Additional options
if(OHM_BUILD_CUDA)
  set(OHM_CUDA_DEBUG "none" CACHE STRING
    "Controls the CUDA debug level, regardless of the current build type. lineinfo adds debug symbols, full also disabled optimisation."
  )
  set_property(CACHE OHM_CUDA_DEBUG PROPERTY STRINGS full lineinfo none)
endif(OHM_BUILD_CUDA)

#==============================================================================
# Configure various CUDA building options.
#==============================================================================
macro(cuda_setup)
  if(OHM_BUILD_CUDA)
    # Architecture reference:
    # https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
    #
    # Require version 9 implied minimum.
    set(CUDA_ARCH_INIT "50;52;53;60;61;62;70;72")
    if(CUDA_VERSION VERSION_GREATER 11)
      # Last version to support 5.x
      set(CUDA_ARCH_INIT "50;52;53;60;61;62;70;72;75;80;86")
    elseif(CUDA_VERSION VERSION_GREATER 10)
      # Adds 7.0, drops 3.0, 3.5, 3.7
      set(CUDA_ARCH_INIT "50;52;53;60;61;62;70;72;75")
    endif()
    # Set the default architectures to all those vailable for the current CUDA verison.
    set(OHM_CUDA_ARCHITECTURES "${CUDA_ARCH_INIT}" CACHE STRING "Specifies the list of CUDA compute architectures to compile for.")

    if(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
      _cuda_setup_build_options()
    else(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
      _cuda_setup_deprecated_build_options()
    endif(NOT OHM_USE_DEPRECATED_CMAKE_CUDA)
  endif(OHM_BUILD_CUDA)
endmacro(cuda_setup)

#==============================================================================
#==============================================================================
macro(_cuda_setup_build_options)
  enable_language(CUDA)
  # Setup architectures
  set(CMAKE_CUDA_ARCHITECTURES "${OHM_CUDA_ARCHITECTURES}")
  # Match CUDA standard to the C++ standard for the project.
  set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
  set(CMAKE_CUDA_STANDARD_REQUIRED ${CMAKE_CXX_STANDARD_REQUIRED})
  # Note; the CMake setup (3.18) adds the compiler flag '-std=c++##'. This is correct for NVCC, GCC and Clang, but is
  # not correct for MSVC where it should be '/std:c++##' (note the use of ':' rather than '='). This generates a build
  # warning that the host compiler will ignore '-std=c++##'. This is harmless, but would be nice to avoid.
  # However, we should add the right flag for MSVC, so we do it below.
  if(MSVC)
    add_compile_options("-std:c++${CMAKE_CXX_STANDARD}")
  endif( MSVC)
  if(OHM_CUDA_DEBUG STREQUAL "full")
    add_compile_options(
      "$<$<COMPILE_LANGUAGE:CUDA>:-g>"
      "$<$<COMPILE_LANGUAGE:CUDA>:-G>"
    )
  elseif(OHM_CUDA_DEBUG STREQUAL "lineinfo")
    add_compile_options(
      "$<$<COMPILE_LANGUAGE:CUDA>:-g>"
      "$<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>"
    )
  endif(OHM_CUDA_DEBUG STREQUAL "full")
  if(BUILD_SHARED_LIBS)
    set(OHM_CUDA_LIBRARY CUDA::cudart)
  else(BUILD_SHARED_LIBS)
    set(OHM_CUDA_LIBRARY CUDA::cudart_static)
  endif(BUILD_SHARED_LIBS)
endmacro(_cuda_setup_build_options)

#==============================================================================
# Setup CUDA build options for the older, deprecated project configuration.
#==============================================================================
macro(_cuda_setup_deprecated_build_options)
  # Setup CUDA_USE_STATIC_CUDA_RUNTIME, forcing it on WIN32 if using BUILD_SHARED_LIBS
  # See the warning message below.
  if(BUILD_SHARED_LIBS)
    if(CUDA_USE_STATIC_CUDA_RUNTIME)
      message("Warning: BUILD_SHARED_LIBS and CUDA_USE_STATIC_CUDA_RUNTIME are both set. This can result in runtime "
              "failures when invoking CUDA kernels. Forcing CUDA_USE_STATIC_CUDA_RUNTIME to OFF.\n"
              "Use cmake-gui, ccmake or manually edit the CMakeCache.txt file for this project to update these values.")
    endif(CUDA_USE_STATIC_CUDA_RUNTIME)
    set(CUDA_USE_STATIC_CUDA_RUNTIME Off)
  else(BUILD_SHARED_LIBS)
    set(CUDA_USE_STATIC_CUDA_RUNTIME On)
  endif(BUILD_SHARED_LIBS)
  set(CUDA_LINK_LIBRARIES_KEYWORD PRIVATE)

    # Configure CUDA NVCC falgs for each selected CUDA architecture.
  if (OHM_CUDA_ARCHITECTURES)
    foreach(ARCH ${OHM_CUDA_ARCHITECTURES})
      _nvcc_flags_append(CUDA_NVCC_FLAGS "-gencode arch=compute_${ARCH},code=sm_${ARCH}")
    endforeach(ARCH)
  endif(OHM_CUDA_ARCHITECTURES)

  # Add debugging flags.
  if(OHM_CUDA_DEBUG STREQUAL "full")
    _nvcc_flags_append(CUDA_NVCC_FLAGS "-g -G")
  elseif(OHM_CUDA_DEBUG STREQUAL "lineinfo")
    _nvcc_flags_append(CUDA_NVCC_FLAGS "-g -lineinfo")
  endif(OHM_CUDA_DEBUG STREQUAL "full")

  # CUDA compiler doesn't seem to respect the CMAKE_POSITION_INDEPENDENT_CODE variable. Explicitly add it for
  # GCC comilation. May be needed for Clang as well (untested).
  if(CMAKE_POSITION_INDEPENDENT_CODE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")# OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      _nvcc_flags_append(CUDA_NVCC_FLAGS "-Xcompiler -fPIC")
    endif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")# OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  endif(CMAKE_POSITION_INDEPENDENT_CODE)

  # Migrate the flags to the calling scope.
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}")
endmacro(_cuda_setup_deprecated_build_options)

#==============================================================================
# This function deals with a peculiar issue with the generation of the
# generated CUDA compilation script.
#
# The CUDA compilation script can end up with bad values in nvcc_flags
# as extracted from the directory property COMPILE_DEFINITIONS. In particular,
# VTK leaves definitions in two poor forms:
# - vtkRenderingContext2D_AUTOINIT=1(vtkRenderingContextOpenGL)
# - vtkRenderingCore_INCLUDE="this/is/a/path"
#
# The first has an issue with brackets, the second the issue is the quotes not
# being escaped. Likely, neither would be an issue if the nvcc_flags value was
# properly quoted when generated.
#==============================================================================
function(_nvcc_fix_definitions)
  get_directory_property(CUDA_NVCC_DEFINITIONS COMPILE_DEFINITIONS)
  # VTK compile definitions interfere with the CUDA definitions. For now take the easy options and remove problem options.
  set(CUDA_NVCC_DEFINITIONS_FIXED)
  if(CUDA_NVCC_DEFINITIONS)
    foreach(_definition ${CUDA_NVCC_DEFINITIONS})
      set(USEDEF "${_definition}")
      if(_definition MATCHES ".*=.*\\(.*\\)")
        # Handle VKT options in the form: vtkRenderingContext2D_AUTOINIT=1(vtkRenderingContextOpenGL)
        string(REGEX REPLACE "\\(.*\\)" "" _definitionFixed "${_definition}")
        #message("Problem: ${_definition} -> ${_definitionFixed}")
        set(_definition "${_definitionFixed}")
      elseif (_definition MATCHES ".*_INCLUDE=\\\".*\\\"")
        # vtkRenderingCore_INCLUDE="path\path"
        # The issue is the quotes not getting escaped later.
        string(REGEX REPLACE "\\\"" "\\\\\"" _definitionFixed "${_definition}")
        #message("Problem: ${_definition} -> ${_definitionFixed}")
        set(_definition "${_definitionFixed}")
      endif()
      #message("${_definition}")
      _nvcc_flags_append(CUDA_NVCC_DEFINITIONS_FIXED "${_definition}")
    endforeach()
    #message("CUDA_NVCC_DEFINITIONS_FIXED: ${CUDA_NVCC_DEFINITIONS_FIXED}")
    set_property(DIRECTORY . PROPERTY COMPILE_DEFINITIONS "${CUDA_NVCC_DEFINITIONS_FIXED}")
    get_directory_property(CUDA_NVCC_DEFINITIONS COMPILE_DEFINITIONS)
    #message("CUDA_NVCC_DEFINITIONS: ${CUDA_NVCC_DEFINITIONS}")
  endif()
endfunction(_nvcc_fix_definitions)


#==============================================================================
# Extend a compiler flags variable FLAGS_VAR with additional FLAGS string.
#==============================================================================
macro(_nvcc_flags_append FLAGS_VAR FLAGS)
  if(${FLAGS_VAR})
    set(${FLAGS_VAR} "${${FLAGS_VAR}} ${FLAGS}")
  else(FLAGS_VAR)
    # Nothing set. Add new options.
    set(${FLAGS_VAR} "${FLAGS}")
  endif(${FLAGS_VAR})
endmacro(_nvcc_flags_append)
