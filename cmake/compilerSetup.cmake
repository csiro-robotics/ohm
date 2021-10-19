# Copyright (c) 2017
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Configure compiler warnings depending on the current compiler.

# Configure warnings for gcc.
macro(warnings_gcc)
  add_compile_options(
    # "-pedantic"
    "-Wall"
    "-Wextra"
    "-Werror=return-type"
    # "-Wconversion"
    # "-Werror=pedantic"
    "-Werror=vla"
    "-Wno-unused-function"
    "-Wno-missing-braces"
    "-Wno-unknown-pragmas"
    "-Wno-parentheses"
    "-Wno-ignored-attributes"
  )
endmacro(warnings_gcc)

# Configure for apple clang.
macro(setup_apple_clang)
  warnings_gcc()
  # Disable precedence warning of && and || in if statements:
  #   && === *
  #   || === +
  add_compile_options(
    "-Wno-logical-op-parentheses"
  )
endmacro(setup_apple_clang)

# Configure for GCC
macro(setup_gcc)
  warnings_gcc()
endmacro(setup_gcc)

# Configure for MSVC
macro(setup_msvc)
  # For Visual Studio, force PDB based debug information even in release builds.
  # This has a small impact on the size of the of the DLLs, but provides debug information for release mode crashes.
  # The PDBs can be kept for debugging specific releases, but do not need to be shipped as part of the runtime, unless
  # shipping an SDK. The last point is to address MSVC linked warnings which are impossible to suppress without providing
  # PDB files.
  # We also add /FS (Force Synchronous PDB Writes) to support CUDA compilation which will try to use the same PDB file
  # for various .cu files (from the same project).
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /FS")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FS")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /FS")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /FS")
  set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "/debug ${CMAKE_MODULE_LINKER_FLAGS_RELEASE}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "/debug ${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/debug ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")

  # Level 3 warnings by default is fine. May go to level 4.
  # Enable multi-processor project compilation.
  option(COMPILER_MULTI_PROCESSOR "Use multiple processor compilation (MSVC)?" OFF)
  if(COMPILER_MULTI_PROCESSOR)
    add_compile_options("/MP")
  endif(COMPILER_MULTI_PROCESSOR)
  if(OHM_BUILD_SHARED_SUPPRESS_WARNINGS)
    # Disable the following MSC compiler warnings.
    # 4251 Warning that data members of a DLL export definition do not themselves have DLL export directives.
    # 4275 Warning that a base class of a DLL export definition do not themselves have DLL export directives.
    # These are valid and important warning which indicate potential ABI incompatibilities. However, ohm is not
    # engineered to be 100% robust to ABI changes and these warnings generate many difficult to avoid errors.
    #
    # For example, using std::vector as a member of a class which is exported from a DLL raises warning C4251. The issue
    # is that this is vulnerable to ABI incompatibilies if a different C++ runtime library is used - such as building
    # ohm and building client code using difference Visual Studio versions.
    #
    # Similarly, C4275 is raised by actions such as deriving std::runtime_error.
    #
    # Both these cases are essentially valid code, but the warnings are also correct to point out the ABI vulnerability.
    # The side effect is that upgrading version will often require a rebuild of client code against the ohm libraries
    # rather than a drop in replacement. Linux and MacOS are slightly innocculated against this because the C++ runtime
    # libraries tend to stay the same for an OS version. The warnings would be much more relevant when writing things
    # like COM interfaces.
    add_compile_options("/wd4251" "/wd4275")
  endif(OHM_BUILD_SHARED_SUPPRESS_WARNINGS)
endmacro(setup_msvc)

# Select the correct warnings configuration.
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
  # using Apple Clang (MacOS)
  setup_apple_clang()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using GCC
  setup_gcc()
# elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
#   # using Intel C++
elseif (MSVC)
  # using Visual Studio C++
  setup_msvc()
else()
  message("Unknown compiler ${CMAKE_CXX_COMPILER_ID}")
endif()
