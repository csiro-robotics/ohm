option(OHM_SYSTEM_GTEST "Have ohm use a system available version of Googletest via find_package()?" ${OHM_VCPKG})

if(OHM_SYSTEM_GTEST)
  # Enable VCPKG tests feature. This will only have an effect when using vcpkg.
  find_package(GTest REQUIRED)
  get_target_property(GTEST_INCLUDE_DIRS GTest::gtest INTERFACE_INCLUDE_DIRECTORIES)
  set(GTEST_LIBRARIES GTest::gtest)
  set(GTEST_MAIN_LIBRARIES GTest::gtest_main)
  return()
endif(OHM_SYSTEM_GTEST)

include(ExternalProject)

find_package(Threads)

# State setup
get_property(_GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

set(_GTEST_CONFIG_ARGS "-Dgtest_force_shared_crt=On")

if(NOT _GENERATOR_IS_MULTI_CONFIG)
  list(APPEND _GTEST_CONFIG_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
endif(NOT _GENERATOR_IS_MULTI_CONFIG)

# Setup download and build of Googletest
ExternalProject_Add(
    googletest
    URL https://github.com/google/googletest/archive/release-1.10.0.zip
    PREFIX ${CMAKE_BINARY_DIR}/3rd-party/googletest
    URL_MD5 82358affdd7ab94854c8ee73a180fc53
    INSTALL_COMMAND ""
    CMAKE_ARGS ${_GTEST_CONFIG_ARGS}
)

# Resolve directories used by gtest
ExternalProject_Get_Property(googletest source_dir)
ExternalProject_Get_Property(googletest binary_dir)

# Helper function for resolving a gtest library name of a build config
function(_gtest_library_path OUTVAR LIBNAME BUILD_CONFIG binary_dir)
  # Handle post fix; gtest is configured to add postifx the debug library names with 'd'
  unset(_POSTFIX)
  if(BUILD_CONFIG STREQUAL "Debug")
    set(_POSTFIX "_d")
  endif(BUILD_CONFIG STREQUAL "Debug")
  set(_PATH_DIR "${binary_dir}/lib")
  # Handle path prefix
  if(_GENERATOR_IS_MULTI_CONFIG)
    set(_PATH_DIR "${_PATH_DIR}/${BUILD_CONFIG}")
  endif(_GENERATOR_IS_MULTI_CONFIG)
  set(${OUTVAR}
    ${_PATH_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}${LIBNAME}${_POSTFIX}${CMAKE_STATIC_LIBRARY_SUFFIX}
    PARENT_SCOPE)
endfunction(_gtest_library_path)

# Resolve build configuration(s)
set(_BUILD_CONFIGS)
if(_GENERATOR_IS_MULTI_CONFIG)
  set(_BUILD_CONFIGS ${CMAKE_CONFIGURATION_TYPES})
else(_GENERATOR_IS_MULTI_CONFIG)
  if(CMAKE_BUILD_TYPE)
    set(_BUILD_CONFIGS ${CMAKE_BUILD_TYPE})
  else(CMAKE_BUILD_TYPE)
    set(_BUILD_CONFIGS Release)
  endif(CMAKE_BUILD_TYPE)
endif(_GENERATOR_IS_MULTI_CONFIG)

# Setup import targets
add_library(gtest UNKNOWN IMPORTED)
add_dependencies(gtest googletest)

add_library(gtest_main UNKNOWN IMPORTED)
add_dependencies(gtest_main gtest googletest)

# Configure include directory now. Used in loop below.
set(GTEST_INCLUDE_DIR "${source_dir}/googletest/include")

# Resolve library paths.
foreach(_CONFIG ${_BUILD_CONFIGS})
  string(TOUPPER ${_CONFIG} _CONFIG_UPPER)
  _gtest_library_path(GTEST_LIBRARY_PATH gtest "${_CONFIG}" "${binary_dir}")
  _gtest_library_path(GTEST_MAIN_LIBRARY_PATH gtest_main "${_CONFIG}" "${binary_dir}")
  if(_GENERATOR_IS_MULTI_CONFIG)
    set_property(TARGET gtest PROPERTY IMPORTED_LOCATION_${_CONFIG_UPPER} "${GTEST_LIBRARY_PATH}")
    set_property(TARGET gtest_main PROPERTY IMPORTED_LOCATION_${_CONFIG_UPPER} "${GTEST_MAIN_LIBRARY_PATH}")
  else(_GENERATOR_IS_MULTI_CONFIG)
    set_property(TARGET gtest PROPERTY IMPORTED_LOCATION "${GTEST_LIBRARY_PATH}")
    set_property(TARGET gtest_main PROPERTY IMPORTED_LOCATION "${GTEST_MAIN_LIBRARY_PATH}")
  endif(_GENERATOR_IS_MULTI_CONFIG)
  set_property(TARGET gtest PROPERTY INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIR}")
  set_property(TARGET gtest_main PROPERTY INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIR}")

  # Make Ninja builds work
  # Ninja has a different way of scanning to Make and needs the gtest library artefact files to exist in order to start
  # the build at all. We "touch" those files here.
  if(CMAKE_GENERATOR MATCHES "^Ninja")
    if(NOT EXISTS ${GTEST_LIBRARY_PATH})
    file(WRITE ${GTEST_LIBRARY_PATH} "")
      endif(NOT EXISTS ${GTEST_LIBRARY_PATH})
    if(NOT EXISTS ${GTEST_MAIN_LIBRARY_PATH})
      file(WRITE ${GTEST_MAIN_LIBRARY_PATH} "")
    endif(NOT EXISTS ${GTEST_MAIN_LIBRARY_PATH})
  endif(CMAKE_GENERATOR MATCHES "^Ninja")
endforeach(_CONFIG)

# Setup GTest variables, mimicing FindGTest.cmake
set(GTEST_INCLUDE_DIRS "${GTEST_INCLUDE_DIR}")
set(GTEST_LIBRARIES gtest Threads::Threads)
set(GTEST_MAIN_LIBRARIES gtest_main)
