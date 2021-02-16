
include(ExternalProject)

# Setup download and build of Googletest
ExternalProject_Add(
    googletest
    URL https://github.com/google/googletest/archive/release-1.10.0.zip
    PREFIX ${CMAKE_BINARY_DIR}/3rd-party/googletest
    URL_MD5 82358affdd7ab94854c8ee73a180fc53
    INSTALL_COMMAND ""
)

# Resolve directories used by gtest
ExternalProject_Get_Property(googletest source_dir)
ExternalProject_Get_Property(googletest binary_dir)

# Resolve library paths.
if(NOT WIN32)
  set(GTEST_LIBRARY_PATH ${binary_dir}/lib/libgtest.a)
  set(GTEST_MAIN_LIBRARY_PATH ${binary_dir}/lib/libgtest_main.a)
endif(NOT WIN32)

# Make Ninja builds work
# Ninja has a different way of scanning to Make and needs the gtest library artefact files to exist in order to start
# the build at all. We "touch" those files here.
file(WRITE ${GTEST_LIBRARY_PATH} "")
file(WRITE ${GTEST_MAIN_LIBRARY_PATH} "")

# Setup import targets
add_library(gtest UNKNOWN IMPORTED)
set_property(TARGET gtest PROPERTY IMPORTED_LOCATION ${GTEST_LIBRARY_PATH})
add_dependencies(gtest googletest)

add_library(gtest_main UNKNOWN IMPORTED)
set_property(TARGET gtest_main PROPERTY IMPORTED_LOCATION ${GTEST_MAIN_LIBRARY_PATH})
add_dependencies(gtest_main gtest googletest)

# Setup GTest variables, mimicing FindGTest.cmake
set(GTEST_INCLUDE_DIR "${source_dir}/include")
set(GTEST_INCLUDE_DIRS "${GTEST_INCLUDE_DIR}")
set(GTEST_LIBRARIES gtest Threads::Threads)
set(GTEST_MAIN_LIBRARIES gtest_main)
