# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)
find_package(GTest)

set(SOURCES
  GpuMapTest.cpp
  GpuTests.cpp
  HeightmapTests.cpp
  KeyTests.cpp
  LayoutTests.cpp
  LineKeysTests.cpp
  LineQueryTests.cpp
  MapperTests.cpp
  MapTests.cpp
  OhmTestUtil.cpp
  OhmTestUtil.h
  RangesTests.cpp
  SerialisationTests.cpp
  TestMain.cpp
)

add_executable(ohmtest ${SOURCES})

set_target_properties(ohmtest PROPERTIES FOLDER tests)
if(MSVC)
  set_target_properties(ohmtest PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmtest
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmtest SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${GTEST_INCLUDE_DIRS}"
)

target_link_libraries(ohmtest ohmtools ohm ohmutil GTest::Main)

add_dependencies(ohmtest marshal_test_data)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmtest 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmtest PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

add_test(NAME ohmtest COMMAND ohmtest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
