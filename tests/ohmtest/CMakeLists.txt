# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)
find_package(PNG QUIET)

# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

if(PNG_FOUND)
  set(OHM_WITH_PNG ON)
endif(PNG_FOUND)
configure_file(OhmTestConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/OhmTestConfig.h")

set(SOURCES
  HeightmapTests.cpp
  KeyTests.cpp
  LayoutTests.cpp
  LineQueryTests.cpp
  MapTests.cpp
  MathsTests.cpp
  OhmTestConfig.in.h
  SerialisationTests.cpp
  VoxelMeanTests.cpp
  RayPatternTests.cpp
  RayValidation.cpp
  RayValidation.h
  TestMain.cpp
  "${CMAKE_CURRENT_BINARY_DIR}/OhmTestConfig.h"
)

if(Eigen3_FOUND)
  message(STATUS "Eigen3 found. Adding NDT tests.")
  list(APPEND SOURCES NdtTests.cpp)
endif(Eigen3_FOUND)

add_executable(ohmtest ${SOURCES})
leak_track_target_enable(ohmtest CONDITION OHM_LEAK_TRACK)
leak_track_suppress(ohmtest CONDITION OHM_LEAK_TRACK
  ${OHM_LEAK_SUPPRESS_TBB}
)

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

target_link_libraries(ohmtest PUBLIC ohmtestcommon ohmtools ohm ohmutil ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
if(PNG_FOUND)
  target_link_libraries(ohmtest PUBLIC PNG::PNG)
endif(PNG_FOUND)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmtest PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmtest PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

if(Eigen3_FOUND)
  target_link_libraries(ohmtest PRIVATE Eigen3::Eigen)
endif(Eigen3_FOUND)

add_test(NAME ohmtest COMMAND ohmtest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
