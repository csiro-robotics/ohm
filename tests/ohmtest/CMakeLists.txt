# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)



# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

configure_file(OhmTestConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/OhmTestConfig.h")

set(SOURCES
  CompressionTests.cpp
  CopyTests.cpp
  IncidentsTests.cpp
  KeyTests.cpp
  LayoutTests.cpp
  LineQueryTests.cpp
  LineWalkTests.cpp
  MapTests.cpp
  MathsTests.cpp
  OhmTestConfig.in.h
  SerialisationTests.cpp
  VoxelMeanTests.cpp
  RaysQueryTests.cpp
  RayPatternTests.cpp
  RayValidation.cpp
  RayValidation.h
  SecondarySampleTests.cpp
  TestMain.cpp
  TouchTimeTests.cpp
  TraversalTests.cpp
  TsdfTests.cpp
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
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmtest SYSTEM
  PRIVATE
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
)

target_link_libraries(ohmtest PRIVATE ohmtestcommon ohmtools ohm ohmutil)

target_link_libraries(ohmtest
  PRIVATE
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    glm::glm
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_EIGEN}>:Eigen3::Eigen>>
)

add_test(NAME ohmtest COMMAND ohmtest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
