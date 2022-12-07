# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)



# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

set(SOURCES
  OhmTestUtil.cpp
  OhmTestUtil.h
  RayPatternTestUtil.h
  TraversalTest.cpp
  TraversalTest.h
  WalkSegmentKeysLegacy.cpp
  WalkSegmentKeysLegacy.h
)

if(Eigen3_FOUND)
  message(STATUS "Eigen3 found. Adding CovarianceTestUtil.")
  list(APPEND SOURCES
    CovarianceTestUtil.cpp
    CovarianceTestUtil.h
  )
endif(Eigen3_FOUND)

add_library(ohmtestcommon STATIC ${SOURCES})

set_target_properties(ohmtestcommon PROPERTIES FOLDER tests)

target_include_directories(ohmtestcommon
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tests>
)

target_link_libraries(ohmtestcommon PUBLIC ohmtools ohm ohmutil)

target_link_libraries(ohmtestcommon
  PRIVATE
    ${GTEST_LIBRARIES}
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_EIGEN}>:Eigen3::Eigen>>
)

add_dependencies(ohmtestcommon marshal_test_data)

target_include_directories(ohmtestcommon
  SYSTEM PRIVATE
    "${GTEST_INCLUDE_DIRS}"
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
  )

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
