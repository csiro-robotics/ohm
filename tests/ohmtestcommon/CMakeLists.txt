# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)

# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

set(SOURCES
  TraversalTest.cpp
  TraversalTest.h
  OhmTestUtil.cpp
  OhmTestUtil.h
  RayPatternTestUtil.h
)

if(Eigen3_FOUND)
  message(STATUS "Eigen3 found. Adding CovarianceTestUtil.")
  list(APPEND SOURCES
    CovarianceTestUtil.cpp
    CovarianceTestUtil.h
  )
endif(Eigen3_FOUND)

add_library(ohmtestcommon ${SOURCES})

set_target_properties(ohmtestcommon PROPERTIES FOLDER tests)

target_include_directories(ohmtestcommon
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tests>
)

target_include_directories(ohmtestcommon SYSTEM
  PRIVATE
    "${GTEST_INCLUDE_DIRS}" # Must include downloaded GTest before potentially including system directories.
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmtestcommon PUBLIC ohmtools ohm ohmutil)

add_dependencies(ohmtestcommon marshal_test_data)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmtestcommon PUBLIC 3es::3es-core ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
else(OHM_TES_DEBUG)
  target_include_directories(ohmtestcommon PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

if(Eigen3_FOUND)
  target_link_libraries(ohmtestcommon PUBLIC Eigen3::Eigen)
endif(Eigen3_FOUND)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
