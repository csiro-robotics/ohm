# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)

set(SOURCES
  OhmTestUtil.cpp
  OhmTestUtil.h
)

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
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmtestcommon ohmtools ohm ohmutil)

add_dependencies(ohmtestcommon marshal_test_data)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmtestcommon 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmtestcommon PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
