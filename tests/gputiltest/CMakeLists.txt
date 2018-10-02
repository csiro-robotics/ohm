# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GTest)

include_directories(.)

set(SOURCES
  gpubuffertest.cpp
  testmain.cpp
)

add_executable(gputiltest ${SOURCES})
set_target_properties(gputiltest PROPERTIES FOLDER tests)
if(MSVC)
  set_target_properties(gputiltest PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(gputiltest
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(gputiltest SYSTEM
  PRIVATE
    "${GTEST_INCLUDE_DIRS}"
)

target_link_libraries(gputiltest ohmutil gputil GTest::Main)

add_test(NAME gputiltest COMMAND gputiltest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS gputiltest DESTINATION bin)
