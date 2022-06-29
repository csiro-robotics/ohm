# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)

set(SOURCES
  HeightmapTests.cpp
)

add_executable(ohmtestheightmap ${SOURCES})
leak_track_target_enable(ohmtestheightmap CONDITION OHM_LEAK_TRACK)
leak_track_suppress(ohmtestheightmap CONDITION OHM_LEAK_TRACK
  ${OHM_LEAK_SUPPRESS_TBB}
)

set_target_properties(ohmtestheightmap PROPERTIES FOLDER tests)
if(MSVC)
  set_target_properties(ohmtestheightmap PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmtestheightmap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_link_libraries(ohmtestheightmap PUBLIC ohmtestcommon ohmtools ohmheightmap ohmutil ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

target_link_libraries(ohmtestheightmap
  PRIVATE
    GTest::gtest
    GTest::gtest_main
    glm::glm
    $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
    $<BUILD_INTERFACE:$<$<BOOL:${WITH_EIGEN}>:Eigen3::Eigen>>
)

target_include_directories(ohmtestheightmap
  SYSTEM PRIVATE
    $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
  )

add_test(NAME ohmtestheightmap COMMAND ohmtestheightmap --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtestheightmap DESTINATION bin)
