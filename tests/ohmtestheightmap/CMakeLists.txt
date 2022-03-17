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

target_include_directories(ohmtestheightmap SYSTEM
  PRIVATE
    "${GTEST_INCLUDE_DIRS}" # Must include downloaded GTest before potentially including system directories.
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmtestheightmap PUBLIC ohmtestcommon ohmtools ohmheightmap ohmutil ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

if(OHM_TES_DEBUG)
  target_link_libraries(ohmtestheightmap PUBLIC 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmtestheightmap SYSTEM PRIVATE "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

add_test(NAME ohmtestheightmap COMMAND ohmtestheightmap --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtestheightmap DESTINATION bin)
