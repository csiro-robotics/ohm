find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohmheightmap.cpp
)

add_executable(ohmheightmap ${SOURCES})

set_target_properties(ohmheightmap PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmheightmap PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmheightmap SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmheightmap ohm ohmutil ${ZLIB_LIBRARIES})

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmheightmap DESTINATION bin)
