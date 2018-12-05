find_package(GLM REQUIRED)
find_package(PNG REQUIRED)

set(SOURCES
  ohmheightmap2image.cpp
)

add_executable(ohmheightmap2image ${SOURCES})

set_target_properties(ohmheightmap2image PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmheightmap2image PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmheightmap2image SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmheightmap2image ohm ohmutil ${ZLIB_LIBRARIES})

target_link_libraries(ohmheightmap2image ohmheightmaputil PNG::PNG)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmheightmap2image DESTINATION bin)
