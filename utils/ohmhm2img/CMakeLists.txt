find_package(GLM REQUIRED)
find_package(PNG REQUIRED)

set(SOURCES
  ohmhm2img.cpp
)

add_executable(ohmhm2img ${SOURCES})

set_target_properties(ohmhm2img PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmhm2img PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmhm2img SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohmhm2img ohm ohmutil ${ZLIB_LIBRARIES} ohmheightmaputil PNG::PNG)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmhm2img DESTINATION bin)
