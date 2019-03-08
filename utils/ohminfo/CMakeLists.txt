find_package(GLM)
find_package(ZLIB)

set(SOURCES
  ohminfo.cpp
)

add_executable(ohminfo ${SOURCES})

set_target_properties(ohminfo PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohminfo PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohminfo SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    "${ZLIB_INCLUDE_DIRS}"
)

target_link_libraries(ohminfo ohm ohmutil ${ZLIB_LIBRARIES})

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohminfo DESTINATION bin)
