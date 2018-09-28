
set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(ohmqueryconfig.in.h ohmqueryconfig.h)
set(SOURCES "${CMAKE_CURRENT_BINARY_DIR}/ohmconfig.h")

set(SOURCES
  ohmquery.cpp
  ohmqueryconfig.in.h
)

add_executable(ohmquery ${SOURCES})

set_target_properties(ohmquery PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmquery PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmquery
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_include_directories(ohmquery SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmquery ohm ohmutil)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmquery 3es-core)
endif(OHM_TES_DEBUG)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmquery DESTINATION bin)
