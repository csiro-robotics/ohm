
set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(OhmQueryConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h")
set(SOURCES "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h")

set(SOURCES
  ohmquery.cpp
  OhmQueryConfig.in.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmquery/OhmQueryConfig.h"
)

add_executable(ohmquery ${SOURCES})

set_target_properties(ohmquery PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmquery PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmquery
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ohmquery>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(ohmquery SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
)

target_link_libraries(ohmquery ${OHM_LIBRARY} ohmutil)

if(OHM_TES_DEBUG)
  target_link_libraries(ohmquery 3es::3es-core)
else(OHM_TES_DEBUG)
  target_include_directories(ohmquery PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
endif(OHM_TES_DEBUG)

install(TARGETS ohmquery DESTINATION bin)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
