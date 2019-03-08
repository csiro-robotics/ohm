if(NOT LIBLAS_FOUND)
  message(STATUS "Skipping ohmcloudfilter. Missing libLAS")
  return()
endif()

# Need to add boost link directories because of LIBLAS.
link_directories("${Boost_LIBRARY_DIRS}")

set(SOURCES
  ohmcloudfilter.cpp
)

add_executable(ohmcloudfilter ${SOURCES})

set_target_properties(ohmcloudfilter PROPERTIES FOLDER utils)
if(MSVC)
  set_target_properties(ohmcloudfilter PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(ohmcloudfilter SYSTEM
  PRIVATE
    "${GLM_INCLUDE_DIR}"
    # "${ZLIB_INCLUDE_DIRS}"
    "${LIBLAS_INCLUDE_DIR}"
)

target_link_libraries(ohmcloudfilter
  ohm
  ohmutil
  ${LIBLAS_LIBRARIES}
)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS ohmcloudfilter DESTINATION bin)
