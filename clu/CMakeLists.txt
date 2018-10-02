find_package(OpenCL)

set(SOURCES
  cl.hpp
  clu.cpp
  clu.h
  clubuffer.h
  cluconstraint.cpp
  cluconstraint.h
  clukernel.cpp
  clukernel.h
  cluprogram.cpp
  cluprogram.h
)

set(PUBLIC_HEADERS
  cl.hpp
  clu.h
  clubuffer.h
  cluconstraint.h
  clukernel.h
  cluprogram.h
)

add_library(clu STATIC ${SOURCES})
target_include_directories(clu
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(clu SYSTEM
  PUBLIC
    "${OpenCL_INCLUDE_DIRS}"
)
# set_property(TARGET clu PROPERTY DEBUG_POSTFIX "d")

install(TARGETS clu EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/clu
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/clu)

source_group("source" REGULAR_EXPRESSION ".*$")
