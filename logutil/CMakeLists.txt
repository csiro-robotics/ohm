
include(GenerateExportHeader)
include(TextFileResource)

configure_file(LogUtilConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/logutil/LogUtilConfig.h")

set(SOURCES
  Logger.cpp
  Logger.h
  LoggerDetail.h
  LogUtil.cpp
  LogUtil.h
)

set(PUBLIC_HEADERS
  "${CMAKE_CURRENT_BINARY_DIR}/logutil/LogUtilConfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/logutil/LogUtilExport.h"
  Logger.h
  LoggerDetail.h
  LogUtil.h
)

add_library(logutil ${SOURCES})
clang_tidy_target(logutil)

target_include_directories(logutil
  PUBLIC
    $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/logutil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

generate_export_header(logutil
  EXPORT_MACRO_NAME logutil_API
  EXPORT_FILE_NAME logutil/LogUtilExport.h
  STATIC_DEFINE LOGUTIL_STATIC)

install(TARGETS logutil EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${OHM_PREFIX_INCLUDE}/logutil
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/logutil)

source_group("source" REGULAR_EXPRESSION ".*$")
