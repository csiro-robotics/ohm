
if(NOT DEFINED GPU_ACCEL)
  message(FATAL_ERROR "GPU acceleration type not defined (GPU_ACCEL)")
endif(NOT DEFINED GPU_ACCEL)

include(TextFileResource)

find_package(GTest)

set(SOURCES
  GpuBufferTest.cpp
  GpuDeviceTest.cpp
  KernelTest.cpp
  TestMain.cpp
)

set(GPU_SOURCES
  matrix.cl
)


if(GPU_ACCEL STREQUAL "OpenCL")
  # Embedding GPU code. Generate source file resources.
  # Only need to embed for OpenCL.
  foreach(CLSOURCE ${GPU_SOURCES})
    get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
    get_filename_component(CLSOURCENAME_WE "${CLSOURCE}" NAME_WE)
    text_file_resource("${CLSOURCE}" "${CLSOURCENAME_WE}Code"
      TYPE opencl
      # ECHO
      PATHS "${CMAKE_CURRENT_LIST_DIR}" "${GPUTIL_DEVICE_INCLUDE_DIRS}"
      # Append to the SOURCES list.
      FILELIST SOURCES
    )
  endforeach(CLSOURCE)
endif(GPU_ACCEL STREQUAL "OpenCL")

add_executable(gputiltest ${SOURCES} ${GPU_SOURCES})
set_target_properties(gputiltest PROPERTIES FOLDER tests)
if(MSVC)
  set_target_properties(gputiltest PROPERTIES DEBUG_POSTFIX "d")
endif(MSVC)

target_include_directories(gputiltest
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
)

target_include_directories(gputiltest SYSTEM
  PRIVATE
    "${GTEST_INCLUDE_DIRS}"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)


target_link_libraries(gputiltest ohmutil gputil GTest::Main)

add_test(NAME gputiltest COMMAND gputiltest --gtest_output=xml:test-reports/)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

install(TARGETS gputiltest DESTINATION bin)
