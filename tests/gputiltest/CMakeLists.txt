include(TextFileResource)

find_package(GTest)
find_package(CUDA)

set(SOURCES
  GpuBufferTest.cpp
  GpuDeviceTest.cpp
  KernelTest.cpp
  TestMain.cpp
)

set(GPU_SOURCES
  matrix.cl
)

function(_gputiltest_setup_target TARGET_NAME GPUTIL_LIBRARY)
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER tests)
  if(MSVC)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
  endif(MSVC)

  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_include_directories(${TARGET_NAME} SYSTEM
    PRIVATE
      "${GTEST_INCLUDE_DIRS}"
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
  )

  target_link_libraries(${TARGET_NAME} ${GPUTIL_LIBRARY} GTest::Main)

  add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME} --gtest_output=xml:test-reports/)

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_gputiltest_setup_target)

if(OPENCL_FOUND)
  get_target_property(GPUTIL_DEVICE_INCLUDE_DIRS gputilocl SOURCE_DIR)
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

  add_executable(gputiltestocl ${SOURCES} ${GPU_SOURCES})
  _gputiltest_setup_target(gputiltestocl gputilocl)
endif(OPENCL_FOUND)

if(CUDA_FOUND)
  list(APPEND GPU_SOURCES
    cuda/matrix_kernel.cu
  )
  cuda_add_executable(gputiltestcuda ${SOURCES} ${GPU_SOURCES})
  _gputiltest_setup_target(gputiltestcuda gputilcuda)
endif(CUDA_FOUND)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
