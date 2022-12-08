include(TextFileResource)

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
  leak_track_target_enable(${TARGET_NAME} CONDITION OHM_LEAK_TRACK)
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER tests)
  if(MSVC)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
  endif(MSVC)

  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_include_directories(${TARGET_NAME}
    SYSTEM PRIVATE
      "${GTEST_INCLUDE_DIRS}"
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
  )

  target_link_libraries(${TARGET_NAME} PUBLIC ${GPUTIL_LIBRARY} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} logutil)

  add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME} --gtest_output=xml:test-reports/)

  install(TARGETS ${TARGET_NAME} DESTINATION bin)
endfunction(_gputiltest_setup_target)

if(OHM_FEATURE_OPENCL)
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
  # Required to run NVIDIA OpenCL
  leak_track_default_options(gputiltestocl CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(gputiltestocl CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_OCL}
  )
endif(OHM_FEATURE_OPENCL)

if(OHM_FEATURE_CUDA)
  list(APPEND GPU_SOURCES
    cuda/matrix_kernel.cu
  )
  if(OHM_USE_DEPRECATED_CMAKE_CUDA)
    cuda_add_executable(gputiltestcuda ${SOURCES} ${GPU_SOURCES})
  else(OHM_USE_DEPRECATED_CMAKE_CUDA)
    add_executable(gputiltestcuda ${SOURCES} ${GPU_SOURCES})
  endif(OHM_USE_DEPRECATED_CMAKE_CUDA)
  _gputiltest_setup_target(gputiltestcuda gputilcuda)
  leak_track_default_options(gputiltestcuda CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(gputiltestcuda CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_CUDA}
  )
endif(OHM_FEATURE_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})
