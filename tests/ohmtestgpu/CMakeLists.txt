# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)

find_package(GLM)
find_package(GTest)

set(SOURCES
  GpuLineKeysTests.cpp
  GpuLineQueryTests.cpp
  GpuMapperTests.cpp
  GpuMapTest.cpp
  GpuRangesTests.cpp
  GpuRayPatternTests.cpp
  GpuSerialisationTests.cpp
  GpuVoxelMeanTests.cpp
  GpuTestMain.cpp
  GpuTests.cpp
)

function(_ohmtestgpu_setup GPU)
  add_executable(ohmtest${GPU} ${SOURCES})
  leak_track_target_enable(ohmtest${GPU} CONDITION OHM_LEAK_TRACK)

  set_target_properties(ohmtest${GPU} PROPERTIES FOLDER tests)
  if(MSVC)
    set_target_properties(ohmtest${GPU} PROPERTIES DEBUG_POSTFIX "d")
  endif(MSVC)

  target_include_directories(ohmtest${GPU}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_include_directories(ohmtest${GPU} SYSTEM
    PRIVATE
      "${GLM_INCLUDE_DIR}"
      "${GTEST_INCLUDE_DIRS}"
  )

  target_link_libraries(ohmtest${GPU} PUBLIC ohmtestcommon ohmtools ohm${GPU} ohmutil GTest::Main)

  if(OHM_TES_DEBUG)
    target_link_libraries(ohmtest${GPU} PUBLIC 3es::3es-core)
  else(OHM_TES_DEBUG)
    target_include_directories(ohmtest${GPU} PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
  endif(OHM_TES_DEBUG)

  add_test(NAME ohmtest${GPU} COMMAND ohmtest${GPU} --gtest_output=xml:test-reports/)
endfunction(_ohmtestgpu_setup)

if(OHM_BUILD_OPENCL)
  _ohmtestgpu_setup(ocl)
  # Required to run NVIDIA OpenCL
  leak_track_default_options(ohmtestocl CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmtestocl CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_OCL}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmtestgpu_setup(cuda)
  leak_track_default_options(ohmtestcuda CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmtestcuda CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_CUDA}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_BUILD_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
