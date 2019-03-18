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
  GpuSerialisationTests.cpp
  GpuSubVoxelTests.cpp
  GpuTestMain.cpp
  GpuTests.cpp
)

function(_ohmtestgpu_setup GPU)
  add_executable(ohmtest${GPU} ${SOURCES})

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

  target_link_libraries(ohmtest${GPU} ohmtestcommon ohmtools ohm${GPU} ohmutil GTest::Main)

  if(OHM_TES_DEBUG)
    target_link_libraries(ohmtest${GPU} 3es::3es-core)
  else(OHM_TES_DEBUG)
    target_include_directories(ohmtest${GPU} PRIVATE SYSTEM "${3ES_INCLUDE_DIRS}")
  endif(OHM_TES_DEBUG)

  add_test(NAME ohmtest${GPU} COMMAND ohmtest${GPU} --gtest_output=xml:test-reports/)
endfunction(_ohmtestgpu_setup)

if(OHM_BUILD_OPENCL)
  _ohmtestgpu_setup(ocl)
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  _ohmtestgpu_setup(cuda)
endif(OHM_BUILD_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
