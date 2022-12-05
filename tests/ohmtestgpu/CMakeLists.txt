# Setup of GTEST changed at CMake 3.5.
cmake_minimum_required(VERSION 3.5)



# Eigen required to support some tests - NDT in particular
find_package(Eigen3 QUIET)

set(SOURCES
  GpuCopyTests.cpp
  GpuIncidentsTests.cpp
  GpuLineKeysTests.cpp
  GpuLineQueryTests.cpp
  GpuMapperTests.cpp
  GpuMapTest.cpp
  GpuRangesTests.cpp
  GpuRayPatternTests.cpp
  GpuRaysQueryTests.cpp
  GpuSerialisationTests.cpp
  GpuTestMain.cpp
  GpuTests.cpp
  GpuTouchTimeTests.cpp
  GpuTraversalTests.cpp
  GpuTsdfTests.cpp
  GpuVoxelMeanTests.cpp
)

if(Eigen3_FOUND)
  message(STATUS "Eigen3 found. Adding NDT tests.")
  list(APPEND SOURCES GpuNdtTests.cpp)
endif(Eigen3_FOUND)

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

  target_link_libraries(ohmtest${GPU} PUBLIC ohmtestcommon ohmtools ohm${GPU} gputil${GPU} ohmutil)

  target_link_libraries(ohmtest${GPU}
    PRIVATE
      ${GTEST_LIBRARIES}
      ${GTEST_MAIN_LIBRARIES}
      glm::glm
      $<BUILD_INTERFACE:$<$<BOOL:${OHM_TES_DEBUG}>:3es::3es-core>>
      $<BUILD_INTERFACE:$<$<BOOL:${OHM_FEATURE_EIGEN}>:Eigen3::Eigen>>
  )


  target_include_directories(ohmtestcommon
    SYSTEM PUBLIC
      $<$<NOT:$<BOOL:${OHM_TES_DEBUG}>>:${3ES_INCLUDE_DIRS}>
    )

  add_test(NAME ohmtest${GPU} COMMAND ohmtest${GPU} --gtest_output=xml:test-reports/)
endfunction(_ohmtestgpu_setup)

if(OHM_FEATURE_OPENCL)
  _ohmtestgpu_setup(ocl)
  # Required to run NVIDIA OpenCL
  leak_track_default_options(ohmtestocl CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmtestocl CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_OCL}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_FEATURE_OPENCL)
if(OHM_FEATURE_CUDA)
  _ohmtestgpu_setup(cuda)
  leak_track_default_options(ohmtestcuda CONDITION OHM_LEAK_TRACK ${OHM_ASAN_OPTIONS_CUDA})
  leak_track_suppress(ohmtestcuda CONDITION OHM_LEAK_TRACK
    ${OHM_LEAK_SUPPRESS_CUDA}
    ${OHM_LEAK_SUPPRESS_TBB}
  )
endif(OHM_FEATURE_CUDA)

source_group("source" REGULAR_EXPRESSION ".*$")
# Needs CMake 3.8+:
# source_group(TREE "${CMAKE_CURRENT_LIST_DIR}" PREFIX source FILES ${SOURCES})

# install(TARGETS ohmtest DESTINATION bin)
