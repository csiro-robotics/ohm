
if(NOT DEFINED GPU_ACCEL)
  set(GPU_ACCEL "OpenCL" CACHE STRING "Select the GPU acceleration API.")
  set_property(CACHE GPU_ACCEL PROPERTY STRINGS CUDA OpenCL)
endif(NOT DEFINED GPU_ACCEL)

set(PUBLIC_HEADERS
  gpu_ext.h
  gpuAccessException.h
  gpuApiException.h
  gpuBuffer.h
  "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h"
  gpuDevice.h
  gpuDeviceInfo.h
  gpuEvent.h
  gpuEventList.h
  gpuException.h
  gpuKernel.h
  gpuPinMode.h
  gpuPinnedBuffer.h
  gpuPlatform.h
  gpuProgram.h
  gpuQueue.h
  gpuThrow.h
  gpuVersion.h
)

set(SOURCES
  gpuAccessException.cpp
  gpuApiException.cpp
  gpuConfig.in.h
  gpuEventList.cpp
  gpuException.cpp
  gpuThrow.cpp
)

list(APPEND SOURCES ${PUBLIC_HEADERS})

set(GPUTIL_TYPE 0)
if(GPU_ACCEL STREQUAL "OpenCL")
  find_package(OpenCL)
  set(GPUTIL_TYPE 1)

  list(APPEND SOURCES
    cl/gpuApiExceptionCode.cpp
    cl/gpuBuffer.cpp
    cl/gpuBufferDetail.h
    cl/gpuDevice.cpp
    cl/gpuDeviceDetail.h
    cl/gpuEvent.cpp
    cl/gpuEventDetail.h
    cl/gpuKernel.cpp
    cl/gpuKernel2.h
    cl/gpuKernelDetail.h
    cl/gpuPinnedBuffer.cpp
    cl/gpuPlatform2.h
    cl/gpuProgram.cpp
    cl/gpuProgramDetail.h
    cl/gpuQueue.cpp
    cl/gpuQueueDetail.h
    )

  set(PUBLIC_PLATFORM_HEADERS
    cl/gpuBufferDetail.h
    cl/gpuDeviceDetail.h
    cl/gpuKernel2.h
    cl/gpuKernelDetail.h
    cl/gpuPlatform2.h
    cl/gpuProgramDetail.h
    cl/gpuQueueDetail.h
  )

  set(PUBLIC_PLATFORM_HEADERS_DIR cl)
elseif(GPU_ACCEL STREQUAL "CUDA")
  find_package(CUDA)
  set(GPUTIL_TYPE 2)

  list(APPEND SOURCES
    cuda/cutil_importcl.h
    cuda/cutil_math.h
    cuda/gpuApiExceptionCode.cpp
    cuda/gpuBuffer.cpp
    cuda/gpuBufferDetail.h
    cuda/gpuDevice.cpp
    cuda/gpuDeviceDetail.h
    cuda/gpuEvent.cpp
    cuda/gpuEventDetail.h
    cuda/gpuKernel.cpp
    cuda/gpuKernel2.h
    cuda/gpuKernelDetail.h
    cuda/gpuPinnedBuffer.cpp
    cuda/gpuPlatform2.h
    cuda/gpuProgram.cpp
    cuda/gpuProgramDetail.h
    cuda/gpuQueue.cpp
    cuda/gpuQueueDetail.h
    cuda/ref.h
    )

  set(PUBLIC_PLATFORM_HEADERS
    cuda/gpuPlatform2.h
  )

  set(PUBLIC_PLATFORM_HEADERS_DIR cuda)
else()
  message(SEND_ERROR "No GPU acceleration selected.")
endif()

configure_file(gpuConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h")

if(GPU_ACCEL STREQUAL "CUDA")
  cuda_add_library(gputil STATIC ${SOURCES})
  target_include_directories(gputil
    PUBLIC
      # TODO: make the need for CUDA directories private.
      "${CUDA_INCLUDE_DIRS}"
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cuda>
  )
else()
  add_library(gputil STATIC ${SOURCES})
  target_include_directories(gputil PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cl>
  )
endif(GPU_ACCEL STREQUAL "CUDA")

target_include_directories(gputil
  PUBLIC
  $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
  $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}/gputil>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/gputil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

# Set device headers variable for other projects to resolve device include paths
set(GPUTIL_DEVICE_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "Include directories for the device to use gputil features")

if(GPU_INCLUDE_DIRS)
  target_include_directories(gputil SYSTEM PRIVATE "${GPU_INCLUDE_DIRS}")
endif(GPU_INCLUDE_DIRS)

# set_property(TARGET gputil PROPERTY DEBUG_POSTFIX "d")

if(GPU_ACCEL STREQUAL "OpenCL")
  target_link_libraries(gputil clu ${OpenCL_LIBRARIES})
endif(GPU_ACCEL STREQUAL "OpenCL")


install(TARGETS gputil EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/gputil)
install(FILES ${PUBLIC_PLATFORM_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/gputil/${PUBLIC_PLATFORM_HEADERS_DIR})

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\cuda" REGULAR_EXPRESSION "/cuda/.*$")
