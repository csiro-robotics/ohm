if(OHM_BUILD_OPENCL)
  find_package(OpenCL QUIET)
endif(OHM_BUILD_OPENCL)
if(OHM_BUILD_CUDA)
  find_package(CUDA QUIET)
endif(OHM_BUILD_CUDA)

if(NOT CUDA_FOUND AND NOT OPENCL_FOUND)
  message(FATAL_ERROR "Neither CUDA nor OpenCL SDK found. A GPU SDK is required to build gputil.")
endif(NOT CUDA_FOUND AND NOT OPENCL_FOUND)

if(NOT OHM_BUILD_CUDA AND NOT OHM_BUILD_OPENCL)
  message(FATAL_ERROR "Neither OHM_BUILD_CUDA nor OHM_BUILD_OPENCL selected for build. A GPU SDK is required to build gputil.")
endif(NOT OHM_BUILD_CUDA AND NOT OHM_BUILD_OPENCL)

set(GPUTIL_TYPE_OPENCL 1)
set(GPUTIL_TYPE_CUDA 2)

set(PUBLIC_HEADERS
  gpu_ext.h
  gpuAccessException.h
  gpuApiException.h
  gpuBuffer.h
  gpuConfig.in.h
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
  "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h"
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

function(_gputil_setup_library TARGET_NAME GPUTIL_TYPE)
  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}>
      $<INSTALL_INTERFACE:${OHM_PREFIX_INCLUDE}/gputil>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/gputil>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
  )

  if(GPU_INCLUDE_DIRS)
    target_include_directories(${TARGET_NAME} SYSTEM PRIVATE "${GPU_INCLUDE_DIRS}")
  endif(GPU_INCLUDE_DIRS)

  # Select GPUTIL_TYPE
  target_compile_definitions(${TARGET_NAME} PUBLIC "-DGPUTIL_TYPE=${GPUTIL_TYPE}")

  # set_property(TARGET gputil PROPERTY DEBUG_POSTFIX "d")

  install(TARGETS ${TARGET_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
endfunction(_gputil_setup_library)

configure_file(gpuConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h")

if(OPENCL_FOUND)
  set(SOURCES_OPENCL
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

  set(PUBLIC_HEADERS_OPENCL
    cl/gpuBufferDetail.h
    cl/gpuDeviceDetail.h
    cl/gpuKernel2.h
    cl/gpuKernelDetail.h
    cl/gpuPlatform2.h
    cl/gpuProgramDetail.h
    cl/gpuQueueDetail.h
  )

  add_library(gputilocl STATIC ${SOURCES} ${SOURCES_OPENCL})
  target_include_directories(gputilocl PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cl>
  )

  target_link_libraries(gputilocl clu ${OpenCL_LIBRARIES})
  _gputil_setup_library(gputilocl ${GPUTIL_TYPE_OPENCL})
endif(OPENCL_FOUND)

if(CUDA_FOUND)
  include(cudautil)
  nvcc_setup(OHM)

  set(SOURCES_CUDA
    cuda/cutil_atomic.h
    cuda/cutil_decl.h
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
    cuda/gpuMemRegion.cpp
    cuda/gpuMemRegion.h
    cuda/gpuPinnedBuffer.cpp
    cuda/gpuPlatform2.h
    cuda/gpuProgram.cpp
    cuda/gpuProgramDetail.h
    cuda/gpuQueue.cpp
    cuda/gpuQueueDetail.h
    cuda/ref.h
    )

  set(PUBLIC_HEADERS_CUDA
    cuda/cutil_atomic.h
    cuda/cutil_decl.h
    cuda/cutil_importcl.h
    cuda/cutil_math.h
    cuda/gpuBufferDetail.h
    cuda/gpuDeviceDetail.h
    cuda/gpuEventDetail.h
    cuda/gpuKernel2.h
    cuda/gpuKernelDetail.h
    cuda/gpuMemRegion.h
    cuda/gpuPlatform2.h
    cuda/gpuProgramDetail.h
    cuda/gpuQueueDetail.h
    cuda/ref.h
  )

  cuda_add_library(gputilcuda STATIC ${SOURCES} ${SOURCES_CUDA})
  target_include_directories(gputilcuda
    PUBLIC
      # TODO: make the need for CUDA directories private.
      "${CUDA_INCLUDE_DIRS}"
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cuda>
  )
  _gputil_setup_library(gputilcuda ${GPUTIL_TYPE_CUDA})
endif(CUDA_FOUND)

install(FILES ${PUBLIC_HEADERS} DESTINATION ${OHM_PREFIX_INCLUDE}/gputil)
if(PUBLIC_HEADERS_CUDA)
  install(FILES ${PUBLIC_HEADERS_CUDA} DESTINATION ${OHM_PREFIX_INCLUDE}/gputil/cuda)
endif(PUBLIC_HEADERS_CUDA)
if(PUBLIC_HEADERS_OPENCL)
  install(FILES ${PUBLIC_HEADERS_OPENCL} DESTINATION ${OHM_PREFIX_INCLUDE}/gputil/cl)
endif(PUBLIC_HEADERS_OPENCL)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\cuda" REGULAR_EXPRESSION "/cuda/.*$")
