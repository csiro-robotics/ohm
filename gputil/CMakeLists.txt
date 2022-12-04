include(GenerateExportHeader)

if(NOT OHM_FEATURE_CUDA AND NOT OHM_FEATURE_OPENCL)
  message(FATAL_ERROR "Neither OHM_FEATURE_CUDA nor OHM_FEATURE_OPENCL selected for build. A GPU SDK is required to build gputil.")
endif(NOT OHM_FEATURE_CUDA AND NOT OHM_FEATURE_OPENCL)

if(OHM_FEATURE_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)
  # Need to find CUDA for deprecated project configuration
  find_package(CUDA)
endif(OHM_FEATURE_CUDA AND OHM_USE_DEPRECATED_CMAKE_CUDA)

set(GPUTIL_TYPE_OPENCL 1)
set(GPUTIL_TYPE_CUDA 2)

set(PUBLIC_HEADERS
  gpu_ext.h
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
  "${CMAKE_CURRENT_BINARY_DIR}/gputil/gputilExport.h"
  "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h"
)

set(SOURCES
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
  # Because of the way we compile our GPU library twice with different names, we must explicitly define the export
  # macro. Curiously, there's a way to overide all the macros except the one used to control whether to export the
  # symbols or not. This puts us in a position where it could either be ohmcuda_EXPORTS or ohmocl_EXPORTS depending
  # on which targets are enabled. We build both the same way though, so define both symbols for all builds.
  target_compile_definitions(${TARGET_NAME} PRIVATE "-Dgputilcuda_EXPORTS" "-Dgputilocl_EXPORTS")

  # set_property(TARGET gputil PROPERTY DEBUG_POSTFIX "d")

  generate_export_header(${TARGET_NAME}
    EXPORT_MACRO_NAME gputilAPI
    DEPRECATED_MACRO_NAME gputil_DEPRECATED
    NO_EXPORT_MACRO_NAME gputil_NO_EXPORT
    NO_DEPRECATED_MACRO_NAME gputil_NO_DEPRECATED
    EXPORT_FILE_NAME gputil/gputilExport.h
    STATIC_DEFINE GPUTIL_STATIC)

  install(TARGETS ${TARGET_NAME} EXPORT ${CMAKE_PROJECT_NAME}-config-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
endfunction(_gputil_setup_library)

configure_file(gpuConfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuConfig.h")

if(OHM_FEATURE_OPENCL)
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

  add_library(gputilocl ${SOURCES} ${SOURCES_OPENCL})
  clang_tidy_target(gputilocl EXCLUDE_MATCHES ".*\.cl")
  target_include_directories(gputilocl PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cl>
  )

  target_link_libraries(gputilocl PUBLIC clu)
  _gputil_setup_library(gputilocl ${GPUTIL_TYPE_OPENCL})
endif(OHM_FEATURE_OPENCL)

if(OHM_FEATURE_CUDA)
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

  if(OHM_USE_DEPRECATED_CMAKE_CUDA)
    cuda_add_library(gputilcuda ${SOURCES} ${SOURCES_CUDA})
    target_include_directories(gputilcuda SYSTEM PUBLIC "${CUDA_INCLUDE_DIRS}")
  else(OHM_USE_DEPRECATED_CMAKE_CUDA)
    add_library(gputilcuda ${SOURCES} ${SOURCES_CUDA})
    target_link_libraries(gputilcuda PUBLIC "${OHM_CUDA_LIBRARY}")
  endif(OHM_USE_DEPRECATED_CMAKE_CUDA)
  target_link_libraries(gputilcuda PUBLIC logutil)

  clang_tidy_target(gputilcuda EXCLUDE_MATCHES ".*\.cl")
  target_include_directories(gputilcuda SYSTEM
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/cuda>
  )
  _gputil_setup_library(gputilcuda ${GPUTIL_TYPE_CUDA})
endif(OHM_FEATURE_CUDA)

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
