
if(NOT DEFINED GPU_ACCEL)
  set(GPU_ACCEL "OpenCL" CACHE STRING "Select the GPU acceleration API.")
  set_property(CACHE GPU_ACCEL PROPERTY STRINGS CUDA OpenCL)
endif(NOT DEFINED GPU_ACCEL)

set(PUBLIC_HEADERS
  gpuaccessexception.h
  gpuapiexception.h
  gpubuffer.h
  "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuconfig.h"
  gpudevice.h
  gpuevent.h
  gpueventlist.h
  gpuexception.h
  gpupinmode.h
  gpupinnedbuffer.h
  gpuplatform.h
  gpuqueue.h
  gputhrow.h
  gpu_ext.h
)

set(SOURCES
  gpuaccessexception.cpp
  gpuapiexception.cpp
  gpuconfig.in.h
  gpueventlist.cpp
  gpuexception.cpp
  gputhrow.cpp
  )

list(APPEND SOURCES ${PUBLIC_HEADERS})

set(GPUTIL_TYPE 0)
if(GPU_ACCEL STREQUAL "OpenCL")
  find_package(OpenCL)
  set(GPUTIL_TYPE 1)

  list(APPEND SOURCES
    cl/gpuapiexceptioncode.cpp
    cl/gpubuffer.cpp
    cl/gpubufferdetail.h
    cl/gpudevice.cpp
    cl/gpudevicedetail.h
    cl/gpuevent.cpp
    cl/gpueventdetail.h
    cl/gpupinnedbuffer.cpp
    cl/gpuplatform2.h
    cl/gpuqueue.cpp
    cl/gpuqueuedetail.h
    )

  set(PUBLIC_PLATFORM_HEADERS
    cl/gpuplatform2.h
  )

  set(PUBLIC_PLATFORM_HEADERS_DIR cl)
elseif(GPU_ACCEL STREQUAL "CUDA")
  find_package(CUDA)
  set(GPUTIL_TYPE 2)

  list(APPEND SOURCES
    cuda/cudaimportcl.h
    cuda/cutil_math.h
    cuda/gpuapiexceptioncode.cpp
    cuda/gpubuffer.cpp
    cuda/gpubufferdetail.h
    cuda/gpudevice.cpp
    cuda/gpudevicedetail.h
    cuda/gpuevent.cpp
    cuda/gpueventdetail.h
    cuda/gpupinnedbuffer.cpp
    cuda/gpuplatform2.h
    cuda/gpuqueue.cpp
    cuda/gpuqueuedetail.h
    cuda/ref.h
    )

  set(PUBLIC_PLATFORM_HEADERS
    cuda/gpuplatform2.h
  )

  set(PUBLIC_PLATFORM_HEADERS_DIR cuda)
else()
  message(SEND_ERROR "No GPU acceleration selected.")
endif()

configure_file(gpuconfig.in.h "${CMAKE_CURRENT_BINARY_DIR}/gputil/gpuconfig.h")

if(GPU_ACCEL STREQUAL "CUDA")
  cuda_add_library(gputil STATIC ${SOURCES})
  target_include_directories(gputil PRIVATE
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/gputil>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  PRIVATE
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
