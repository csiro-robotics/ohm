
include(GenerateExportHeader)
include(TextFileResource)

find_package(GLM)
find_package(ZLIB)
if(OHM_THREADS)
  find_package(TBB)
endif(OHM_THREADS)

# Configure OHM_GPU as a value for configure_file
set(OHM_GPU_OPENCL 1)
set(OHM_GPU_CUDA 2)

if(OHM_GPU STREQUAL "OpenCL")
  set(OHM_GPU ${OHM_GPU_OPENCL})
else()
  set(OHM_GPU ${OHM_GPU_CUDA})
endif(OHM_GPU STREQUAL "OpenCL")

set(TES_ENABLE ${OHM_TES_DEBUG})
configure_file(ohmconfig.in.h ohmconfig.h)
set(SOURCES "${CMAKE_CURRENT_BINARY_DIR}/ohmconfig.h")

set(SOURCES
  private/clearanceprocessdetail.cpp
  private/clearanceprocessdetail.h
  private/maplayerdetail.h
  private/maplayoutdetail.h
  private/occupancylinequerydetail.h
  private/occupancymapdetail.cpp
  private/occupancymapdetail.h
  private/occupancynearestneighboursdetail.h
  private/occupancynodealgorithms.cpp
  private/occupancynodealgorithms.h
  private/occupancyquerydetail.h
  private/RoiRangeFill.cpp
  private/RoiRangeFill.h
  private/voxellayoutdetail.h
  mapcache.cpp
  mapcache.h
  mapchunk.cpp
  mapchunk.h
  maplayer.cpp
  maplayer.h
  maplayout.cpp
  maplayout.h
  mapnode.h
  mapprobability.h
  mapregion.cpp
  mapregion.h
  occupancygpu.cpp
  occupancygpu.h
  occupancykey.cpp
  occupancykey.h
  occupancykeylist.cpp
  occupancykeylist.h
  occupancylinekeysquery.cpp
  occupancylinekeysquery.h
  occupancylinequery.cpp
  occupancylinequery.h
  occupancymap.cpp
  occupancymap.h
  occupancymapserialise.cpp
  occupancymapserialise.h
  occupancynearestneighbours.cpp
  occupancynearestneighbours.h
  occupancynode.cpp
  occupancynode.h
  occupancyquery.cpp
  occupancyquery.h
  occupancyqueryflag.h
  occupancytype.cpp
  occupancytype.h
  ohmclearanceprocess.cpp
  ohmclearanceprocess.h
  ohmdatatype.cpp
  ohmdatatype.h
  ohmdefaultlayers.h
  ohmmapper.cpp
  ohmmapper.h
  ohmmappingprocess.cpp
  ohmmappingprocess.h
  ohmstream.cpp
  ohmstream.h
  ohmvoxellayout.cpp
  ohmvoxellayout.h
)

if(TES_ENABLE)
  # Only required to define the 3es server symbol.
  list(APPEND SOURCES occupancyutil.cpp)
endif(TES_ENABLE)

set(PUBLIC_HEADERS
  mapcache.h
  mapchunk.h
  maplayer.h
  maplayout.h
  mapnode.h
  mapprobability.h
  mapregion.h
  occupancygpu.h
  occupancykey.h
  occupancykeylist.h
  occupancylinekeysquery.h
  occupancylinequery.h
  occupancymap.h
  occupancymapserialise.h
  occupancynearestneighbours.h
  occupancynode.h
  occupancyquery.h
  occupancyqueryflag.h
  occupancytype.h
  ohmdatatype.h
  ohmclearanceprocess.h
  ohmdefaultlayers.h
  ohmmapper.h
  ohmmappingprocess.h
  ohmvoxellayout.h
  "${CMAKE_CURRENT_BINARY_DIR}/ohmconfig.h"
  "${CMAKE_CURRENT_BINARY_DIR}/ohmexport.h"
  )

if(OHM_GPU)
  # Add gputil:gpu_ext.h to this project for correct marshalling.
  # Always re-resolve gpu_ext.h in case it moves.
  unset(GPU_EXT_H CACHE)
  find_file(GPU_EXT_H gpu_ext.h PATHS ${GPUTIL_DEVICE_INCLUDE_DIRS})

  set(GPU_SOURCES
    gpu/linekeys.cl
    gpu/nearestneighboursquery.cl
    gpu/regionupdate.cl
    gpu/roirangefill.cl
  )
  if(NOT OHM_EMBED_GPU_CODE)
    list(APPEND GPU_SOURCES gpu/linewalk.cl)
    list(APPEND GPU_SOURCES gpu/regions.cl)
  endif(NOT OHM_EMBED_GPU_CODE)

  set(GPU_HEADERS
    gpukey.h
    mapnode.h
  )

  list(APPEND SOURCES
    private/occupancygpumapdetail.cpp
    private/occupancygpumapdetail.h
    gpucache.cpp
    gpucache.h
    gpucacheparams.h
    gpulayercache.cpp
    gpulayercache.h
    occupancygpumap.cpp
    occupancygpumap.h
  )

  list(APPEND PUBLIC_HEADERS
    gpucache.h
    gpucacheparams.h
    gpulayercache.h
    occupancygpumap.h
  )
endif(OHM_GPU)

set(USE_CUDA FALSE)
if(OHM_GPU)
  list(APPEND SOURCES ${GPU_SOURCES} ${GPU_HEADERS} "${GPU_EXT_H}")
  if(OHM_GPU_API STREQUAL "OpenCL")
    get_target_property(CLU_INCLUDE_DIRS clu INCLUDE_DIRECTORIES)
    list(APPEND SOURCES
      cl/clprogram.cpp
      cl/clprogram.h
      cl/gpumap_cl.cpp
      cl/linekeysquery_cl.cpp
      cl/nearestneighboursquery_cl.cpp
      cl/RoiRangeFill.cpp
    )

    if(OHM_EMBED_GPU_CODE)
      # Embedding GPU code. Generate source file resources.
      # Only need to embed for OpenCL.
      foreach(CLSOURCE ${GPU_SOURCES})
        get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
        get_filename_component(CLSOURCENAME_WE "${CLSOURCE}" NAME_WE)
        text_file_resource("${CLSOURCE}" "${CLSOURCENAME_WE}Code"
          TYPE opencl
          PATHS "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/gpu" "${GPUTIL_DEVICE_INCLUDE_DIRS}"
          # Append to the SOURCES list.
          FILELIST SOURCES
        )
      endforeach(CLSOURCE)
    endif(OHM_EMBED_GPU_CODE)
  else()
    # CUDA
    set(USE_CUDA TRUE)
    find_package(CUDA)
    list(APPEND SOURCES
      cuda/nearestneighboursquery.cu
      cuda/roirangefill.cu
    )
  endif(OHM_GPU_API STREQUAL "OpenCL")
endif(OHM_GPU)

if(USE_CUDA)
  cuda_add_library(ohm ${SOURCES})
else(USE_CUDA)
  add_library(ohm ${SOURCES})
endif(USE_CUDA)
target_link_libraries(ohm ${ZLIB_LIBRARIES} 3es::3es-core ohmutil gputil)

if(OHM_GPU_API STREQUAL "OpenCL")
  target_link_libraries(ohm clu)

  if(NOT OHM_EMBED_GPU_CODE)
    # Not embedding GPU source code. Make sure we marshal GPU headers as well as code.
    get_filename_component(CLSOURCENAME "${GPU_EXT_H}" NAME)
    add_custom_command(
      OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${GPU_EXT_H}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
      MAIN_DEPENDENCY "${GPU_EXT_H}"
      COMMENT "Copying ${CLSOURCENAME}"
    )

    list(APPEND GPU_SOURCES "${GPU_HEADERS}")
    foreach(CLSOURCE ${GPU_SOURCES})
      get_filename_component(CLSOURCENAME "${CLSOURCE}" NAME)
      # Not embedding GPU code. Marshal GPU source files.
      add_custom_command(
        OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_LIST_DIR}/${CLSOURCE}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CLSOURCENAME}"
        MAIN_DEPENDENCY "${CMAKE_CURRENT_LIST_DIR}/${CLSOURCE}"
        COMMENT "Copying ${CLSOURCE}"
      )
    endforeach(CLSOURCE)
  endif(NOT OHM_EMBED_GPU_CODE)
endif()

generate_export_header(ohm
      EXPORT_MACRO_NAME ohm_API
      EXPORT_FILE_NAME ohmexport.h
      STATIC_DEFINE OHM_STATIC)

target_include_directories(ohm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_include_directories(ohm SYSTEM
  PUBLIC "${GLM_INCLUDE_DIR}"
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../3rdparty/tes/3es-core>
  PRIVATE "${ZLIB_INCLUDE_DIR}"
)

if(OHM_THREADS)
  # target_include_directories(ohm SYSTEM
  #   PRIVATE "${TBB_INCLUDE_DIR}"
  # )
  target_link_libraries(ohm ${TBB_LIBRARIES})
endif(OHM_THREADS)


install(TARGETS ohm EXPORT ${CMAKE_PROJECT_NAME}-config-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES ${PUBLIC_HEADERS} DESTINATION include/ohm)

source_group("source" REGULAR_EXPRESSION ".*$")
source_group("source\\cl" REGULAR_EXPRESSION "/cl/.*$")
source_group("source\\gpu" REGULAR_EXPRESSION "/gpu/.*$")
source_group("source\\private" REGULAR_EXPRESSION "/private/.*$")
