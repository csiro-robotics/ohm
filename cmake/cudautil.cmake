#==============================================================================
# Setup some CUDA complilation control options using the given PREFIX.
#==============================================================================
function(nvcc_setup PREFIX)
  set(${PREFIX}_CUDA_ARCHITECTURES "30" CACHE STRING "Specifies the list of CUDA compute architectures to compile for.")
  foreach(ARCH ${${PREFIX}_CUDA_ARCHITECTURES})
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_${ARCH},code=sm_${ARCH}")
  endforeach(ARCH)
  nvcc_fix_definitions()

  set(${PREFIX}_CUDA_DEBUG "none" CACHE STRING "Controls the CUDA debug level, regardless of the current build type.")
  set_property(CACHE ${PREFIX}_CUDA_DEBUG PROPERTY STRINGS full lineinfo none)

  find_package(CUDA)
  if(${PREFIX}_CUDA_DEBUG STREQUAL "full")
    list(APPEND CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -g -G -O0 -lineinfo")
  elseif(${PREFIX}_CUDA_DEBUG STREQUAL "lineinfo")
    list(APPEND CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -g -lineinfo")
  endif(${PREFIX}_CUDA_DEBUG STREQUAL "full")
  if(NOT ${PREFIX}_CUDA_DEBUG STREQUAL "none")
    set(CUDA_NVCC_FLAGS_DEBUG "${CUDA_NVCC_FLAGS_DEBUG} -g -G -O0 -lineinfo")
  endif(NOT ${PREFIX}_CUDA_DEBUG STREQUAL "none")

  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" PARENT_SCOPE)
endfunction(nvcc_setup)

#==============================================================================
# This function deals with a peculiar issue with the generation of the
# generated CUDA compilation script.
#
# The CUDA compilation script can end up with bad values in nvcc_flags
# as extracted from the directory property COMPILE_DEFINITIONS. In particular,
# VTK leaves definitions in two poor forms:
# - vtkRenderingContext2D_AUTOINIT=1(vtkRenderingContextOpenGL)
# - vtkRenderingCore_INCLUDE="this/is/a/path"
#
# The first has an issue with brackets, the second the issue is the quotes not
# being escaped. Likely, neither would be an issue if the nvcc_flags value was
# properly quoted when generated.
#==============================================================================
function(nvcc_fix_definitions)
  get_directory_property(CUDA_NVCC_DEFINITIONS COMPILE_DEFINITIONS)
  # VTK compile definitions interfere with the CUDA definitions. For now take the easy options and remove problem options.
  #message("CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA-CUDA")
  #message("CUDA_NVCC_DEFINITIONS: ${CUDA_NVCC_DEFINITIONS}")
  set(CUDA_NVCC_DEFINITIONS_FIXED)
  if(CUDA_NVCC_DEFINITIONS)
    foreach(_definition ${CUDA_NVCC_DEFINITIONS})
      set(USEDEF "${_definition}")
      if(_definition MATCHES ".*=.*\\(.*\\)")
        # Handle VKT options in the form: vtkRenderingContext2D_AUTOINIT=1(vtkRenderingContextOpenGL)
        string(REGEX REPLACE "\\(.*\\)" "" _definitionFixed "${_definition}")
        #message("Problem: ${_definition} -> ${_definitionFixed}")
        set(_definition "${_definitionFixed}")
      elseif (_definition MATCHES ".*_INCLUDE=\\\".*\\\"")
        # vtkRenderingCore_INCLUDE="path\path"
        # The issue is the quotes not getting escaped later.
        string(REGEX REPLACE "\\\"" "\\\\\"" _definitionFixed "${_definition}")
        #message("Problem: ${_definition} -> ${_definitionFixed}")
        set(_definition "${_definitionFixed}")
      endif()
      #message("${_definition}")
      list(APPEND CUDA_NVCC_DEFINITIONS_FIXED "${_definition}")
    endforeach()
    #message("CUDA_NVCC_DEFINITIONS_FIXED: ${CUDA_NVCC_DEFINITIONS_FIXED}")
    set_property(DIRECTORY . PROPERTY COMPILE_DEFINITIONS "${CUDA_NVCC_DEFINITIONS_FIXED}")
    get_directory_property(CUDA_NVCC_DEFINITIONS COMPILE_DEFINITIONS)
    #message("CUDA_NVCC_DEFINITIONS: ${CUDA_NVCC_DEFINITIONS}")
  endif()
endfunction(nvcc_fix_definitions)
