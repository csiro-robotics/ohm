#==============================================================================
# Setup some CUDA complilation control options using the given PREFIX.
#==============================================================================
function(nvcc_setup PREFIX)
  find_package(CUDA)
  set(${PREFIX}_CUDA_ARCHITECTURES "50;60;61;70;75" CACHE STRING "Specifies the list of CUDA compute architectures to compile for.")
  if (${PREFIX}_CUDA_ARCHITECTURES)
    foreach(ARCH ${${PREFIX}_CUDA_ARCHITECTURES})
      _nvcc_flags_append(CUDA_NVCC_FLAGS "-gencode arch=compute_${ARCH},code=sm_${ARCH}")
    endforeach(ARCH)
  endif(${PREFIX}_CUDA_ARCHITECTURES)
  nvcc_fix_definitions()

  set(${PREFIX}_CUDA_DEBUG "none" CACHE STRING "Controls the CUDA debug level, regardless of the current build type.")
  set_property(CACHE ${PREFIX}_CUDA_DEBUG PROPERTY STRINGS full lineinfo none)

  if(${PREFIX}_CUDA_DEBUG STREQUAL "full")
    _nvcc_flags_append(CUDA_NVCC_FLAGS "-g -G -O0")
  elseif(${PREFIX}_CUDA_DEBUG STREQUAL "lineinfo")
    _nvcc_flags_append(CUDA_NVCC_FLAGS "-g -G -lineinfo")
  endif(${PREFIX}_CUDA_DEBUG STREQUAL "full")

  # CUDA compiler doesn't seem to respect the CMAKE_POSITION_INDEPENDENT_CODE variable. Explicitly add it for
  # GCC comilation. May be needed for Clang (untested).
  if(CMAKE_POSITION_INDEPENDENT_CODE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")# OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      _nvcc_flags_append(CUDA_NVCC_FLAGS "-Xcompiler -fPIC")
    endif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")# OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  endif(CMAKE_POSITION_INDEPENDENT_CODE)

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
      _nvcc_flags_append(CUDA_NVCC_DEFINITIONS_FIXED "${_definition}")
    endforeach()
    #message("CUDA_NVCC_DEFINITIONS_FIXED: ${CUDA_NVCC_DEFINITIONS_FIXED}")
    set_property(DIRECTORY . PROPERTY COMPILE_DEFINITIONS "${CUDA_NVCC_DEFINITIONS_FIXED}")
    get_directory_property(CUDA_NVCC_DEFINITIONS COMPILE_DEFINITIONS)
    #message("CUDA_NVCC_DEFINITIONS: ${CUDA_NVCC_DEFINITIONS}")
  endif()
endfunction(nvcc_fix_definitions)


macro(_nvcc_flags_append FLAGS_VAR FLAGS)
  if(${FLAGS_VAR})
    set(${FLAGS_VAR} "${${FLAGS_VAR}} ${FLAGS}")
  else(FLAGS_VAR)
    # Nothing set. Add new options.
    set(${FLAGS_VAR} "${FLAGS}")
  endif(${FLAGS_VAR})
endmacro(_nvcc_flags_append)
