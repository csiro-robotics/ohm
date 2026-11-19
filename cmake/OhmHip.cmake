# Build the CUDA backend with HIP for AMD GPUs. The CUDA sources, targets and options are reused: OHM_FEATURE_CUDA is
# set so the CUDA targets are configured, the .cu files compile as HIP, and gputil/cuda/hip_compat supplies the CUDA
# headers they include.
if(CMAKE_VERSION VERSION_LESS 3.21)
  message(FATAL_ERROR "OHM_FEATURE_HIP requires CMake 3.21 or later.")
endif(CMAKE_VERSION VERSION_LESS 3.21)

set(OHM_FEATURE_CUDA ON)
set(OHM_USE_DEPRECATED_CMAKE_CUDA OFF)

if(WIN32)
  # HIP on Windows uses the GNU style clang++ for every language, which does not define WIN32 as MSVC builds do.
  add_compile_definitions(WIN32)

  # CMake detects the GPU architecture with rocm_agent_enumerator, which ROCm does not ship on Windows, and otherwise
  # silently falls back to a default architecture. Detect the GPUs present with amdgpu-arch instead.
  if(NOT DEFINED CMAKE_HIP_ARCHITECTURES AND NOT DEFINED ENV{HIPARCHS})
    set(_hip_hints)
    if(CMAKE_HIP_COMPILER)
      get_filename_component(_hip_compiler_dir "${CMAKE_HIP_COMPILER}" DIRECTORY)
      list(APPEND _hip_hints "${_hip_compiler_dir}")
    endif(CMAKE_HIP_COMPILER)
    if(DEFINED ENV{HIP_PATH})
      list(APPEND _hip_hints "$ENV{HIP_PATH}/bin" "$ENV{HIP_PATH}/lib/llvm/bin")
    endif(DEFINED ENV{HIP_PATH})
    find_program(OHM_AMDGPU_ARCH amdgpu-arch HINTS ${_hip_hints})
    if(OHM_AMDGPU_ARCH)
      execute_process(COMMAND "${OHM_AMDGPU_ARCH}" RESULT_VARIABLE _hip_arch_result OUTPUT_VARIABLE _hip_archs
                      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
      if(_hip_arch_result EQUAL 0 AND _hip_archs)
        string(REGEX REPLACE "[\r\n]+" ";" _hip_archs "${_hip_archs}")
        list(REMOVE_DUPLICATES _hip_archs)
        set(CMAKE_HIP_ARCHITECTURES "${_hip_archs}" CACHE STRING "GPU architectures to build HIP code for.")
      endif(_hip_arch_result EQUAL 0 AND _hip_archs)
    endif(OHM_AMDGPU_ARCH)
    if(NOT DEFINED CMAKE_HIP_ARCHITECTURES)
      message(WARNING "No AMD GPU detected. Set CMAKE_HIP_ARCHITECTURES to select the GPU architecture to build for.")
    endif(NOT DEFINED CMAKE_HIP_ARCHITECTURES)
  endif(NOT DEFINED CMAKE_HIP_ARCHITECTURES AND NOT DEFINED ENV{HIPARCHS})
endif(WIN32)

# Compile the .cu files in the given list as HIP. Other files in the list are left alone.
function(ohm_hip_sources)
  foreach(_source ${ARGN})
    if(_source MATCHES "\\.cu$")
      set_source_files_properties(${_source} PROPERTIES LANGUAGE HIP)
    endif()
  endforeach(_source)
endfunction(ohm_hip_sources)

macro(cuda_setup)
  # Use CMAKE_HIP_ARCHITECTURES to select the GPU targets. When unset, enable_language(HIP) detects the GPUs present.
  enable_language(HIP)
  set(CMAKE_HIP_STANDARD ${CMAKE_CXX_STANDARD})
  set(CMAKE_HIP_STANDARD_REQUIRED ${CMAKE_CXX_STANDARD_REQUIRED})
  # nvcc implicitly includes cuda_runtime.h in every .cu file. Do the same, resolving to gputil/cuda/hip_compat.
  add_compile_options("$<$<COMPILE_LANGUAGE:HIP>:SHELL:-include cuda_runtime.h>")
  find_package(hip REQUIRED)
  set(OHM_CUDA_LIBRARY hip::host)
endmacro(cuda_setup)
