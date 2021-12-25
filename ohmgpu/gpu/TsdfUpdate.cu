// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without voxel means
#include "TsdfUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(tsdfRayUpdate);
