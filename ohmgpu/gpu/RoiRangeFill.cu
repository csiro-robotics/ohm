// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without voxel means
#include "RoiRangeFill.cl"

GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxels);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegions);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstacles);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResults);
