#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without sub-voxels
#include "RoiRangeFill.cl"

// Build base with sub-voxels
#define SUB_VOXEL
#include "RoiRangeFill.cl"

GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxels);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegions);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstacles);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResults);
GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxelsSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegionsSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstaclesSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResultsSubVox);
