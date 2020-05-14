#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without voxel means
#include "RoiRangeFill.cl"

GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxels);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegions);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstacles);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResults);
