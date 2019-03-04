#include <gputil/gpu_ext.h>

#include "RoiRangeFill.cl"

GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxels);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegions);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstacles);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResults);
GPUTIL_CUDA_DEFINE_KERNEL(seedRegionVoxelsSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(seedFromOuterRegionsSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(propagateObstaclesSubVox);
GPUTIL_CUDA_DEFINE_KERNEL(migrateResultsSubVox);
