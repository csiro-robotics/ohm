#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base with voxel means
#define VOXEL_MEAN
#include "RegionUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdateSubVox);
