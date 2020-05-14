#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without voxel means
#include "RegionUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdate);
