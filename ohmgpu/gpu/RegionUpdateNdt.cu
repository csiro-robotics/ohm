#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base with voxel means and NDT
#define VOXEL_MEAN
#define NDT
#include "RegionUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdateNdt);
