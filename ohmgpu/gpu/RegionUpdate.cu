#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

// Build base without sub-voxels
#include "RegionUpdate.cl"

// Build base with sub-voxels
#define SUB_VOXEL
#include "RegionUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdate);
GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdateSubVox);
