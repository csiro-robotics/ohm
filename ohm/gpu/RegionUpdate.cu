#include <gputil/gpu_ext.h>

#include "RegionUpdate.cl"

GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdate);
GPUTIL_CUDA_DEFINE_KERNEL(regionRayUpdateSubVox);
