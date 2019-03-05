#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

#include "NearestNeighboursQuery.cl"

#include <functional>

GPUTIL_CUDA_DEFINE_KERNEL(showNNInfo);
GPUTIL_CUDA_DEFINE_KERNEL(nearestNeighbours);
