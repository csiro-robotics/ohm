#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

#include "TransformSamples.cl"

GPUTIL_CUDA_DEFINE_KERNEL(transformTimestampedPoints);
