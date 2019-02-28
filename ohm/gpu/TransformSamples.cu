#include <gputil/gpu_ext.h>

#include "TransformSamples.cl"

const void * __host__ transformTimestampedPointsPtr()
{
  return (const void *)&transformTimestampedPoints;
}
