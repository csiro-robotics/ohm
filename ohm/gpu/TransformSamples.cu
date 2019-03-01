#include <gputil/gpu_ext.h>

#include "TransformSamples.cl"

const void *transformTimestampedPointsPtr()
{
  return (const void *)&transformTimestampedPoints;
}
