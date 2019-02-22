#include <gputil/cuda/cutil_importcl.h>
#include <gputil/cuda/cutil_math.h>

#include "matrix.cl"

const void * __host__ matrixMultiplyPtr()
{
  return (const void *)&matrixMultiply;
}
