#include <gputil/cuda/cutil_importcl.h>
#include <gputil/cuda/cutil_math.h>

#include "matrix.cl"

const void * matrixMultiplyPtr()
{
  return (const void *)&matrixMultiply;
}
