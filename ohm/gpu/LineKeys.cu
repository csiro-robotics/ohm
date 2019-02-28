#include <gputil/gpu_ext.h>

#include "LineKeys.cl"

const void * __host__ calculateLinesPtr()
{
  return (const void *)&calculateLines;
}
