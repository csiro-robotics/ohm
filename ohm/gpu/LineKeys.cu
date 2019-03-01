#include <gputil/gpu_ext.h>

#include "LineKeys.cl"

const void *calculateLinesPtr()
{
  return (const void *)&calculateLines;
}
