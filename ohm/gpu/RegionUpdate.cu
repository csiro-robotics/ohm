#include <gputil/gpu_ext.h>

#include "RegionUpdate.cl"

const void * __host__ regionRayUpdatePtr()
{
  return (const void *)&regionRayUpdate;
}
