#include <gputil/gpu_ext.h>

#include "RegionUpdate.cl"

const void *regionRayUpdatePtr()
{
  return (const void *)&regionRayUpdate;
}

const void *regionRayUpdateSubVoxPtr()
{
  return (const void *)&regionRayUpdateSubVox;
}
