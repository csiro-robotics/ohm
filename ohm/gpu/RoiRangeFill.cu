#include <gputil/gpu_ext.h>

#include "RoiRangeFill.cl"

const void * __host__ seedRegionVoxelsPtr()
{
  return (const void *)&seedRegionVoxels;
}

const void * __host__ seedFromOuterRegionsPtr()
{
  return (const void *)&seedFromOuterRegions;
}

const void * __host__ propagateObstaclesPtr()
{
  return (const void *)&propagateObstacles;
}

const void * __host__ migrateResultsPtr()
{
  return (const void *)&migrateResults;
}
