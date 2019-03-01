#include <gputil/gpu_ext.h>

#include "RoiRangeFill.cl"

const void *seedRegionVoxelsPtr()
{
  return (const void *)&seedRegionVoxels;
}

const void *seedFromOuterRegionsPtr()
{
  return (const void *)&seedFromOuterRegions;
}

const void *propagateObstaclesPtr()
{
  return (const void *)&propagateObstacles;
}

const void *migrateResultsPtr()
{
  return (const void *)&migrateResults;
}
