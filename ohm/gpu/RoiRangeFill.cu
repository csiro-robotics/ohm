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

const void *seedRegionVoxelsSubVoxPtr()
{
  return (const void *)&seedRegionVoxelsSubVox;
}

const void *seedFromOuterRegionsSubVoxPtr()
{
  return (const void *)&seedFromOuterRegionsSubVox;
}

const void *propagateObstaclesSubVoxPtr()
{
  return (const void *)&propagateObstaclesSubVox;
}

const void *migrateResultsSubVoxPtr()
{
  return (const void *)&migrateResultsSubVox;
}
