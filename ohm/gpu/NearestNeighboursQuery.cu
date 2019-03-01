#include <gputil/gpu_ext.h>

#include "NearestNeighboursQuery.cl"

const void *showNNInfoPtr()
{
  return (const void *)&showNNInfo;
}

const void *nearestNeighboursPtr()
{
  return (const void *)&nearestNeighbours;
}
