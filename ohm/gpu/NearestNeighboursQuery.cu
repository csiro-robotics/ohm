#include <gputil/gpu_ext.h>

#include "NearestNeighboursQuery.cl"

const void * __host__ showNNInfoPtr()
{
  return (const void *)&showNNInfo;
}

const void * __host__ nearestNeighboursPtr()
{
  return (const void *)&nearestNeighbours;
}
