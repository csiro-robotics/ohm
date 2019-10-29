// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PlaneWalker.h"

#include "ohm/Key.h"
#include "ohm/OccupancyMap.h"

using namespace ohm;

PlaneWalker::PlaneWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis)
  : map(map)
  , min_ext_key(min_ext_key)
  , max_ext_key(max_ext_key)
{
  switch (up_axis)
  {
  case UpAxis::kX:
    /* fallthrough */
  case UpAxis::kNegX:
    axis_indices[0] = 1;
    axis_indices[1] = 2;
    axis_indices[2] = 0;
    break;
  case UpAxis::kY:
    /* fallthrough */
  case UpAxis::kNegY:
    axis_indices[0] = 0;
    axis_indices[1] = 2;
    axis_indices[2] = 1;
    break;
  case UpAxis::kZ:
    /* fallthrough */
  case UpAxis::kNegZ:
    axis_indices[0] = 0;
    axis_indices[1] = 1;
    axis_indices[2] = 2;
    break;
  }
}


void PlaneWalker::begin(Key &key) const  // NOLINT(google-runtime-references)
{
  key = min_ext_key;
  key.setRegionAxis(axis_indices[2], 0);
  key.setLocalAxis(axis_indices[2], 0);
}


bool PlaneWalker::walkNext(Key &key) const  // NOLINT(google-runtime-references)
{
  map.stepKey(key, axis_indices[0], 1);
  if (!key.isBounded(axis_indices[0], min_ext_key, max_ext_key))
  {
    // Finished walking this axis. Reset and walk outer axis.
    key.setLocalAxis(axis_indices[0], min_ext_key.localKey()[axis_indices[0]]);
    key.setRegionAxis(axis_indices[0], min_ext_key.regionKey()[axis_indices[0]]);

    map.stepKey(key, axis_indices[1], 1);
    if (!key.isBounded(axis_indices[1], min_ext_key, max_ext_key))
    {
      return false;
    }
  }

  return true;
}
