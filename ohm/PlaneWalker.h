// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEWALKER_H
#define OHM_PLANEWALKER_H

#include "OhmConfig.h"

#include "ohm/UpAxis.h"

namespace ohm
{
  class Key;
  class OccupancyMap;

  /// Helper class for walking a plane in the heightmap given any up axis.
  /// Manages walking the correct axis based on the @c UpAxis.
  ///
  /// Usage:
  /// - Initialise
  /// - call @c begin().
  /// - do work
  /// - call @c walkNext() and loop if true.
  class ohm_API PlaneWalker
  {
  public:
    const OccupancyMap &map;
    const Key &min_ext_key, &max_ext_key;
    int axis_indices[3] = { 0, 0, 0 };

    PlaneWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis);

    void begin(Key &key) const;  // NOLINT(google-runtime-references)

    bool walkNext(Key &key) const;  // NOLINT(google-runtime-references)
  };
}  // namespace ohm

#endif // OHM_PLANEWALKER_H
