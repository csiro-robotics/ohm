// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef MAPFLAG_H
#define MAPFLAG_H

#include "OhmConfig.h"

#include <type_traits>

namespace ohm
{
  /// Flags used to augment initialisation of an @c OccupancyMap.
  enum class MapFlag : unsigned
  {
    /// No special features.
    None = 0u,
    /// Enable sub-voxel positioning.
    SubVoxelPosition = (1u << 0),
    /// Use sub-voxel positioning to augment occupancy probability. Occupied voxels with points falling towards the
    /// outer extents of a voxel are considered free, not occupied. This is essentially an outlier or noise reduction
    /// inference.
    /// @see @c subVoxelOccupancyFilter()
    SubVoxelOccupancy = (1u << 1),

    /// Full sub-voxel features: positioning and occupancy augmentation.
    SubVoxel = SubVoxelPosition | SubVoxelOccupancy
  };

  const char *mapFlagToString(MapFlag flag);
  MapFlag mapFlagFromString(const char *str);
}

inline ohm::MapFlag operator | (ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) | static_cast<T>(right));
}

inline ohm::MapFlag &operator |= (ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left | right;
  return left;
}

inline ohm::MapFlag operator & (ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) & static_cast<T>(right));
}

inline ohm::MapFlag &operator &= (ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left & right;
  return left;
}

inline ohm::MapFlag operator ^ (ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) ^ static_cast<T>(right));
}

inline ohm::MapFlag &operator ^= (ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left ^ right;
  return left;
}

inline ohm::MapFlag operator ~ (ohm::MapFlag value)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(~static_cast<T>(value));
}

#endif // MAPFLAG_H
