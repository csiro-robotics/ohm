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
  kNone = 0u,
  /// Enable voxel mean position tracking.
  kVoxelMean = (1u << 0u),
  /// Maintain compressed voxels in memory. Compression is performed off thread.
  kCompressed = (1u << 1u),
  /// Maintain the decay rate in addition to the occupancy layer. See @c default_layer::decayRateLayerName() .
  /// This implies @c kVoxelMean layer is also used in order to track the voxel sample count.
  kDecayRate = (1u << 2u),

  /// Default map creation flags.
  kDefault = kCompressed
};

const char *mapFlagToString(MapFlag flag);
MapFlag mapFlagFromString(const char *str);
}  // namespace ohm

inline ohm::MapFlag operator|(ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) | static_cast<T>(right));
}

inline ohm::MapFlag &operator|=(ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left | right;
  return left;
}

inline ohm::MapFlag operator&(ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) & static_cast<T>(right));
}

inline ohm::MapFlag &operator&=(ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left & right;
  return left;
}

inline ohm::MapFlag operator^(ohm::MapFlag left, ohm::MapFlag right)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(static_cast<T>(left) ^ static_cast<T>(right));
}

inline ohm::MapFlag &operator^=(ohm::MapFlag &left, ohm::MapFlag right)
{
  left = left ^ right;
  return left;
}

inline ohm::MapFlag operator~(ohm::MapFlag value)
{
  using T = std::underlying_type_t<ohm::MapFlag>;
  return static_cast<ohm::MapFlag>(~static_cast<T>(value));
}

#endif  // MAPFLAG_H
