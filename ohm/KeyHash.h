// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef KEYHASH_H
#define KEYHASH_H

#include "OhmConfig.h"

#include "Key.h"

#include <ohmutil/VectorHash.h>

namespace ohm
{
/// Hashing function for @c Key objects.
struct ohm_API KeyHash
{
  /// Hash operator.
  /// @param key The key to hash.
  /// @return A 32-bit hash value for @p key.
  inline uint32_t operator()(const Key &key) const
  {
    const auto &region = key.regionKey();
    const auto &local = key.localKey();
    return vhash::hashBits(region.x | (local.x << 16), region.y | (local.y << 16), region.z | (local.z << 16));
  }
};
}  // namespace ohm

#endif  // KEYHASH_H
