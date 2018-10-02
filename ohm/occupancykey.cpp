//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "occupancykey.h"

#include <ohmutil/vectorhash.h>

#define INVALID_VALUE std::numeric_limits<decltype(OccupancyKey::InvalidValue)>::lowest()

using namespace ohm;

const OccupancyKey OccupancyKey::null(glm::ivec3(INVALID_VALUE), 0, 0, 0);
int16_t OccupancyKey::InvalidValue = INVALID_VALUE;

size_t OccupancyKey::Hash::operator()(const OccupancyKey &key) const
{
  glm::u32vec3 hash;
  hash.x = key._regionKey.x | key._regionKey.z << 16;
  hash.y = key._regionKey.y;
  hash.z = key._local[0] | key._local[1] << 8 | key._local[2] << 16;
  return vhash::hashBits(hash.x, hash.y, hash.z);
}

unsigned OccupancyKey::regionHash() const
{
  const glm::u32vec3 rk = _regionKey;
  return vhash::hashBits(rk.x, rk.y, rk.z);
}
