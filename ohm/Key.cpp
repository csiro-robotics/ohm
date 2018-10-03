//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Key.h"

#include <ohmutil/VectorHash.h>

#define INVALID_VALUE std::numeric_limits<decltype(OccupancyKey::kInvalidValue)>::lowest()

using namespace ohm;

const OccupancyKey OccupancyKey::kNull(glm::ivec3(INVALID_VALUE), 0, 0, 0);
const int16_t OccupancyKey::kInvalidValue = INVALID_VALUE;

size_t OccupancyKey::Hash::operator()(const OccupancyKey &key) const
{
  glm::u32vec3 hash;
  hash.x = key.region_key_.x | key.region_key_.z << 16;
  hash.y = key.region_key_.y;
  hash.z = key.local_[0] | key.local_[1] << 8 | key.local_[2] << 16;
  return vhash::hashBits(hash.x, hash.y, hash.z);
}

unsigned OccupancyKey::regionHash() const
{
  const glm::u32vec3 rk = region_key_;
  return vhash::hashBits(rk.x, rk.y, rk.z);
}
