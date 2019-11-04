//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#include "Key.h"

#include <ohmutil/VectorHash.h>

#define INVALID_VALUE std::numeric_limits<decltype(Key::kInvalidValue)>::lowest()

using namespace ohm;

const int16_t Key::kInvalidValue = INVALID_VALUE;
const Key Key::kNull(glm::ivec3(INVALID_VALUE), 0, 0, 0);  // NOLINT

size_t Key::Hash::operator()(const Key &key) const
{
  glm::u32vec3 hash;
  hash.x = key.region_key_.x | key.region_key_.z << 16;
  hash.y = key.region_key_.y;
  hash.z = key.local_[0] | key.local_[1] << 8 | key.local_[2] << 16;
  return vhash::hashBits(hash.x, hash.y, hash.z);
}

unsigned Key::regionHash() const
{
  const glm::u32vec3 rk = region_key_;
  return vhash::hashBits(rk.x, rk.y, rk.z);
}


void Key::clampToAxis(int axis, const Key &min_bounds, const Key &max_bounds)
{
  if (region_key_[axis] < min_bounds.region_key_[axis])
  {
    // Below lower bounds region. Clamp region and local.
    region_key_[axis] = min_bounds.region_key_[axis];
    local_[axis] = min_bounds.local_[axis];
  }
  else if (region_key_[axis] == min_bounds.region_key_[axis] && local_[axis] < min_bounds.local_[axis])
  {
    // Below lower bounds local. Clamp local.
    local_[axis] = min_bounds.local_[axis];
  }

  if (region_key_[axis] > max_bounds.region_key_[axis])
  {
    // Above upper bounds region. Clamp region and local.
    region_key_[axis] = max_bounds.region_key_[axis];
    local_[axis] = max_bounds.local_[axis];
  }
  else if (region_key_[axis] == max_bounds.region_key_[axis] && local_[axis] > max_bounds.local_[axis])
  {
    // Above upper bounds local. Clamp local.
    local_[axis] = max_bounds.local_[axis];
  }
}
