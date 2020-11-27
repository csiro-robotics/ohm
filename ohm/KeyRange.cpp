// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "KeyRange.h"

#include "OccupancyMap.h"
#include "private/OccupancyMapDetail.h"

namespace ohm
{
KeyRange::KeyRange(const Key &min_key, const Key &max_key, const ohm::OccupancyMap &map)
  : KeyRange(min_key, max_key, map.regionVoxelDimensions())
{}

void KeyRange::expand(const Key &key)
{
  // First handle null keys.
  if (key.isNull())
  {
    // New key is null. Cannot expand.
    return;
  }

  if (min_key_.isNull())
  {
    // Min is null. Set to key.
    min_key_ = key;
  }

  if (max_key_.isNull())
  {
    // Max is null. Set to key.
    max_key_ = key;
  }

  // Build ranges from the min and max respectively to the new key.
  const glm::ivec3 min_to_key = OccupancyMap::rangeBetween(min_key_, key, region_dimensions_);
  const glm::ivec3 max_to_key = OccupancyMap::rangeBetween(max_key_, key, region_dimensions_);
  // With min_to_key and max_to_key set, we can ascertain which axes need expanding.
  // Each axis of min_to_key which is negative indicates that the new key has a lower bound than min_key_.
  // Likewise, each axis of max_to_key which is positive indicates that the new key has a higher bound than max_key_.

  for (int i = 0; i < 3; ++i)
  {
    if (min_to_key[i] < 0)
    {
      min_key_.setRegionAxis(i, key.regionKey()[i]);
      min_key_.setLocalAxis(i, key.localKey()[i]);
    }
    if (max_to_key[i] > 0)
    {
      max_key_.setRegionAxis(i, key.regionKey()[i]);
      max_key_.setLocalAxis(i, key.localKey()[i]);
    }
  }
}

glm::ivec3 KeyRange::range() const
{
  // The range defines a closed interval, so we have to add 1 in each dimension for the volume to ensure the maximal
  // extents are consdiered.
  const glm::ivec3 volume_correction = glm::ivec3(1);
  return ohm::OccupancyMap::rangeBetween(min_key_, max_key_, region_dimensions_) + volume_correction;
}


size_t KeyRange::indexOf(const Key &key) const
{
  if (!isValid())
  {
    return 0;
  }

  const glm::ivec3 range = this->range();
  const glm::ivec3 range_to_key = ohm::OccupancyMap::rangeBetween(min_key_, key, region_dimensions_);
  return size_t(range_to_key.x) + size_t(range_to_key.y * range.x) + size_t(range_to_key.z * range.x * range.y);
}


Key KeyRange::fromIndex(size_t index) const
{
  if (!isValid())
  {
    return Key::kNull;
  }

  const glm::ivec3 range = this->range();
  glm::ivec3 offset(0);
  offset.z = index / (range.x * range.y);
  offset.y = index % (range.x * range.y);
  offset.x = offset.y % range.x;
  offset.y /= range.x;

  Key key = min_key_;
  OccupancyMapDetail::moveKeyAlongAxis(key, 0, offset.x, region_dimensions_);
  OccupancyMapDetail::moveKeyAlongAxis(key, 1, offset.y, region_dimensions_);
  OccupancyMapDetail::moveKeyAlongAxis(key, 2, offset.z, region_dimensions_);

  return key;
}


void KeyRange::walkNext(Key &key, const Key &min_key, const Key &max_key, const glm::ivec3 &region_dimensions)
{
  // Try walk X axis.
  if (!key.equalOnAxis(max_key, 0))
  {
    OccupancyMapDetail::moveKeyAlongAxis(key, 0, 1, region_dimensions);
    return;
  }

  // At X limit. Reset X axis to min and try walk Y.
  key.setRegionAxis(0, min_key.regionKey()[0]);
  key.setLocalAxis(0, min_key.localKey()[0]);

  if (!key.equalOnAxis(max_key, 1))
  {
    OccupancyMapDetail::moveKeyAlongAxis(key, 1, 1, region_dimensions);
    return;
  }

  // At Y limit. Reset Y axis to min and try walk Z.
  key.setRegionAxis(1, min_key.regionKey()[1]);
  key.setLocalAxis(1, min_key.localKey()[1]);

  OccupancyMapDetail::moveKeyAlongAxis(key, 2, 1, region_dimensions);
}


void KeyRange::updateEndKey()
{
  end_key_ = min_key_;
  // Last axis which gets walked. We need to set this to the max key, then walk that axis on 1 as this is the pattern
  // iteration takes.
  const unsigned last_axis = 2;
  end_key_.setRegionAxis(last_axis, max_key_.regionKey()[last_axis]);
  end_key_.setLocalAxis(last_axis, max_key_.localKey()[last_axis]);
  OccupancyMapDetail::moveKeyAlongAxis(end_key_, last_axis, 1, region_dimensions_);
}

}  // namespace ohm
