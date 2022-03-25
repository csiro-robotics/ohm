// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ClearingPattern.h"
#include "private/ClearingPatternDetail.h"

#include "OccupancyMap.h"
#include "RayPattern.h"

namespace ohm
{
ClearingPattern::ClearingPattern(const RayPattern *pattern, bool take_ownership)
  : imp_(new ClearingPatternDetail)
{
  imp_->pattern = pattern;
  imp_->has_pattern_ownership = take_ownership;
  imp_->ray_flags = kDefaultRayFlags;
}

ClearingPattern::~ClearingPattern()
{
  if (imp_->has_pattern_ownership)
  {
    delete imp_->pattern;
  }
}

const RayPattern *ClearingPattern::pattern() const
{
  return imp_->pattern;
}

bool ClearingPattern::hasPatternOwnership() const
{
  return imp_->has_pattern_ownership;
}

unsigned ClearingPattern::rayFlags() const
{
  // Must not use kRfReverseWalk for clearing patterns and ray queries or we cannot stop on the first obstruction.
  return imp_->ray_flags & ~kRfReverseWalk;
}

void ClearingPattern::setRayFlags(unsigned ray_flags)
{
  imp_->ray_flags = ray_flags;
}

const glm::dvec3 *ClearingPattern::lastRaySet(size_t *element_count) const
{
  *element_count = imp_->ray_set.size();
  return imp_->ray_set.data();
}


const glm::dvec3 *ClearingPattern::buildRaySet(size_t *element_count, const glm::dvec3 &position,
                                               const glm::dquat &rotation)
{
  imp_->pattern->buildRays(&imp_->ray_set, position, rotation);
  *element_count = imp_->ray_set.size();
  return imp_->ray_set.data();
}


const glm::dvec3 *ClearingPattern::buildRaySet(size_t *element_count, const glm::dmat4 &pattern_transform)
{
  imp_->pattern->buildRays(&imp_->ray_set, pattern_transform);
  *element_count = imp_->ray_set.size();
  return imp_->ray_set.data();
}
}  // namespace ohm
