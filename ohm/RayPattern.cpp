// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayPattern.h"

#include "private/RayPatternDetail.h"

#include <glm/ext.hpp>

using namespace ohm;

RayPattern::RayPattern(ohm::RayPatternDetail *detail)
  : imp_(detail)
{}

RayPattern::RayPattern()
  : RayPattern(new ohm::RayPatternDetail)
{}

RayPattern::~RayPattern()
{
  // no-op
}

void RayPattern::addPoints(const glm::dvec3 *points, size_t point_count)
{
  for (size_t pidx = 0; pidx < point_count; ++pidx)
  {
    imp_->points.push_back(points[pidx]);
  }
}

size_t RayPattern::pointCount() const
{
  return imp_->points.size();
}

const glm::dvec3 *RayPattern::points() const
{
  return imp_->points.data();
}

size_t RayPattern::buildRays(std::vector<glm::dvec3> *rays, const glm::dvec3 &position, const glm::dquat &rotation,
                             double scaling) const
{
  rays->clear();
  rays->reserve(pointCount() * 2);
  for (glm::dvec3 ray : imp_->points)
  {
    rays->push_back(position);
    ray = rotation * (scaling * ray) + position;
    rays->push_back(ray);
  }

  return rays->size();
}

size_t RayPattern::buildRays(std::vector<glm::dvec3> *rays, const glm::dmat4 &pattern_transform) const
{
  const glm::dvec3 position = pattern_transform[3];
  rays->clear();
  rays->reserve(pointCount() * 2);
  for (glm::dvec3 ray : imp_->points)
  {
    rays->push_back(position);
    // Apply transformation.
    ray = glm::dvec3(pattern_transform * glm::dvec4(ray, 1.0));
    rays->push_back(ray);
  }

  return rays->size();
}
