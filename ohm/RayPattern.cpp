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

RayPattern::~RayPattern() = default;

void RayPattern::addPoints(const glm::dvec3 *points, size_t point_count)
{
  for (size_t pidx = 0; pidx < point_count; ++pidx)
  {
    imp_->sample_pairs.emplace_back(glm::dvec3(0.0));
    imp_->sample_pairs.push_back(points[pidx]);
  }
}

void RayPattern::addRays(const glm::dvec3 *ray_pairs, size_t elements)
{
  for (size_t ridx = 0; ridx + 1 < elements; ridx += 2)
  {
    imp_->sample_pairs.push_back(ray_pairs[ridx + 0]);
    imp_->sample_pairs.push_back(ray_pairs[ridx + 1]);
  }
}

void RayPattern::addRay(const glm::dvec3 &ray_start, const glm::dvec3 &ray_end)
{
  imp_->sample_pairs.push_back(ray_start);
  imp_->sample_pairs.push_back(ray_end);
}

size_t RayPattern::rayCount() const
{
  return imp_->sample_pairs.size() / 2;
}

const glm::dvec3 *RayPattern::rayPoints() const
{
  return imp_->sample_pairs.data();
}

size_t RayPattern::buildRays(std::vector<glm::dvec3> *rays, const glm::dvec3 &position, const glm::dquat &rotation,
                             double scaling) const
{
  rays->clear();
  rays->reserve(rayCount());
  for (glm::dvec3 sample : imp_->sample_pairs)
  {
    sample = rotation * (scaling * sample) + position;
    rays->push_back(sample);
  }

  return rays->size();
}

size_t RayPattern::buildRays(std::vector<glm::dvec3> *rays, const glm::dmat4 &pattern_transform) const
{
  rays->clear();
  rays->reserve(rayCount());
  for (glm::dvec3 sample : imp_->sample_pairs)
  {
    // Apply transformation.
    sample = glm::dvec3(pattern_transform * glm::dvec4(sample, 1.0));
    rays->push_back(sample);
  }

  return rays->size();
}
