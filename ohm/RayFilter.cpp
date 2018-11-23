// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayFilter.h"

#include <Aabb.h>

namespace ohm
{
  bool goodRay(const glm::dvec3 &start, const glm::dvec3 &end, double max_range)
  {
    bool is_good = true;
    is_good = is_good && !glm::any(glm::isnan(start));
    is_good = is_good && !glm::any(glm::isnan(end));

    const glm::dvec3 ray = end - start;
    is_good = is_good && (max_range <= 0 || glm::dot(ray, ray) > max_range * max_range);

    return is_good;
  }


  bool goodRayFilter(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, double max_range)
  {
    if (goodRay(*start, *end, max_range))
    {
      return true;
    }

    *filter_flags |= kRffInvalid;
    return false;
  }


  bool clipBounded(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, const ohm::Aabb &clip_box)
  {
    unsigned line_clip_flags = 0;
    if (clip_box.clipLine(*start, *end, &line_clip_flags))
    {
      if (!clip_box.contains(*start) && !clip_box.contains(*end))
      {
        return false;
      }
    }


    *filter_flags |= !!(line_clip_flags & Aabb::kClippedStart) * kRffClippedStart;
    *filter_flags |= !!(line_clip_flags & Aabb::kClippedEnd) * kRffClippedEnd;

    return true;
  }


  bool clipNear(glm::dvec3 * /*start*/, glm::dvec3 *end, unsigned *filter_flags, const ohm::Aabb &clip_box)
  {
    const bool clipped = clip_box.contains(*end);
    *filter_flags |= !!(clipped) * kRffClippedEnd;
    return true;
  }
} // namespace ohm
