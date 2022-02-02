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
  is_good = is_good && !glm::any(glm::isnan(start)) && !glm::any(glm::isinf(start));
  is_good = is_good && !glm::any(glm::isnan(end)) && !glm::any(glm::isinf(end));

  const glm::dvec3 ray = end - start;
  is_good = is_good && (max_range <= 0 || glm::dot(ray, ray) <= max_range * max_range);

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


bool clipRayFilter(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags, double max_length)
{
  bool is_good = true;
  is_good = is_good && !glm::any(glm::isnan(*start)) && !glm::any(glm::isinf(*start));
  is_good = is_good && !glm::any(glm::isnan(*end)) && !glm::any(glm::isinf(*end));

  glm::dvec3 ray = *end - *start;
  const double ray_length_sqr = glm::dot(ray, ray);
  if (is_good && max_length > 0 && ray_length_sqr > max_length * max_length)
  {
    // Ray is good, but too long. Clip it.
    // Normalise ray.
    ray /= std::sqrt(ray_length_sqr);
    // Clip and mark
    *end = *start + ray * max_length;
    *filter_flags |= kRffClippedEnd;
  }

  *filter_flags |= !is_good * kRffInvalid;
  return is_good;
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


  // Lint(KS): everything is unsigned.
  *filter_flags |= !!(line_clip_flags & Aabb::kClippedStart) * kRffClippedStart;  // NOLINT(hicpp-signed-bitwise)
  *filter_flags |= !!(line_clip_flags & Aabb::kClippedEnd) * kRffClippedEnd;      // NOLINT(hicpp-signed-bitwise)

  return true;
}


bool clipToBounds(glm::dvec3 * /*start*/, glm::dvec3 *end, unsigned *filter_flags, const ohm::Aabb &clip_box)
{
  const bool clipped = clip_box.contains(*end);
  // Lint(KS): everything is unsigned.
  *filter_flags |= !!(clipped)*kRffClippedEnd;  // NOLINT(hicpp-signed-bitwise)
  return true;
}
}  // namespace ohm
