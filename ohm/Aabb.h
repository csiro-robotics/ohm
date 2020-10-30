// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_AABB_H
#define OHM_AABB_H

#include "OhmConfig.h"

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <algorithm>

namespace ohm
{
/// An axis aligned box
class ohm_API Aabb
{
public:
  /// Empty constructor: uninitialised.
  Aabb();

  /// Create an initialised AABB using the given @p seed scalar.
  ///
  /// The resulting box has all channels set to @p seed. This box will not pass @c isValid(), but is initialised.
  /// Typically, use this constructor to ensure an initialised, but unpopulated box calling @c Aabb(0.0).
  ///
  /// @param seed The seed value for all channels.
  Aabb(double seed);

  /// Copy constructor.
  /// @param other Value to copy.
  Aabb(const Aabb &other);
  /// Initialise with the given min/max extents.
  /// @param min_ext Minimum extents. Must be <= @p max_ext.
  /// @param max_ext Maximum extents.
  Aabb(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext);

  /// Initialise from @p centre and @p half_extents.
  ///
  /// Box is calculated as:
  /// @code
  ///   min_ext = centre - half_extents;
  ///   max_ext = centre + half_extents;
  /// @endcode
  ///
  /// @param centre The box centre.
  /// @param half_extents The half extents for the box. All elements must be >= 0.
  static Aabb fromCentreHalfExtents(const glm::dvec3 &centre, const glm::dvec3 &half_extents);

  /// Initialise from @p centre and @p full_extents.
  ///
  /// Box is calculated as:
  /// @code
  ///   min_ext = centre - 0.5 * full_extents;
  ///   max_ext = centre + 0.5 * full_extents;
  /// @endcode
  ///
  /// @param centre The box centre.
  /// @param full_extents The extents (diagonal) for the box. All elements must be >= 0.
  static Aabb fromCentreFullExtents(const glm::dvec3 &centre, const glm::dvec3 &full_extents);

  /// Set the box lower extents corner.
  /// @param ext New coordinate for the lower box extents. No validation (for being <= @c maxExtents).
  inline void setMinExtents(const glm::dvec3 &ext) { corners_[0] = ext; }

  /// Set the box upper extents corner.
  /// @param ext New coordinate for the upper box extents. No validation (for being >= @c minExtents).
  inline void setMaxExtents(const glm::dvec3 &ext) { corners_[1] = ext; }

  /// Query the lower box extents.
  /// @return The lower bounds of the box.
  inline glm::dvec3 minExtents() const { return corners_[0]; }

  /// Access a mutable reference to the minimum extents. Incorrect usage may result in undefined behaviour.
  /// @return A reference to the lower bounds.
  inline glm::dvec3 &minExtentsMutable() { return corners_[0]; }

  /// Query the upper box extents.
  /// @return The upper bounds of the box.
  inline glm::dvec3 maxExtents() const { return corners_[1]; }

  /// Access a mutable reference to the maximum extents. Incorrect usage may result in undefined behaviour.
  /// @return A reference to the upper bounds.
  inline glm::dvec3 &maxExtentsMutable() { return corners_[1]; }

  /// Query the box centre.
  /// @return The box centre.
  inline glm::dvec3 centre() const { return 0.5 * (corners_[0] + corners_[1]); }

  /// Query the box half extents.
  /// @return The box half extents (half the diagonal).
  inline glm::dvec3 halfExtents() const { return 0.5 * (corners_[1] - corners_[0]); }

  /// Query the full box extents.
  /// @return The full extents (the diagonal).
  inline glm::dvec3 diagonal() const { return corners_[1] - corners_[0]; }

  /// Corner indexing enum. See @c corner()
  /// Corners are defined by a three letter acronym, each letter referencing a axis in XYZ order.
  /// Each letter may be either L/l for lower bound (minimum extent) or U/u for upper bound (maximum extent).
  enum CornerIndex : int
  {
    kCornerLll,
    kCornerUll,
    kCornerLul,
    kCornerUul,
    kCornerLlu,
    kCornerUlu,
    kCornerLuu,
    kCornerUuu,
    kCornerCount
  };

  /// Return a corner point of the box.
  ///
  /// The corner indexing is defined by @c CornerIndex.
  /// @param corner_index The index of the corner of interest [0, 8).
  /// @return The coordinates of the requested corner.
  glm::dvec3 corner(int corner_index) const;

  /// Expand the extents to include @p pt. Does not handle the first point of a point set.
  /// @param pt The point to expand the extents to include.
  void expand(const glm::dvec3 &pt);

  /// Test whether this box and @p other overlap. Touching is considered an overlap.
  ///
  /// The box is optionally extended by @p epsilon.
  ///
  /// @param other The box to test.
  /// @param epsilon Error tolerance value.
  /// @return True if the boxes overlap.
  bool overlaps(const Aabb &other, double epsilon = 0) const;

  /// Test whether this box contains the point @p point. Touching is containing.
  ///
  /// The box is optionally extended by @p epsilon.
  ///
  /// @param point The point to test.
  /// @param epsilon Error tolerance value.
  /// @return True @p point is in this box.
  bool contains(const glm::dvec3 &point, double epsilon = 0) const;

  /// Test for equality between this and @p other with tolerance @p epsilon.
  /// @param other The box to test against.
  /// @param epsilon Error tolerance value.
  /// @return True if this and @p other are within @p epsilon of equality (per axis error).
  bool isEqual(const Aabb &other, double epsilon) const;

  /// Is this a valid AABB where the minimum corner is less than the maximum.
  /// @return True if this is a valid, non-zero sized box.
  bool isValid() const;

  /// Flags for clipping results.
  enum ClipResult
  {
    kClippedStart = (1 << 0),
    kClippedEnd = (1 << 1)
  };

  /// Clip the line from @p start to @p end such that both points lie in or on the box.
  /// @param[in,out] start The line start point.
  /// @param[in,out] end The line end point.
  /// @param[out] clip_flags Clipping flags indicating how the line has been clipped. See : @c ClipResult.
  /// @return True when the line segment intersects the box and has been clipped. Essentially, false when @p start
  ///     and @p end are unmodified.
  bool clipLine(glm::dvec3 &start, glm::dvec3 &end,  // NOLINT(google-runtime-references)
                unsigned *clip_flags = nullptr) const;

  /// Test for precise quality between this and @p other.
  /// @param other The box to test against.
  /// @return True on exact equality.
  bool operator==(const Aabb &other) const;

  /// Test whether and @p other are not presicely equal.
  /// @param other The box to test against.
  /// @return True if the boxes are not exactly equality.
  bool operator!=(const Aabb &other) const;

  /// Assignment operator.
  /// @param other The value to assign.
  /// @return *this
  Aabb &operator=(const Aabb &other);

  /// Translate the box by @p offset.
  /// @param offset Translation to apply.
  /// @return *this
  Aabb &operator+=(const glm::dvec3 &offset);

  /// Translate the box by @p -offset.
  /// @param offset Inverse of the translation to apply.
  /// @return *this
  Aabb &operator-=(const glm::dvec3 &offset);

  /// Scale box by @p scalar.
  /// @param scalar Scaling factor.
  /// @return *this
  Aabb &operator*=(double scalar);

  /// Scale box by the inverse of @p scalar.
  /// @param scalar Inverse scaling factor. Must not be zero.
  /// @return *this
  Aabb &operator/=(double scalar);

private:
  static inline double sign(double val) { return val >= 0 ? 1.0f : -1.0f; }
  static double calcTimeVal(double limit, double origin, double direction);
  static bool calcIntervalOverlap(const glm::dvec2 &a, const glm::dvec2 &b, glm::dvec2 *overlap);

  glm::dvec3 corners_[2];  // = { glm::dvec3(0), glm::dvec3(0) };
};


inline Aabb::Aabb() = default;  // NOLINT(cppcoreguidelines-pro-type-member-init)


inline Aabb::Aabb(double seed)  // NOLINT(cppcoreguidelines-pro-type-member-init)
{
  corners_[0] = corners_[1] = glm::dvec3(seed);
}


inline Aabb::Aabb(const Aabb &other)  // NOLINT(cppcoreguidelines-pro-type-member-init)
{
  corners_[0] = other.corners_[0];
  corners_[1] = other.corners_[1];
}


inline Aabb::Aabb(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext)  // NOLINT
{
  corners_[0] = min_ext;
  corners_[1] = max_ext;
}


inline Aabb Aabb::fromCentreHalfExtents(const glm::dvec3 &centre, const glm::dvec3 &half_extents)
{
  return Aabb(centre - half_extents, centre + half_extents);
}


inline Aabb Aabb::fromCentreFullExtents(const glm::dvec3 &centre, const glm::dvec3 &full_extents)
{
  return Aabb(centre - 0.5 * full_extents, centre + 0.5 * full_extents);
}


inline glm::dvec3 Aabb::corner(int corner_index) const
{
  glm::dvec3 c;
  c[0] = (corner_index & 1) == 0 ? corners_[0][0] : corners_[1][0];
  c[1] = (corner_index & 2) == 0 ? corners_[0][1] : corners_[1][1];
  c[2] = (corner_index & 4) == 0 ? corners_[0][2] : corners_[1][2];
  return c;
}


inline void Aabb::expand(const glm::dvec3 &pt)
{
  corners_[0].x = std::min(corners_[0].x, pt.x);
  corners_[0].y = std::min(corners_[0].y, pt.y);
  corners_[0].z = std::min(corners_[0].z, pt.z);
  corners_[1].x = std::max(corners_[1].x, pt.x);
  corners_[1].y = std::max(corners_[1].y, pt.y);
  corners_[1].z = std::max(corners_[1].z, pt.z);
}


inline bool Aabb::overlaps(const Aabb &other, double epsilon) const
{
  const glm::bvec3 max_less_min = glm::lessThan(corners_[1] + glm::dvec3(epsilon), other.corners_[0]);
  const glm::bvec3 min_greater_max = glm::greaterThan(corners_[0] - glm::dvec3(epsilon), other.corners_[1]);
  return !glm::any(max_less_min) && !glm::any(min_greater_max);
}


inline bool Aabb::contains(const glm::dvec3 &point, double epsilon) const
{
  return overlaps(Aabb(point - glm::dvec3(epsilon), point + glm::dvec3(epsilon)));
}


inline bool Aabb::isEqual(const Aabb &other, double epsilon) const
{
  const glm::dvec3 min_diff = glm::abs(corners_[0] - other.corners_[0]);
  const glm::dvec3 max_diff = glm::abs(corners_[1] - other.corners_[1]);
  const glm::dvec3 epsilon_v(epsilon);
  return glm::all(glm::lessThanEqual(min_diff, epsilon_v)) && glm::all(glm::lessThanEqual(max_diff, epsilon_v));
}


inline bool Aabb::isValid() const
{
  return glm::all(glm::lessThan(corners_[0], corners_[1]));
}


inline bool Aabb::clipLine(glm::dvec3 &start, glm::dvec3 &end, unsigned *clip_flags) const
{
  // From: https://tavianator.com/fast-branchless-raybounding-box-intersections/
  // Convert to ray format.
  glm::dvec3 direction = end - start;
  const glm::dvec3 origin(start);
  const double max_time = glm::length(direction);
  direction *= 1.0 / max_time;
  const glm::dvec3 inv_dir(1.0 / direction[0], 1.0 / direction[1], 1.0 / direction[2]);
  const glm::ivec3 sign(!!(inv_dir.x < 0), !!(inv_dir.y < 0), !!(inv_dir.z < 0));

  // TODO(KS): remove branching.
  glm::dvec2 tx, ty, tz;

  if (clip_flags)
  {
    *clip_flags = 0;
  }

  tx[0] = calcTimeVal(corners_[sign[0]].x, origin.x, inv_dir.x);
  tx[1] = calcTimeVal(corners_[1 - sign[0]].x, origin.x, inv_dir.x);
  ty[0] = calcTimeVal(corners_[sign[1]].y, origin.y, inv_dir.y);
  ty[1] = calcTimeVal(corners_[1 - sign[1]].y, origin.y, inv_dir.y);

  glm::dvec2 time_best;

  if (!calcIntervalOverlap(tx, ty, &time_best))
  {
    return false;
  }

  tz[0] = calcTimeVal(corners_[sign[2]].z, origin.z, inv_dir.z);
  tz[1] = calcTimeVal(corners_[1 - sign[2]].z, origin.z, inv_dir.z);

  if (!calcIntervalOverlap(time_best, tz, &time_best))
  {
    return false;
  }

  bool intersected = false;
  if (time_best[0] > 0)
  {
    start = origin + time_best[0] * direction;
    // Clamp to the box to cater for floating point error.
    start[0] = std::max(corners_[0][0], std::min(start[0], corners_[1][0]));
    start[1] = std::max(corners_[0][1], std::min(start[1], corners_[1][1]));
    start[2] = std::max(corners_[0][2], std::min(start[2], corners_[1][2]));
    if (clip_flags)
    {
      *clip_flags |= kClippedStart;
    }

    intersected = true;
  }

  if (time_best[1] < max_time)
  {
    end = origin + time_best[1] * direction;
    // Clamp to the box to cater for floating point error.
    end[0] = std::max(corners_[0][0], std::min(end[0], corners_[1][0]));
    end[1] = std::max(corners_[0][1], std::min(end[1], corners_[1][1]));
    end[2] = std::max(corners_[0][2], std::min(end[2], corners_[1][2]));
    if (clip_flags)
    {
      *clip_flags |= kClippedEnd;
    }

    intersected = true;
  }

  return intersected;
}


inline bool Aabb::operator==(const Aabb &other) const
{
  return glm::all(glm::equal(corners_[0], other.corners_[0])) && glm::all(glm::equal(corners_[1], other.corners_[1]));
}


inline bool Aabb::operator!=(const Aabb &other) const
{
  return !(*this == other);
}


inline Aabb &Aabb::operator=(const Aabb &other)
{
  corners_[0] = other.corners_[0];
  corners_[1] = other.corners_[1];
  return *this;
}


inline Aabb operator+(const Aabb &box, const glm::dvec3 &offset)
{
  return Aabb(box.minExtents() + offset, box.maxExtents() + offset);
}


inline Aabb operator+(const glm::dvec3 &offset, const Aabb &box)
{
  return box + offset;
}


inline Aabb operator-(const Aabb &box, const glm::dvec3 &offset)
{
  return Aabb(box.minExtents() - offset, box.maxExtents() - offset);
}


inline Aabb operator-(const glm::dvec3 &offset, const Aabb &box)
{
  return box - offset;
}


inline Aabb &Aabb::operator+=(const glm::dvec3 &offset)
{
  corners_[0] += offset;
  corners_[1] += offset;
  return *this;
}


inline Aabb &Aabb::operator-=(const glm::dvec3 &offset)
{
  corners_[0] -= offset;
  corners_[1] -= offset;
  return *this;
}


inline Aabb operator*(const Aabb &box, double scalar)
{
  return Aabb(box.minExtents() * scalar, box.maxExtents() * scalar);
}


inline Aabb operator*(double scalar, const Aabb &box)
{
  return box * scalar;
}


inline Aabb operator/(const Aabb &box, double scalar)
{
  return box * (1.0 / scalar);
}


inline Aabb &Aabb::operator*=(double scalar)
{
  corners_[0] *= scalar;
  corners_[1] *= scalar;
  return *this;
}


inline Aabb &Aabb::operator/=(double scalar)
{
  const double scalar_inv = 1.0 / scalar;
  corners_[0] *= scalar_inv;
  corners_[1] *= scalar_inv;
  return *this;
}


inline double Aabb::calcTimeVal(double limit, double origin, double direction)
{
  // Always performing the line on the return value nearly works, but occasionally
  // returns NaN instead of infinite results.
  if (direction == std::numeric_limits<double>::infinity())
  {
    return sign(limit - origin) * std::numeric_limits<double>::infinity();
  }
  if (direction == -std::numeric_limits<double>::infinity())
  {
    return sign(limit - origin) * -std::numeric_limits<double>::infinity();
  }
  return (limit - origin) * direction;
}


inline bool Aabb::calcIntervalOverlap(const glm::dvec2 &a, const glm::dvec2 &b, glm::dvec2 *overlap)
{
  if (a[0] > b[1])
  {
    return false;
  }
  if (b[0] > a[1])
  {
    return false;
  }

  (*overlap)[0] = std::max(a[0], b[0]);
  (*overlap)[1] = std::min(a[1], b[1]);
  return true;
}

}  // namespace ohm

#endif  // OHM_AABB_H
