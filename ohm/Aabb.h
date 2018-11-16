// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_AABB_H
#define OHM_AABB_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

namespace ohm
{
  /// An axis aligned box
  class Aabb
  {
  public:
    /// Empty constructor: uninitialised.
    Aabb();
    /// Copy constructor.
    /// @param other Value to copy.
    Aabb(const Aabb &other);
    /// Initialise with the given min/max extents.
    /// @param min_ext Minimum extents. Must be <= @p max_ext.
    /// @param max_ext Maximum extents.
    Aabb(const glm::dvec3 min_ext, const glm::dvec3 &max_ext);

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
    inline void setMinExtents(const glm::dvec3 &ext) { min_ext = ext; }

    /// Set the box upper extents corner.
    /// @param ext New coordinate for the upper box extents. No validation (for being >= @c minExtents).
    inline void setMaxExtents(const glm::dvec3 &ext) { max_ext = ext; }

    /// Query the lower box extents.
    /// @return The lower bounds of the box.
    inline glm::dvec3 minExtents() const { return min_ext; }

    /// Query the upper box extents.
    /// @return The upper bounds of the box.
    inline glm::dvec3 maxExtents() const { return max_ext; }

    /// Query the box centre.
    /// @return The box centre.
    inline glm::dvec3 centre() const { return 0.5 * (min_ext + max_ext); }

    /// Query the box half extents.
    /// @return The box half extents (half the diagonal).
    inline glm::dvec3 halfExtents() const { return 0.5 * (max_ext - centre()); }

    /// Query the full box extents.
    /// @return The full extents (the diagonal).
    inline glm::dvec3 diagonal() const { return max_ext - centre(); }

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
    const Aabb &operator=(const Aabb &other);

    /// Translate the box by @p offset.
    /// @param offset Translation to apply.
    /// @return *this
    Aabb &operator += (const glm::dvec3 &offset);

    /// Translate the box by @p -offset.
    /// @param offset Inverse of the translation to apply.
    /// @return *this
    Aabb &operator -= (const glm::dvec3 &offset);

    /// Scale box by @p scalar.
    /// @param scalar Scaling factor.
    /// @return *this
    Aabb &operator *= (double scalar);

    /// Scale box by the inverse of @p scalar.
    /// @param scalar Inverse scaling factor. Must not be zero.
    /// @return *this
    Aabb &operator /= (double scalar);

  private:
    glm::dvec3 min_ext = glm::dvec3(0);
    glm::dvec3 max_ext = glm::dvec3(0);
  };


  inline Aabb::Aabb() {}


  inline Aabb::Aabb(const Aabb &other)
    : min_ext(other.min_ext)
    , max_ext(other.max_ext)
  {
  }


  inline Aabb::Aabb(const glm::dvec3 min_ext, const glm::dvec3 &max_ext)
    : min_ext(min_ext)
    , max_ext(max_ext)
  {
  }


  inline Aabb Aabb::fromCentreHalfExtents(const glm::dvec3 &centre, const glm::dvec3 &half_extents)
  {
    return Aabb(centre - half_extents, centre + half_extents);
  }


  inline Aabb Aabb::fromCentreFullExtents(const glm::dvec3 &centre, const glm::dvec3 &full_extents)
  {
    return Aabb(centre - 0.5 * full_extents, centre + 0.5 * full_extents);
  }


  inline bool Aabb::overlaps(const Aabb &other, double epsilon) const
  {
    const glm::bvec3 max_less_min = glm::lessThan(max_ext + glm::dvec3(epsilon), other.min_ext);
    const glm::bvec3 min_greater_max = glm::greaterThan(min_ext - glm::dvec3(epsilon), other.max_ext);
    return !glm::any(max_less_min) && !glm::any(min_greater_max);
  }


  inline bool Aabb::contains(const glm::dvec3 &point, double epsilon) const
  {
    return overlaps(Aabb(point - glm::dvec3(epsilon), point + glm::dvec3(epsilon)));
  }


  inline bool Aabb::isEqual(const Aabb &other, double epsilon) const
  {
    const glm::dvec3 min_diff = glm::abs(min_ext - other.min_ext);
    const glm::dvec3 max_diff = glm::abs(max_ext - other.max_ext);
    const glm::dvec3 epsilon_v(epsilon);
    return glm::all(glm::lessThanEqual(min_diff, epsilon_v)) && glm::all(glm::lessThanEqual(max_diff, epsilon_v));
  }


  inline bool Aabb::operator==(const Aabb &other) const
  {
    return glm::all(glm::equal(min_ext, other.min_ext)) && glm::all(glm::equal(max_ext, other.max_ext));
  }


  inline bool Aabb::operator!=(const Aabb &other) const
  {
    return !(*this == other);
  }


  inline const Aabb &Aabb::operator=(const Aabb &other)
  {
    min_ext = other.min_ext;
    max_ext = other.max_ext;
    return *this;
  }


  inline Aabb operator + (const Aabb &box, const glm::dvec3 &offset)
  {
    return Aabb(box.minExtents() + offset, box.maxExtents() + offset);
  }


  inline Aabb operator + (const glm::dvec3 &offset, const Aabb &box)
  {
    return box + offset;
  }


  inline Aabb operator - (const Aabb &box, const glm::dvec3 &offset)
  {
    return Aabb(box.minExtents() - offset, box.maxExtents() - offset);
  }


  inline Aabb operator - (const glm::dvec3 &offset, const Aabb &box)
  {
    return box - offset;
  }


  inline Aabb &Aabb::operator += (const glm::dvec3 &offset)
  {
    min_ext += offset;
    max_ext += offset;
    return *this;
  }


  inline Aabb &Aabb::operator -= (const glm::dvec3 &offset)
  {
    min_ext -= offset;
    max_ext -= offset;
    return *this;
  }


  inline Aabb operator * (const Aabb &box, double scalar)
  {
    return Aabb(box.minExtents() * scalar, box.maxExtents() * scalar);
  }


  inline Aabb operator * (double scalar, const Aabb &box)
  {
    return box * scalar;
  }


  inline Aabb operator / (const Aabb &box, double scalar)
  {
    return box * (1.0 / scalar);
  }


  inline Aabb &Aabb::operator *= (double scalar)
  {
    min_ext *= scalar;
    max_ext *= scalar;
    return *this;
  }


  inline Aabb &Aabb::operator /= (double scalar)
  {
    const double scalar_inv = 1.0 / scalar;
    min_ext *= scalar_inv;
    max_ext *= scalar_inv;
    return *this;
  }
}

#endif // OHM_AABB_H
