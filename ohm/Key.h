//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHM_KEY_H
#define OHM_KEY_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include <cinttypes>

namespace ohm
{
  /// A key into the occupancy map.
  ///
  /// The key has two indexing parts: a @c regionKey() and a @c localKey(). The region key
  /// represents the high level region or chunk indexed by the key (see @c OccupancyMap for
  /// details), while the local key is a three dimensional index into the chunk itself.
  ///
  /// Note: The default constructor creates a garbage key. Use @c Key::null
  /// or @c Key(nullptr) to initialise a null key.
  class ohm_API Key
  {
  public:
    /// Initialiser value for @c kNull.
    static const int16_t kInvalidValue;
    /// A static instance of a null key (equivalent to @c Key(nullptr) ).
    static const Key kNull;

    /// Hashing structure for the key. To use with hash based containers.
    /// The resulting hash is a 32-bit unsigned integer.
    struct Hash
    {
      /// Calculate the 32-bit hash for @c key.
      /// @param key The key to hash.
      /// @return The 32-bit hash value.
      size_t operator()(const Key &key) const;
    };
    friend Hash;

    /// Construct a garbage key.
    Key();
    /// Construct a @c null key, but only if passed a @c nullptr. Passing a non-null pointer
    /// has undefined results.
    /// @param ptr Must be @c nullptr for defined results.
    Key(void *ptr);

    /// Construct a key for a region.
    /// @param rx The x coordinate for the region key.
    /// @param ry The y coordinate for the region key.
    /// @param rz The z coordinate for the region key.
    /// @param x The x coordinate for the local key. Must be in range for the region.
    /// @param y The y coordinate for the local key. Must be in range for the region.
    /// @param z The z coordinate for the local key. Must be in range for the region.
    Key(int16_t rx, int16_t ry, int16_t rz, uint8_t x, uint8_t y, uint8_t z);

    /// Construct a key for a region.
    /// @param region_key Initialises the region key part of the key.
    /// @param x The x coordinate for the local key. Must be in range for the region.
    /// @param y The y coordinate for the local key. Must be in range for the region.
    /// @param z The z coordinate for the local key. Must be in range for the region.
    Key(const glm::i16vec3 &region_key, uint8_t x, uint8_t y, uint8_t z);

    /// Construct a key for a region.
    /// @param region_key Initialises the region key part of the key.
    /// @param local_key Initialises the local key part of the key.
    Key(const glm::i16vec3 &region_key, const glm::u8vec3 &local_key);

    /// Calculates the hash value of the @c regionKey(), used to lookup a @c MapChunk.
    /// @return The hash of the @c regionKey() part.
    unsigned regionHash() const;

    /// Set the region and local key coordinates along @p axis to match that of @p other.
    ///
    /// @param axis Identifies the axis to set [0, 2] matching XYZ.
    /// @param other The key to copy from.
    void setAxisFrom(int axis, const Key &other);

    /// Set one of the region key axis values.
    /// @param axis The axis to set with 0, 1, 2 cooresponding to x, y, z.
    /// @param val The value to set for the axis.
    void setRegionAxis(int axis, int16_t val);

    /// Access the region part of this key. See class comments.
    /// @return The region part of the key.
    inline const glm::i16vec3 &regionKey() const { return region_key_; }

    /// Set the region part for this key. See class comments.
    /// @param key The new @c regionKey() to set.
    inline void setRegionKey(const glm::i16vec3 &key) { region_key_ = key; }

    /// Set one of the local key axis values.
    /// @param axis The axis to set with 0, 1, 2 cooresponding to x, y, z.
    /// @param val The value to set for the axis. Must be valid for the associated @c MapChunk.
    void setLocalAxis(int axis, uint8_t val);

    /// Access the local part for this key. See class comments.
    /// @return The local part of the key.
    inline const glm::u8vec3 &localKey() const { return local_; }

    /// Set the local part for this key. See class comments.
    /// @param local The new @c localKey() to set.
    inline void setLocalKey(const glm::u8vec3 &local) { local_ = local; }

    /// Test if this key lies within the extents of @p min_bounds and @p max_bounds along the X axis.
    ///
    /// The bounds parameters specify a closed bounding interval, meaning that a key is bounded if it equals
    /// either of the bounding parameters.
    ///
    /// @param min_bounds The minimum bounding range.
    /// @param max_bounds The maximum bounding range.
    /// @return True if this key lies within the X range of [min_bounds, max_bounds].
    inline bool isBoundedX(const Key &min_bounds, const Key &max_bounds) const
    {
      return isBounded(0, min_bounds, max_bounds);
    }

    /// Test if this key lies within the extents of @p min_bounds and @p max_bounds along the Y axis.
    ///
    /// The bounds parameters specify a closed bounding interval, meaning that a key is bounded if it equals
    /// either of the bounding parameters.
    ///
    /// @param min_bounds The minimum bounding range.
    /// @param max_bounds The maximum bounding range.
    /// @return True if this key lies within the Y range of [min_bounds, max_bounds].
    inline bool isBoundedY(const Key &min_bounds, const Key &max_bounds) const
    {
      return isBounded(1, min_bounds, max_bounds);
    }

    /// Test if this key lies within the extents of @p min_bounds and @p max_bounds along the Z axis.
    ///
    /// The bounds parameters specify a closed bounding interval, meaning that a key is bounded if it equals
    /// either of the bounding parameters.
    ///
    /// @param min_bounds The minimum bounding range.
    /// @param max_bounds The maximum bounding range.
    /// @return True if this key lies within the Z range of [min_bounds, max_bounds].
    inline bool isBoundedZ(const Key &min_bounds, const Key &max_bounds) const
    {
      return isBounded(2, min_bounds, max_bounds);
    }

    /// Test if this key lies within the 3D extents of @p min_bounds and @p max_bounds.
    ///
    /// The bounds parameters specify a closed bounding interval, meaning that a key is bounded if it equals
    /// either of the bounding parameters.
    ///
    /// @param min_bounds The minimum bounding extents.
    /// @param max_bounds The maximum bounding extents.
    /// @return True if this key lies within the box defined by [min_bounds, max_bounds].
    bool isBounded(const Key &min_bounds, const Key &max_bounds) const;

    /// Test if this key lies within the extents of @p min_bounds and @p max_bounds along the specified @p axis.
    ///
    /// The bounds parameters specify a closed bounding interval, meaning that a key is bounded if it equals
    /// either of the bounding parameters.
    ///
    /// @param axis The bounding axis to test. Must be [0, 2] identifying XYZ respectively.
    /// @param min_bounds The minimum bounding range.
    /// @param max_bounds The maximum bounding range.
    /// @return True if this key lies within the X range of [min_bounds, max_bounds].
    bool isBounded(int axis, const Key &min_bounds, const Key &max_bounds) const;

    /// Do the region and local key coordinates along @p axis match @p other?
    /// @param axis The axis to test. Must be [0, 2] identifying XYZ respectively.
    /// @param other The key to test against.
    /// @return True on precise region and local key match in @p axis.
    bool axisMatches(int axis, const Key &other) const;

    /// Test whether this is a null key or not. Note, the default constructor creates
    /// invalid keys, not null keys.
    /// @return True if this is a null key.
    inline bool isNull() const { return region_key_ == glm::i16vec3(kInvalidValue); }

    /// Assignment operator.
    /// @param other Key to copy.
    /// @return @c *this
    Key &operator=(const Key &other);

    /// Key equality test.
    /// @param other The key to compare against.
    /// @return True when this key's region and local parts exactly match that of @p other.
    bool operator==(const Key &other) const;

    /// Key inequality test.
    /// @param other The key to compare against.
    /// @return True when this key's region and local parts do not exactly match that of @p other.
    bool operator!=(const Key &other) const;

  private:
    glm::i16vec3 region_key_;
    glm::u8vec3 local_;
  };

  inline Key::Key() = default; // NOLINT


  inline Key::Key(int16_t rx, int16_t ry, int16_t rz, uint8_t x, uint8_t y, uint8_t z)
    : Key(glm::i16vec3(rx, ry, rz), glm::u8vec3(x, y, z))
  {}


  inline Key::Key(const glm::i16vec3 &region_key, uint8_t x, uint8_t y, uint8_t z)
    : Key(region_key, glm::u8vec3(x, y, z))
  {}


  inline Key::Key(const glm::i16vec3 &region_key, const glm::u8vec3 &local_key)
    : region_key_(region_key)
    , local_(local_key)
  {}


  inline Key::Key(void *ptr)
    : region_key_(0)
    , local_(0)
  {
    if (ptr == nullptr)
    {
      *this = Key::kNull;
    }
  }


  inline void Key::setAxisFrom(int axis, const Key &other)
  {
    setRegionAxis(axis, other.region_key_[axis]);
    setLocalAxis(axis, other.local_[axis]);
  }


  inline void Key::setRegionAxis(int axis, int16_t val) { region_key_[axis] = val; }


  inline void Key::setLocalAxis(int axis, uint8_t val) { local_[axis] = val; }


  inline bool Key::isBounded(const Key &min_bounds, const Key &max_bounds) const
  {
    return isBounded(0, min_bounds, max_bounds) && isBounded(1, min_bounds, max_bounds) &&
           isBounded(2, min_bounds, max_bounds);
  }


  inline bool Key::isBounded(int axis, const Key &min_bounds, const Key &max_bounds) const
  {
    if (region_key_[axis] < min_bounds.region_key_[axis] || region_key_[axis] > max_bounds.region_key_[axis])
    {
      return false;
    }

    if (region_key_[axis] == min_bounds.region_key_[axis] && local_[axis] < min_bounds.local_[axis])
    {
      return false;
    }

    if (region_key_[axis] == max_bounds.region_key_[axis] && local_[axis] > max_bounds.local_[axis])
    {
      return false;
    }

    return true;
  }


  inline bool Key::axisMatches(int axis, const Key &other) const
  {
    return region_key_[axis] == other.region_key_[axis] && local_[axis] == other.local_[axis];
  }


  inline Key &Key::operator=(const Key &other) = default;

  inline bool Key::operator==(const Key &other) const
  {
    return region_key_ == other.region_key_ && local_ == other.local_;
  }


  inline bool Key::operator!=(const Key &other) const
  {
    return region_key_ != other.region_key_ || local_ != other.local_;
  }
}  // namespace ohm

#endif  // OHM_KEY_H
