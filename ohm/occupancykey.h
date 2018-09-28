//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OCCUPANCYKEY_H_
#define OCCUPANCYKEY_H_

#include "ohmconfig.h"

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
  /// Note: The default constructor creates a garbage key. Use @c OccupancyKey::null
  /// or @c OccupancyKey(nullptr) to initialise a null key.
  class ohm_API OccupancyKey
  {
  public:
    /// A static instance of a null key (equivalent to @c OccupancyKey(nullptr) ).
    static const OccupancyKey null;

    /// Hashing structure for the key. To use with hash based containers.
    /// The resulting hash is a 32-bit unsigned integer.
    struct Hash
    {
      /// Calculate the 32-bit hash for @c key.
      /// @param key The key to hash.
      /// @return The 32-bit hash value.
      size_t operator()(const OccupancyKey &key) const;
    };
    friend Hash;

    /// Construct a garbage key.
    OccupancyKey();
    /// Construct a @c null key, but only if passed a @c nullptr. Passing a non-null pointer
    /// has undefined results.
    /// @param ptr Must be @c nullptr for defined results.
    OccupancyKey(void *ptr);

    /// Construct a key for a region.
    /// @param rx The x coordinate for the region key.
    /// @param ry The y coordinate for the region key.
    /// @param rx The z coordinate for the region key.
    /// @param x The x coordinate for the local key. Must be in range for the region.
    /// @param y The y coordinate for the local key. Must be in range for the region.
    /// @param z The z coordinate for the local key. Must be in range for the region.
    OccupancyKey(short rx, short ry, short rz, uint8_t x, uint8_t y, uint8_t z);

    /// Construct a key for a region.
    /// @param regionKey Initialises the region key part of the key.
    /// @param x The x coordinate for the local key. Must be in range for the region.
    /// @param y The y coordinate for the local key. Must be in range for the region.
    /// @param z The z coordinate for the local key. Must be in range for the region.
    OccupancyKey(const glm::i16vec3 &regionKey, uint8_t x, uint8_t y, uint8_t z);

    /// Construct a key for a region.
    /// @param regionKey Initialises the region key part of the key.
    /// @param localKey Initialises the local key part of the key.
    OccupancyKey(const glm::i16vec3 &regionKey, const glm::u8vec3 &localKey);

    /// Calculates the hash value of the @c regionKey(), used to lookup a @c MapChunk.
    /// @return The hash of the @c regionKey() part.
    unsigned regionHash() const;

    /// Set one of the region key axis values.
    /// @param axis The axis to set with 0, 1, 2 cooresponding to x, y, z.
    /// @param val The value to set for the axis.
    void setRegionAxis(int axis, int16_t val);

    /// Access the region part of this key. See class comments.
    /// @return The region part of the key.
    inline const glm::i16vec3 &regionKey() const { return _regionKey; }

    /// Set the region part for this key. See class comments.
    /// @param key The new @c regionKey() to set.
    inline void setRegionKey(const glm::i16vec3 &key) { _regionKey = key; }

    /// Set one of the local key axis values.
    /// @param axis The axis to set with 0, 1, 2 cooresponding to x, y, z.
    /// @param val The value to set for the axis. Must be valid for the associated @c MapChunk.
    void setLocalAxis(int axis, uint8_t val);

    /// Access the local part for this key. See class comments.
    /// @return The local part of the key.
    inline const glm::u8vec3 &localKey() const { return _local; }

    /// Set the local part for this key. See class comments.
    /// @param key The new @c localKey() to set.
    inline void setLocalKey(const glm::u8vec3 &local) { _local = local; }

    /// Test whether this is a null key or not. Note, the default constructor creates
    /// invalid keys, not null keys.
    /// @return True if this is a null key.
    inline bool isNull() const { return _regionKey == glm::i16vec3(InvalidValue); }

    /// Assignment operator.
    /// @param other Key to copy.
    /// @return @c *this
    OccupancyKey &operator = (const OccupancyKey &other);

    /// Key equality test.
    /// @param other The key to compare against.
    /// @return True when this key's region and local parts exactly match that of @p other.
    bool operator == (const OccupancyKey &other) const;

    /// Key inequality test.
    /// @param other The key to compare against.
    /// @return True when this key's region and local parts do not exactly match that of @p other.
    bool operator != (const OccupancyKey &other) const;

  private:
    glm::i16vec3 _regionKey;
    glm::u8vec3 _local;

    static int16_t InvalidValue;
  };

  inline OccupancyKey::OccupancyKey()
  {
  }


  inline OccupancyKey::OccupancyKey(short rx, short ry, short rz, uint8_t x, uint8_t y, uint8_t z)
    : OccupancyKey(glm::i16vec3(rx, ry, rz), glm::u8vec3(x, y, z))
  {
  }


  inline OccupancyKey::OccupancyKey(const glm::i16vec3 &regionKey, uint8_t x, uint8_t y, uint8_t z)
    : OccupancyKey(regionKey, glm::u8vec3(x, y, z))
  {
  }


  inline OccupancyKey::OccupancyKey(const glm::i16vec3 &regionKey, const glm::u8vec3 &localKey)
    : _regionKey(regionKey)
    , _local(localKey)
  {
  }


  inline OccupancyKey::OccupancyKey(void *ptr)
  {
    if (ptr == nullptr)
    {
      *this = OccupancyKey::null;
    }
  }


  inline void OccupancyKey::setRegionAxis(int axis, int16_t val)
  {
    _regionKey[axis] = val;
  }


  inline void OccupancyKey::setLocalAxis(int axis, uint8_t val)
  {
    _local[axis] = val;
  }


  inline OccupancyKey &OccupancyKey::operator = (const OccupancyKey &other)
  {
    _regionKey = other._regionKey;
    _local = other._local;
    return *this;
  }

  inline bool OccupancyKey::operator == (const OccupancyKey &other) const
  {
    return _regionKey == other._regionKey && _local == other._local;
  }


  inline bool OccupancyKey::operator != (const OccupancyKey &other) const
  {
    return _regionKey != other._regionKey || _local != other._local;
  }
}

#endif // OCCUPANCYKEY_H_
