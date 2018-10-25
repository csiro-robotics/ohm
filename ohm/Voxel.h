//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHMVOXEL_H
#define OHMVOXEL_H

#include "OhmConfig.h"

#include "MapProbability.h"
#include "Key.h"

#include <limits>

namespace ohm
{
  struct OccupancyMapDetail;
  struct MapChunk;

  /// Represents a voxel in an @c OccupancyMap.
  ///
  /// The @c VoxelBase is derived into a @c Voxel and @c VoxelConst. Those
  /// two classes represent the public API for accessing voxels in an @c OccupancyMap.
  ///
  /// This class wraps an underlying occupancy voxel, tracks its associated @c OccupancyKey
  /// and @p MapChunk, and supports data access on voxel without calling into @c OccupancyMap
  /// functions. This makes minor modifications, such as modifying the @c value() or accessing
  /// @c voxel() much more efficient.
  ///
  /// A voxel may only be used so long as the @p MapChunk remains used within the
  /// @p OccupancyMap. As such it should be used as a short lived object.
  class ohm_API VoxelBase
  {
  public:
    /// Value used to identify invalid or uninitialised voxel voxels.
    /// @return A numeric value considered invalid for a voxel value.
    static constexpr float invalidMarkerValue()
    {
      return std::numeric_limits<float>::infinity();
    }

    /// Construct an invalid voxel reference.
    inline VoxelBase() : key_(OccupancyKey::kNull), chunk_(nullptr), map_(nullptr) {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline VoxelBase(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : key_(key), chunk_(chunk), map_(map) {}

    /// Copy constructor.
    /// @param other Object to copy.
    inline VoxelBase(const VoxelBase &other)
      : key_(other.key_), chunk_(other.chunk_), map_(other.map_) {}

    /// Returns the requested key for the voxel reference.
    /// See class comments regarding the difference between @c key() and @c trueKey().
    /// @return The key of the voxel reference.
    inline const OccupancyKey &key() const { return key_; }

    /// Deprecated: use @c occupancy().
    /// @return The occupancy value for the referenced voxel..
    inline float value() const { return occupancy(); }

    /// Returns the @c value of the wrapped voxel. Voxel reference must be valid.
    /// @return The voxel value.
    float occupancy() const;

    /// Convert the @c value() to an occupancy probability.
    /// @return An occupancy probability in the range [0, 1], based on @c value().
    inline float probability() const { return valueToProbability(value()); }

    /// Returns the cached range to the nearest obstacel for this voxel.
    /// This is calculated by the @c VoxelRanges query.
    ///
    /// If referencing an uncertain voxel, then the result depends on @p uncertainAsObstructed.
    /// When @c true (default) the return value is 0 (obstruction) otherwise it is -1 (clear).
    ///
    /// @param uncertain_as_obstructed Defines the behaviour when referencing an uncertain voxel.
    /// @return The range to the nearest obstacle, zero if this is an obstructed voxel,
    ///     or negative (-1) if there are no obstacles within the specified query range.
    float clearance(bool uncertain_as_obstructed = true) const;

    /// Queries the timestamp for the chunk containing this voxel.
    /// @return The last timestamp associated to the chunk. See @c Voxel::touchRegion()
    double regionTimestamp() const;

    /// Is this a valid voxel reference?
    /// @return True if this is a valid voxel reference.
    inline bool isValid() const { return chunk_ != nullptr && !key_.isNull(); }

    /// Is this a null reference?
    /// @return True if this is a null voxel reference.
    inline bool isNull() const { return chunk_ == nullptr || key_.isNull(); }

    /// Is this an occupied, non null voxel?
    /// @return True if this voxel is not null, not free and not uncertain.
    bool isOccupied() const;

    /// Is this a free, non null voxel?
    /// @return True if this voxel is not null, not occupied and not uncertain.
    bool isFree() const;

    /// Is this an untouched, uncertain voxel? Node that a null voxel will report value.
    ///
    /// Nodes are initially uncertain until a valid probability value is set.
    ///
    /// @return True if the referenced voxel is not null and does not have a valid
    ///     probabilty value an is considered uncertain.
    inline bool isUncertain() const { return !isNull() && value() == invalidMarkerValue(); }

    /// Is this a untouched, uncertain voxel or a null voxel?
    ///
    /// Nodes are initially uncertain until a valid probability value is set.
    ///
    /// @return True if the referenced voxel is null or uncertain.
    inline bool isUncertainOrNull() const { return isNull() || value() == invalidMarkerValue(); }

    /// Access the @c MapChunk containing this voxel.
    /// Primarily for internal usage.
    /// @return The MapChunk to which this voxel belongs.
    inline const MapChunk *chunk() const { return chunk_; }

    /// Is this a valid reference?
    /// @return True if this is a valid voxel reference.
    inline operator bool() const { return isValid(); }

    /// Equality comparison.
    /// @param other The voxel reference to compare to.
    /// @return True if this is reference is exactly equivalent to @p other.
    inline bool operator == (const VoxelBase &other) const
    {
      return chunk_ == other.chunk_ && map_ == other.map_ && key_ == other.key_;
    }

    /// Inequality comparison.
    /// @param other The voxel reference to compare to.
    /// @return True if this is reference is not exactly equivalent to @p other.
    inline bool operator != (const VoxelBase &other) const
    {
      return chunk_ != other.chunk_ || map_ != other.map_ || key_ != other.key_;
    }

    /// Simple assignment operator.
    /// @param other The values to assign.
    /// @return <tt>*this</tt>
    inline VoxelBase &operator = (const VoxelBase &other)
    {
      key_ = other.key_;
      chunk_ = other.chunk_;
      map_ = other.map_;
      return *this;
    }

    /// Adjust the voxel key to reference the next voxel in the region chunk. This linearly walks the voxel memory
    /// and can be much faster than iterating using keys.'
    ///
    /// The voxel is not adjusted when it is already referencing the last voxel in the region.
    ///
    /// @return @c true when the voxel is valid and successfully walks to the next voxel, @c false when there are
    ///   no more voxels in the region.
    bool nextInRegion();

  protected:
    /// Key of the voxel in the key.
    OccupancyKey key_;
    /// A pointer to the chunk referenced by _key.
    MapChunk *chunk_;
    /// Details of the owning map.
    OccupancyMapDetail *map_;
  };

  /// Represents a mutable voxel in an @c OccupancyMap.
  ///
  /// This specialisation of @p VoxelBase supports modifying the voxel content such as its
  /// values. It also allows the containing chunk or region to be touched.
  class ohm_API Voxel : public VoxelBase
  {
  public:
    /// Parent class type.
    typedef VoxelBase Super;

    /// Default constructor (null voxel).
    inline Voxel() : Super() {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline Voxel(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map) {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline Voxel(const Voxel &other) : Super(other) {}

    /// Set the value of the referenced tree voxel. May expand pruned voxels. Voxel must be valid.
    ///
    /// The new value is clamped to the map's allowed min/max value. The function also honours the saturation
    /// flags for the map, where voxel values may be considered saturated at a min or max value and no longer
    /// change. The @p force parameter may be used to disable honouring these constraints.
    ///
    /// Note that setting the voxel value to @c invalidMarkerValue() also does not honour min/max value or
    /// saturation.
    ///
    /// @param value New value to set.
    /// @param force Force set the value without honouring saturation or min/max voxel values.
    void setValue(float value, bool force = false);

    /// Set the voxel's @c clearange() value. This is normally generated by the @c VoxelRanges
    /// query.
    ///
    /// This will @c touchMap() on a valid voxel..
    /// @param range The new obstacle range.
    void setClearance(float range);

    /// Updates the timestamp for the @c MapRegion to which this voxel belongs.
    /// This may be used to manage time based expiry.
    /// @param timestamp The timestamp to set for the region.
    void touchRegion(double timestamp);

    /// Touch the underlying map, modifying the @p OccupancyMap::stamp() value.
    void touchMap();

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    Voxel &operator = (const Voxel &other);

  protected:
    friend class VoxelConst;
  };


  /// Represents a read only voxel in an @c OccupancyMap.
  class ohm_API VoxelConst : public VoxelBase
  {
  public:
    /// Parent class type.
    typedef VoxelBase Super;

    /// Default constructor (null voxel).
    inline VoxelConst() : Super() {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline VoxelConst(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map) {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline VoxelConst(const VoxelConst &other) : Super(other) {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline VoxelConst(const Voxel &other)
      : Super(other.key_, other.chunk_, other.map_) {}

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    VoxelConst &operator = (const VoxelConst &other);

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    VoxelConst &operator = (const Voxel &other);

    /// @internal
    Voxel makeMutable() const;
  };

  inline Voxel &Voxel::operator = (const Voxel &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }


  inline VoxelConst &VoxelConst::operator = (const VoxelConst &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }


  inline VoxelConst &VoxelConst::operator = (const Voxel &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }
}

#endif // OHMVOXEL_H
