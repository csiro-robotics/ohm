//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHMVOXEL_H
#define OHMVOXEL_H

#include "OhmConfig.h"

#include "DefaultLayer.h"
#include "Key.h"
#include "MapProbability.h"

#include <limits>
#include <type_traits>

namespace ohm
{
  struct OccupancyMapDetail;
  struct MapChunk;

  namespace voxel
  {
    /// Get the address of the voxel referenced by @p key in @p layer_index.
    ///
    /// This function function decodes accesses the voxel memory for @p layer_index in @p chunk and returns a pointer
    /// to the voxel referenced by @p key. There are several expectations on this method, or it will fail, returning
    /// null:
    /// - @p chunk and @p map are non null.
    /// - @p key references the @p chunk (see @c Key::regionCoord() and @c MapRegion::coord)
    /// - @p expected_size is either zero (no validation) or exactly matches the voxel size.
    ///
    /// Some additional assumptions are made which the caller *must* ensure or results are undefined.
    /// - @p layer_index is valid in the map's @c MapLayout.
    /// - @p key.localKey() is in range of the map's region dimensions.
    ///
    /// This function operated on a mutable @c chunk and returns a non-const pointer.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @param expected_size Optional voxel size validation. Fails if voxels do not match this size. Zero to skip
    ///     validation.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    uint8_t *voxelPtr(const Key &key, MapChunk *chunk, OccupancyMapDetail *map, int layer_index, size_t expected_size);

    /// Get the address of the voxel referenced by @p key in @p layer_index.
    ///
    /// This function overload deals with non-mutable @p chunk pointers.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @param expected_size Optional voxel size validation. Fails if voxels do not match this size. Zero to skip
    ///     validation.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    const uint8_t *voxelPtr(const Key &key, const MapChunk *chunk, OccupancyMapDetail *map, int layer_index,
                            size_t expected_size);

    /// The address of the voxel referenced by @p key as type @c T.
    ///
    /// This function has the same requirements as @c voxelPtr(), however the voxel size is validated against the
    /// @p sizeof(T).
    ///
    /// This overload requires @c T is a pointer type.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    template <typename T>
    T voxelPtrAs(const Key &key, MapChunk *chunk, OccupancyMapDetail *map, int layer_index)
    {
      return reinterpret_cast<T>(voxelPtr(key, chunk, map, layer_index, sizeof(typename std::remove_pointer<T>::type)));
    }

    /// The address of the voxel referenced by @p key as type @c T.
    ///
    /// This function has the same requirements as @c voxelPtr(), however the voxel size is validated against the
    /// @p sizeof(T).
    ///
    /// This overload requires @c T is a constant pointer type.
    ///
    /// @param key The key for the voxel to access.
    /// @param chunk The chunk in which the voxel lies.
    /// @param map Internal details of the @c OccupancyMap.
    /// @return A pointer to the voxel memory of voxel at @p key, or null on failure.
    template <typename T>
    T voxelPtrAs(const Key &key, const MapChunk *chunk, OccupancyMapDetail *map, int layer_index)
    {
      return reinterpret_cast<T>(voxelPtr(key, chunk, map, layer_index, sizeof(typename std::remove_pointer<T>::type)));
    }

    /// A helper function for accessing the occupancy threshold of @p map.
    /// @return The equivalent to @c OccupancyMap::occupancyThreshold().
    float occupancyThreshold(const OccupancyMapDetail &map);

    /// A helper function for accessing the voxel dimensions of each region in @p map.
    /// @return The equivalent to @c OccupancyMap::regionVoxelDimensions().
    glm::u8vec3 regionVoxelDimensions(const OccupancyMapDetail &map);

    /// Retrieve the coordinates for the centre of the voxel identified by @p key, local to the map origin.
    /// @param key The voxel of interest.
    /// @param map Internal details of the @c OccupancyMap of interest.
    /// @return The voxel coordinates, relative to the @c OccupancyMap::origin().
    glm::dvec3 centreLocal(const Key &key, const OccupancyMapDetail &map);

    /// Retrieve the global coordinates for the centre of the voxel identified by @p key. This includes the
    /// @c OccupancyMap::origin().
    /// @param key The voxel of interest.
    /// @param map Internal details of the @c OccupancyMap of interest.
    /// @return The global voxel coordinates.
    glm::dvec3 centreGlobal(const Key &key, const OccupancyMapDetail &map);

    /// Value used to identify invalid or uninitialised voxel voxels.
    /// @return A numeric value considered invalid for a voxel value.
    constexpr float invalidMarkerValue() { return std::numeric_limits<float>::infinity(); }
  }  // namespace voxel

  /// Represents a voxel in an @c OccupancyMap.
  ///
  /// The @c VoxelBase is derived into a @c Voxel and @c VoxelConst. Those
  /// two classes represent the public API for accessing voxels in an @c OccupancyMap.
  ///
  /// This class wraps an underlying occupancy voxel, tracks its associated @c Key
  /// and @p MapChunk, and supports data access on voxel without calling into @c OccupancyMap
  /// functions. This makes minor modifications, such as modifying the @c value() or accessing
  /// @c voxel() much more efficient.
  ///
  /// A voxel may only be used so long as the @p MapChunk remains used within the
  /// @p OccupancyMap. As such it should be used as a short lived object.
  template <typename MAPCHUNK>
  class ohm_API VoxelBase
  {
  public:
    /// Construct an invalid voxel reference.
    inline VoxelBase()
      : key_(Key::kNull)
      , chunk_(nullptr)
      , map_(nullptr)
    {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline VoxelBase(const Key &key, MAPCHUNK *chunk, OccupancyMapDetail *map)
      : key_(key)
      , chunk_(chunk)
      , map_(map)
    {}

    /// Copy constructor.
    /// @param other Object to copy.
    template <typename OTHERCHUNK>
    inline VoxelBase(const VoxelBase<OTHERCHUNK> &other)
      : key_(other.key_)
      , chunk_(other.chunk_)
      , map_(other.map_)
    {}

    /// Returns the requested key for the voxel reference.
    /// See class comments regarding the difference between @c key() and @c trueKey().
    /// @return The key of the voxel reference.
    inline const Key &key() const { return key_; }

    /// Deprecated: use @c occupancy().
    /// @return The occupancy value for the referenced voxel..
    inline float value() const { return occupancy(); }

    /// Returns the @c value of the wrapped voxel. Voxel reference must be valid.
    /// @return The voxel value.
    float occupancy() const;

    /// Convert the @c value() to an occupancy probability.
    /// @return An occupancy probability in the range [0, 1], based on @c value().
    inline float probability() const { return valueToProbability(value()); }

    /// Returns the cached range to the nearest obstacle for this voxel.
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

    /// Retrieve the coordinates for the centre of the voxel identified by @p key, local to the map origin.
    ///
    /// Results are undefined if @c isValid() is @c false.
    /// @return The voxel coordinates, relative to the @c OccupancyMap::origin().
    inline glm::dvec3 centreLocal() const { return voxel::centreLocal(key_, *map_); }

    /// Retrieve the global coordinates for the centre of the voxel identified by @p key. This includes the
    /// @c OccupancyMap::origin().
    ///
    /// Results are undefined if @c isValid() is @c false.
    /// @return The global voxel coordinates.
    glm::dvec3 centreGlobal() const { return voxel::centreGlobal(key_, *map_); }

    /// Does this voxel reference a valid map?
    ///
    /// This is slightly different from @c isValid(). Whereas @c isValid() ensures values can be read from this voxel
    /// this method checks if this voxel has a map reference and valid key. If this is true, then the @c neighbour()
    /// function may be used.
    inline bool isMapValid() const { return map_ != nullptr && !key_.isNull(); }

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
    inline bool isUncertain() const { return !isNull() && value() == voxel::invalidMarkerValue(); }

    /// Is this a untouched, uncertain voxel or a null voxel?
    ///
    /// Nodes are initially uncertain until a valid probability value is set.
    ///
    /// @return True if the referenced voxel is null or uncertain.
    inline bool isUncertainOrNull() const { return isNull() || value() == voxel::invalidMarkerValue(); }

    /// Access the voxel content at @p layer_index as the given (pointer) type.
    ///
    /// Type @c T must be a pointer type and must be @c const for @c VoxelConst.. Results are undefined if this voxel
    /// is not valid.
    ///
    /// @param layer_index The map layer index from which to extract voxel data.
    /// @return A pointer to this voxel's data within the @c MapLayer @p layer_index.
    template <typename T>
    T layerContent(unsigned layer_index) const
    {
      return voxel::voxelPtrAs<T>(key_, chunk_, map_, layer_index);
    }

    /// Access the @c MapChunk containing this voxel.
    /// Primarily for internal usage.
    /// @return The MapChunk to which this voxel belongs.
    inline MAPCHUNK *chunk() const { return chunk_; }

    /// Is this a valid reference?
    /// @return True if this is a valid voxel reference.
    inline operator bool() const { return isValid(); }

    /// Equality comparison.
    /// @param other The voxel reference to compare to.
    /// @return True if this is reference is exactly equivalent to @p other.
    template <typename OTHERCHUNK>
    inline bool operator==(const VoxelBase<OTHERCHUNK> &other) const
    {
      return chunk_ == other.chunk_ && map_ == other.map_ && key_ == other.key_;
    }

    /// Inequality comparison.
    /// @param other The voxel reference to compare to.
    /// @return True if this is reference is not exactly equivalent to @p other.
    template <typename OTHERCHUNK>
    inline bool operator!=(const VoxelBase<OTHERCHUNK> &other) const
    {
      return chunk_ != other.chunk_ || map_ != other.map_ || key_ != other.key_;
    }

    /// Simple assignment operator.
    /// @param other The values to assign.
    /// @return <tt>*this</tt>
    template <typename OTHERCHUNK>
    inline VoxelBase &operator=(const VoxelBase<OTHERCHUNK> &other)
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
    Key key_;
    /// A pointer to the chunk referenced by _key.
    MAPCHUNK *chunk_;
    /// Details of the owning map.
    OccupancyMapDetail *map_;
  };

  /// Represents a mutable voxel in an @c OccupancyMap.
  ///
  /// This specialisation of @p VoxelBase supports modifying the voxel content such as its
  /// values. It also allows the containing chunk or region to be touched.
  class ohm_API Voxel : public VoxelBase<MapChunk>
  {
  public:
    /// Parent class type.
    typedef VoxelBase<MapChunk> Super;

    /// Default constructor (null voxel).
    inline Voxel()
      : Super()
    {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline Voxel(const Key &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map)
    {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline Voxel(const Voxel &other)
      : Super(other)
    {}

    /// Set the value of the referenced tree voxel. May expand pruned voxels. Voxel must be valid.
    ///
    /// The new value is clamped to the map's allowed min/max value. The function also honours the saturation
    /// flags for the map, where voxel values may be considered saturated at a min or max value and no longer
    /// change. The @p force parameter may be used to disable honouring these constraints.
    ///
    /// Note that setting the voxel value to @c voxel::invalidMarkerValue() also does not honour min/max value or
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

    /// Retrieve another voxel relative to this one.
    /// @param dx The number of voxels to offset from this one along the X axis.
    /// @param dy The number of voxels to offset from this one along the Y axis.
    /// @param dz The number of voxels to offset from this one along the Z axis.
    /// @return A reference to the nearby voxel. The result may not be a valid voxel.
    Voxel neighbour(int dx, int dy, int dz) const;

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    Voxel &operator=(const Voxel &other);

  protected:
    friend class VoxelConst;
  };


  /// Represents a read only voxel in an @c OccupancyMap.
  class ohm_API VoxelConst : public VoxelBase<const MapChunk>
  {
  public:
    /// Parent class type.
    typedef VoxelBase<const MapChunk> Super;

    /// Default constructor (null voxel).
    inline VoxelConst()
      : Super()
    {}

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline VoxelConst(const Key &key, const MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map)
    {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline VoxelConst(const VoxelConst &other)
      : Super(other)
    {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline VoxelConst(const Voxel &other)
      : Super(other.key_, other.chunk_, other.map_)
    {}

    /// Retrieve another voxel relative to this one.
    /// @param dx The number of voxels to offset from this one along the X axis.
    /// @param dy The number of voxels to offset from this one along the Y axis.
    /// @param dz The number of voxels to offset from this one along the Z axis.
    /// @return A reference to the nearby voxel. The result may not be a valid voxel.
    VoxelConst neighbour(int dx, int dy, int dz) const;

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    VoxelConst &operator=(const VoxelConst &other);

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other Voxel to assign.
    VoxelConst &operator=(const Voxel &other);

    /// @internal
    Voxel makeMutable() const;
  };

  inline Voxel &Voxel::operator=(const Voxel &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }


  inline VoxelConst &VoxelConst::operator=(const VoxelConst &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }


  inline VoxelConst &VoxelConst::operator=(const Voxel &other)
  {
    chunk_ = other.chunk_;
    key_ = other.key_;
    map_ = other.map_;
    return *this;
  }


  template <typename MAPCHUNK>
  float VoxelBase<MAPCHUNK>::occupancy() const
  {
    if (const float *occupancy = ohm::voxel::voxelPtrAs<const float *>(key_, chunk_, map_, kDlOccupancy))
    {
      return *occupancy;
    }

    return voxel::invalidMarkerValue();
  }


  template <typename MAPCHUNK>
  float VoxelBase<MAPCHUNK>::clearance(bool invalid_as_obstructed) const
  {
    if (const float *clearance = ohm::voxel::voxelPtrAs<const float *>(key_, chunk_, map_, kDlClearance))
    {
      return *clearance;
    }

    return (invalid_as_obstructed) ? 0.0f : -1.0f;
  }


  template <typename MAPCHUNK>
  double VoxelBase<MAPCHUNK>::regionTimestamp() const
  {
    return (chunk_) ? chunk_->touched_time : 0.0;
  }


  template <typename MAPCHUNK>
  bool VoxelBase<MAPCHUNK>::isOccupied() const
  {
    const float val = value();
    return !isNull() && val >= voxel::occupancyThreshold(*map_) && val != voxel::invalidMarkerValue();
  }


  template <typename MAPCHUNK>
  bool VoxelBase<MAPCHUNK>::isFree() const
  {
    const float val = value();
    return !isNull() && val < voxel::occupancyThreshold(*map_) && val != voxel::invalidMarkerValue();
  }


  template <typename MAPCHUNK>
  bool VoxelBase<MAPCHUNK>::nextInRegion()
  {
    if (!chunk_)
    {
      return false;
    }

    const glm::u8vec3 region_dim = voxel::regionVoxelDimensions(*map_);
    if (key_.localKey().x + 1 == region_dim.x)
    {
      if (key_.localKey().y + 1 == region_dim.y)
      {
        if (key_.localKey().z + 1 == region_dim.z)
        {
          return false;
        }

        key_.setLocalKey(glm::u8vec3(0, 0, key_.localKey().z + 1));
      }
      else
      {
        key_.setLocalKey(glm::u8vec3(0, key_.localKey().y + 1, key_.localKey().z));
      }
    }
    else
    {
      key_.setLocalAxis(0, key_.localKey().x + 1);
    }

    return true;
  }
}  // namespace ohm

#endif  // OHMVOXEL_H
