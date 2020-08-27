//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OHMVOXELDEPRECATED_H
#define OHMVOXELDEPRECATED_H

#include "OhmConfig.h"

#include "DefaultLayer.h"
#include "Key.h"
#include "MapChunk.h"
#include "MapLayout.h"
#include "MapProbability.h"
#include "VoxelUtil.h"

#include <limits>
#include <type_traits>

namespace ohm
{
  struct OccupancyMapDetail;
  struct MapChunk;

  /// Represents a voxel in an @c OccupancyMap.
  ///
  /// The @c VoxelBase is derived into a @c Voxel and @c VoxelConst. Those two classes represent the public API for
  /// accessing voxels in an @c OccupancyMap.
  ///
  /// This class wraps an underlying occupancy voxel, tracks its associated @c Key and @p MapChunk, and supports data
  /// access on voxel without calling into @c OccupancyMap functions. This makes minor modifications, such as modifying
  /// the @c value() or accessing @c voxel() much more efficient.
  ///
  /// A voxel may only be used so long as the @p MapChunk remains used within the @p OccupancyMap. As such it should be
  /// used as a short lived object.
  ///
  /// @note The @c Voxel and @c VoxelConst implementations are provided for convenience. However, they should be avoided
  /// for optimal performance, in favour of accessing the @c MapChunk and associated memory directly.
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
    /// @param invalid_as_obstructed Defines the behaviour when referencing an invalid voxel.
    /// @return The range to the nearest obstacle, zero if this is an obstructed voxel,
    ///     or negative (-1) if there are no obstacles within the specified query range.
    float clearance(bool invalid_as_obstructed = true) const;

    /// Queries the timestamp for the chunk containing this voxel.
    /// @return The last timestamp associated to the chunk. See @c Voxel::touchRegionTimestamp()
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

    /// Retrieves the (global) position of a voxel with consideration to voxel mean positioning.
    /// This is equivalent to @c centreGlobal() if voxel mean positioning is not enabled, or not resolved for the voxel.
    ///
    /// @seealso @ref voxelmean
    /// @return The global voxel coordinates with voxel mean positioning.
    glm::dvec3 position() const { return voxel::position(key_, *chunk_, *map_); }

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
  ///
  /// Modifying a voxel requires that the map and it's chunk be touched for the layer being modified. This can be done
  /// in a variety of ways, but is not automated by the @c Voxel class for performance reasons. For isolated voxel
  /// updates calling @c touchMap() is satisfactory, passing the layer index for each layer modified where relevant
  /// layer indices are available from @c MapLayout . For example, changing a voxel value modifies the occupancy layer
  /// which can be resolved using @c MapLayout::occupancyLayer() .
  ///
  /// For batch updates, it is recommended that the map is touched once for the batch as a whole -
  /// @c OccupancyMap::touch() - and the @c MapChunk::dirty_stamp and @c MapChunk::touched_stamps values are updated
  /// appropriately (generally maximising the stamp or assuming there are no multi-threaded changes).
  class ohm_API Voxel : public VoxelBase<MapChunk>
  {
  public:
    /// Parent class type.
    using Super = VoxelBase<MapChunk>;

    /// Default constructor (null voxel).
    inline Voxel() = default;

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline Voxel(const Key &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map)
    {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline Voxel(const Voxel &other) = default;

    /// Set the value of the referenced tree voxel. May expand pruned voxels. Voxel must be valid.
    ///
    /// The new value is clamped to the map's allowed min/max value. The function also honours the saturation
    /// flags for the map, where voxel values may be considered saturated at a min or max value and no longer
    /// change. The @p force parameter may be used to disable honouring these constraints.
    ///
    /// Note that setting the voxel value to @c voxel::invalidMarkerValue() is not supported. Use @c setUncertain() .
    ///
    /// Note: @c touchMap() should be called directly for occupancy layer index, or collectively called at a higher
    /// level.
    ///
    /// @param value New value to set. Does not support
    /// @param force Force set the value without honouring saturation or min/max voxel values.
    void setValue(float value, bool force = false);

    /// Forcibly set the value to @c voxel::invalidMarkerValue() , returning the voxel to the @c isUncertain() state.
    void setUncertain();

    /// Set the voxel's @c clearace() value. This is normally generated by the @c VoxelRanges
    /// query.
    ///
    ///
    /// Note: @c touchMap() should be called directly for clearance layer index, or collectively called at a higher
    /// level.
    ///
    /// @param range The new obstacle range.
    void setClearance(float range);

    /// Sets the position of the voxel mean. This requires that the occupancy layer supports the @c VoxelMean layer
    ///
    /// Calling this function resets the point count to @c point_count if that argument is given and non-zero. Otherwise
    /// the current point count is retained.
    ///
    /// Note that the position set will not be accurately reflected when queried with @c position() as voxel mean
    /// positioning uses quantisation.
    ///
    /// The call fails if the map does not have voxel mean positioning enabled, or if the voxel reference is invalid.
    ///
    /// @seealso @ref voxelmean
    ///
    /// @param position The target position to set should be within the voxel bounds, but will be clamped.
    /// @param point_count The new point count associated with the mean coordinate. Zero to leave the current as is.
    /// @return True on success. False indicates there is not voxel mean positioning enabled, or the voxel is invalid.
    bool setPosition(const glm::dvec3 &position, unsigned point_count = 0);

    /// Updates the voxel mean position of this voxel. As with @c setPosition(), this request the @c MapLayout
    /// voxel layer for the @c VoxelMean.
    ///
    /// This method takes the given @c position, assumed to be within the voxel bounds, and adds it to the voxel mean.
    ///
    /// @seealso @ref voxelmean
    ///
    /// @param position The new position information used to update the voxel mean position. Expected to be within the
    ///   bounds of the voxel.
    bool updatePosition(const glm::dvec3 &position);

    /// Updates the timestamp for the @c MapRegion to which this voxel belongs.
    /// This may be used to manage time based expiry.
    /// @param timestamp The timestamp to set for the region.
    void touchRegionTimestamp(double timestamp);

    /// Touch the underlying map, modifying the @p OccupancyMap::stamp() value.
    ///
    /// Must only be called if @c isValid() is @c true.
    ///
    /// @param layer Specifies which voxel layer is touched. The layer value must be in [0, MapLayout::layerCount()].
    void touchMap(int layer);

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
    using Super = VoxelBase<const MapChunk>;

    /// Default constructor (null voxel).
    inline VoxelConst() = default;

    /// Construct a voxel reference to an existing voxel.
    /// @param key The key of the voxel to reference.
    /// @param chunk The @c MapChunk containing the voxel referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline VoxelConst(const Key &key, const MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map)
    {}

    /// Copy constructor.
    /// @param other Voxel to copy.
    inline VoxelConst(const VoxelConst &other) = default;

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
    return *voxel::voxelOccupancyPtr(key_, chunk_, map_);
  }


  template <typename MAPCHUNK>
  float VoxelBase<MAPCHUNK>::clearance(bool invalid_as_obstructed) const
  {
    if (isValid() && chunk_->layout->clearanceLayer() >= 0)
    {
      if (const float *clearance =
            ohm::voxel::voxelPtrAs<const float *>(key_, chunk_, map_, chunk_->layout->clearanceLayer()))
      {
        return *clearance;
      }
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
    const float val = (isValid()) ? value() : voxel::invalidMarkerValue();
    return val != voxel::invalidMarkerValue() && val >= voxel::occupancyThreshold(*map_);
  }


  template <typename MAPCHUNK>
  bool VoxelBase<MAPCHUNK>::isFree() const
  {
    const float val = (isValid()) ? value() : voxel::invalidMarkerValue();
    return val != voxel::invalidMarkerValue() && val < voxel::occupancyThreshold(*map_);
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

#endif  // OHMVOXELDEPRECATED_H
