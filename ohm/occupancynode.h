//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2017
//
#ifndef OCCUPANCYNODE_H_
#define OCCUPANCYNODE_H_

#include "ohmconfig.h"

#include "mapprobability.h"
#include "occupancykey.h"

#include <limits>

namespace ohm
{
  struct OccupancyMapDetail;
  struct MapChunk;

  /// Represents a node in an @c OccupancyMap.
  ///
  /// The @c NodeBase is derived into a @c OccupancyNode and @c OccupancyNodeConst. Those
  /// two classes represent the public API for accessing nodes in an @c OccupancyMap.
  ///
  /// This class wraps an underlying occupancy voxel, tracks its associated @c OccupancyKey
  /// and @p MapChunk, and supports data access on node without calling into @c OccupancyMap
  /// functions. This makes minor modifications, such as modifying the @c value() or accessing
  /// @c node() much more efficient.
  ///
  /// A node may only be used so long as the @p MapChunk remains used within the
  /// @p OccupancyMap. As such it should be used as a short lived object.
  class ohm_API NodeBase
  {
  public:
    /// Value used to identify invalid or uninitialised voxel nodes.
    /// @return A numeric value considered invalid for a voxel value.
    static constexpr float invalidMarkerValue()
    {
      return std::numeric_limits<float>::infinity();
    }

    /// Construct an invalid node reference.
    inline NodeBase() : _key(OccupancyKey::null), _chunk(nullptr), _map(nullptr) {}

    /// Construct a node reference to an existing node.
    /// @param key The key of the node to reference.
    /// @param chunk The @c MapChunk containing the node referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline NodeBase(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : _key(key), _chunk(chunk), _map(map) {}

    /// Copy constructor.
    /// @param other Object to copy.
    inline NodeBase(const NodeBase &other)
      : _key(other._key), _chunk(other._chunk), _map(other._map) {}

    /// Returns the requested key for the node reference.
    /// See class comments regarding the difference between @c key() and @c trueKey().
    /// @return The key of the node reference.
    inline const OccupancyKey &key() const { return _key; }

    /// Deprecated: use @c occupancy().
    /// @return The occupancy value for the referenced node..
    inline float value() const { return occupancy(); }

    /// Returns the @c value of the wrapped node. OccupancyNode reference must be valid.
    /// @return The node value.
    float occupancy() const;

    /// Convert the @c value() to an occupancy probability.
    /// @return An occupancy probability in the range [0, 1], based on @c value().
    inline float probability() const { return valueToProbability(value()); }

    /// Returns the cached range to the nearest obstacel for this voxel.
    /// This is calculated by the @c VoxelRanges query.
    ///
    /// If referencing an uncertain node, then the result depends on @p uncertainAsObstructed.
    /// When @c true (default) the return value is 0 (obstruction) otherwise it is -1 (clear).
    ///
    /// @param uncertainAsObstructed Defines the behaviour when referencing an uncertain node.
    /// @return The range to the nearest obstacle, zero if this is an obstructed voxel,
    ///     or negative (-1) if there are no obstacles within the specified query range.
    float clearance(bool uncertainAsObstructed = true) const;

    /// Queries the timestamp for the chunk containing this node.
    /// @return The last timestamp associated to the chunk. See @c OccupancyNode::touchRegion()
    double regionTimestamp() const;

    /// Is this a valid node reference?
    /// @return True if this is a valid node reference.
    inline bool isValid() const { return _chunk != nullptr && !_key.isNull(); }

    /// Is this a null reference?
    /// @return True if this is a null node reference.
    inline bool isNull() const { return _chunk == nullptr || _key.isNull(); }

    /// Is this an occupied, non null node?
    /// @return True if this node is not null, not free and not uncertain.
    bool isOccupied() const;

    /// Is this a free, non null node?
    /// @return True if this node is not null, not occupied and not uncertain.
    bool isFree() const;

    /// Is this an untouched, uncertain node? Node that a null node will report value.
    ///
    /// Nodes are initially uncertain until a valid probability value is set.
    ///
    /// @return True if the referenced node is not null and does not have a valid
    ///     probabilty value an is considered uncertain.
    inline bool isUncertain() const { return !isNull() && value() == invalidMarkerValue(); }

    /// Is this a untouched, uncertain node or a null node?
    ///
    /// Nodes are initially uncertain until a valid probability value is set.
    ///
    /// @return True if the referenced node is null or uncertain.
    inline bool isUncertainOrNull() const { return isNull() || value() == invalidMarkerValue(); }

    /// Access the @c MapChunk containing this node.
    /// Primarily for internal usage.
    /// @return The MapChunk to which this node belongs.
    inline const MapChunk *chunk() const { return _chunk; }

    /// Is this a valid reference?
    /// @return True if this is a valid node reference.
    inline operator bool() const { return isValid(); }

    /// Equality comparison.
    /// @param other The node reference to compare to.
    /// @return True if this is reference is exactly equivalent to @p other.
    inline bool operator == (const NodeBase &other) const
    {
      return _chunk == other._chunk && _map == other._map && _key == other._key;
    }

    /// Inequality comparison.
    /// @param other The node reference to compare to.
    /// @return True if this is reference is not exactly equivalent to @p other.
    inline bool operator != (const NodeBase &other) const
    {
      return _chunk != other._chunk || _map != other._map || _key != other._key;
    }

    /// Simple assignment operator.
    /// @param other The values to assign.
    /// @return <tt>*this</tt>
    inline NodeBase &operator = (const NodeBase &other)
    {
      _key = other._key;
      _chunk = other._chunk;
      _map = other._map;
      return *this;
    }

    /// Adjust the node key to reference the next node in the region chunk. This linearly walks the node memory
    /// and can be much faster than iterating using keys.'
    ///
    /// The node is not adjusted when it is already referencing the last node in the region.
    ///
    /// @return @c true when the node is valid and successfully walks to the next node, @c false when there are
    ///   no more nodes in the region.
    bool nextInRegion();

  protected:
    /// Key of the node in the key.
    OccupancyKey _key;
    /// A pointer to the chunk referenced by _key.
    MapChunk *_chunk;
    /// Details of the owning map.
    OccupancyMapDetail *_map;
  };

  /// Represents a mutable node in an @c OccupancyMap.
  ///
  /// This specialisation of @p NodeBase supports modifying the node content such as its
  /// values. It also allows the containing chunk or region to be touched.
  class ohm_API OccupancyNode : public NodeBase
  {
  public:
    /// Parent class type.
    typedef NodeBase Super;

    /// Default constructor (null node).
    inline OccupancyNode() : Super() {}

    /// Construct a node reference to an existing node.
    /// @param key The key of the node to reference.
    /// @param chunk The @c MapChunk containing the node referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline OccupancyNode(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map) {}

    /// Copy constructor.
    /// @param other OccupancyNode to copy.
    inline OccupancyNode(const OccupancyNode &other) : Super(other) {}

    /// Set the value of the referenced tree node. May expand pruned nodes. OccupancyNode must be valid.
    ///
    /// The new value is clamped to the map's allowed min/max value. The function also honours the saturation
    /// flags for the map, where node values may be considered saturated at a min or max value and no longer
    /// change. The @p force parameter may be used to disable honouring these constraints.
    ///
    /// Note that setting the node value to @c invalidMarkerValue() also does not honour min/max value or
    /// saturation.
    ///
    /// @param value New value to set.
    /// @param force Force set the value without honouring saturation or min/max node values.
    void setValue(float value, bool force = false);

    /// Set the node's @c clearange() value. This is normally generated by the @c VoxelRanges
    /// query.
    ///
    /// This will @c touchMap() on a valid node..
    /// @param range The new obstacle range.
    void setClearance(float range);

    /// Updates the timestamp for the @c MapRegion to which this node belongs.
    /// This may be used to manage time based expiry.
    /// @param timestamp The timestamp to set for the region.
    void touchRegion(double timestamp);

    /// Touch the underlying map, modifying the @p OccupancyMap::stamp() value.
    void touchMap();

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other OccupancyNode to assign.
    OccupancyNode &operator = (const OccupancyNode &other);

  protected:
    friend class OccupancyNodeConst;
  };


  /// Represents a read only node in an @c OccupancyMap.
  class ohm_API OccupancyNodeConst : public NodeBase
  {
  public:
    /// Parent class type.
    typedef NodeBase Super;

    /// Default constructor (null node).
    inline OccupancyNodeConst() : Super() {}

    /// Construct a node reference to an existing node.
    /// @param key The key of the node to reference.
    /// @param chunk The @c MapChunk containing the node referenced by @p key.
    /// @param map The data for the @c OccupancyMap to which @p chunk belongs.
    inline OccupancyNodeConst(const OccupancyKey &key, MapChunk *chunk, OccupancyMapDetail *map)
      : Super(key, chunk, map) {}

    /// Copy constructor.
    /// @param other OccupancyNode to copy.
    inline OccupancyNodeConst(const OccupancyNodeConst &other) : Super(other) {}

    /// Copy constructor.
    /// @param other OccupancyNode to copy.
    inline OccupancyNodeConst(const OccupancyNode &other)
      : Super(other._key, other._chunk, other._map) {}

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other OccupancyNode to assign.
    OccupancyNodeConst &operator = (const OccupancyNodeConst &other);

    /// Assignment, changing the reference only (not the referenced data).
    /// @param other OccupancyNode to assign.
    OccupancyNodeConst &operator = (const OccupancyNode &other);

    /// @internal
    OccupancyNode makeMutable() const;
  };

  inline OccupancyNode &OccupancyNode::operator = (const OccupancyNode &other)
  {
    _chunk = other._chunk;
    _key = other._key;
    _map = other._map;
    return *this;
  }


  inline OccupancyNodeConst &OccupancyNodeConst::operator = (const OccupancyNodeConst &other)
  {
    _chunk = other._chunk;
    _key = other._key;
    _map = other._map;
    return *this;
  }


  inline OccupancyNodeConst &OccupancyNodeConst::operator = (const OccupancyNode &other)
  {
    _chunk = other._chunk;
    _key = other._key;
    _map = other._map;
    return *this;
  }
}

#endif // OCCUPANCYNODE_H_
