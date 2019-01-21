// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYQUERY_H
#define OCCUPANCYQUERY_H

#include "OhmConfig.h"

#include <cstddef>

namespace ohm
{
  class OccupancyMap;
  class Key;
  struct QueryDetail;

  /// Base class for a query operation on an @p OccupancyMap.
  ///
  /// Typical usage is to setup the dervied query, then call @c exec() or @c execAsync() and @c wait().
  /// Results may be available via methods such as @c numberOfResults() and @p intersectedVoxels(). Query behaviour may
  /// be modified by setting various @c QueryFlag values in @c setQueryFlags(), although not all flags are honoured by
  /// all query implementations.
  ///
  /// Most queries operate on searching for occupied voxels, as defined by @c OccupancyMap::occupancyType(). By default
  /// only voxels of type @c OccupancyType::Occupied are considered occupied. For most queries, setting the query flag
  /// @c QF_UnknownAsOccupied ensures that @c OccupancyType::Uncertain are also considered as relevant obstructions.
  /// We define the set of such relevant voxels as the set of @em obstructed voxels.
  ///
  /// @todo Create a base class for the @c Query called @c MapOperation. @c Query adds the concept of results, which
  /// @c MapOperation does not have.
  class ohm_API Query
  {
  protected:
    /// Constructor. The @p detail is stored in @p imp_, allowing derived classes to a allocate
    /// derived detail structure. When null, the base implementation is allocated by this constructor.
    /// @param detail The @c Query data detail structure.
    Query(QueryDetail *detail = nullptr);

  public:
    /// Virtual destructor.
    virtual ~Query();

    /// Get the map the query has been associated with.
    /// @return The target map.
    const OccupancyMap *map() const;

    /// Set the map to operate on.
    /// @param map The map to query.
    void setMap(OccupancyMap *map);

    /// Access the @c QueryFlag bits set for this query.
    /// @return The @c QueryFlag set for this query.
    unsigned queryFlags() const;

    /// Set the @c QueryFlag bits for this query.
    /// @param flags The new @c QueryFlag value set for this query.
    void setQueryFlags(unsigned flags);

    /// Query the number of available results.
    ///
    /// This affects the number of items available from methods such as @p intersectedVoxel().
    /// It is valid after execution completes until the next call to @p exec() or @c execAsync().
    ///
    /// @return The number of available results.
    size_t numberOfResults() const;

    /// Query the voxels intersected by the query, where voxels are identified by their @c Key.
    ///
    /// When non-null, the number of items available in @p intersectedVoxels() is available via
    /// @c numberOfResults().
    ///
    /// Sort order and semantics may vary slightly for specific queries.
    ///
    /// @return A pointer to the array of voxels intersected by the query.
    const Key *intersectedVoxels() const;

    /// Get the array of distances associated with the @c intersectedVoxels().
    /// This may not be valid for some queries and could return null.
    ///
    /// When non-null, the array contains @c numberOfResults() elements, where each element corresponds
    /// to a voxel reported by @c intersectedVoxels().
    ///
    /// @return The array of distances to the line segment for each @c intersectedVoxel().
    const float *ranges() const;

    /// Execute a synchronous query.
    ///
    /// This calls through to the implementation in @p onExecute().
    ///
    /// @return True on success.
    bool execute();

    /// Execute an asynchronous query.
    ///
    /// This calls through to the implementation in @p onExecuteAsync(). On success, completion
    /// can be synchronised by calling @p wait() with an optional wait timeout.
    ///
    /// The method will fail when already executing a query.
    ///
    /// @return True on successfully starting query execution.
    bool executeAsync();

    /// Wait for/terminate any asynchronous query and clear results data. This will also wait
    /// for any oustanding asynchronous query.
    ///
    /// Some queries may differentiate between a hard and soft reset (@p hardReset @c true or @c false
    /// respectively). In a hard reset, all information from the previous query are discraded. In a soft
    /// reset, some data may be cached. For example, @c VoxelRanges supports a soft reset where it will
    /// not recalculate obstacle ranges for recently queried regions.
    ///
    /// @param hard_reset True for a hard reset, false for a soft reset.
    void reset(bool hard_reset = true);

    /// Wait for an asynchronous query to complete.
    ///
    /// @param timeout_ms Maximum amount of time to wait for completion (milliseconds).
    /// @return True if on return there is no asychronous query running. This does not mean that there was
    ///   one running to begin with.
    bool wait(unsigned timeout_ms = ~0u);

  protected:
    /// Virtual call for when a map is set.
    /// This is only called from @p setMap(), not from the constructor.
    virtual inline void onSetMap() {}

    /// Virtual function called to execute the query from @c execute().
    ///
    /// Derived classes must implement a synchronous query in this function returning only on failure or once
    /// the query completes. Results must be available by the end of a successful call.
    ///
    /// @return True on successfully completing a query.
    virtual bool onExecute() = 0;

    /// Virtual function called to execute the query asynchronously from @c executeAsync().
    ///
    /// Derived classes must implement an asynchronous query in this function returning only on failure or once
    /// the query has started. The call must fail when there is already an asynchronous query running from this
    /// instance.
    ///
    /// @return True on successfully starting an asynchronous query.
    virtual bool onExecuteAsync() = 0;

    // TODO: make pure virtual
    virtual bool onWaitAsync(unsigned timeout_ms);

    /// Called from @c reset(bool hardReset) to complete or terminate any
    /// outstanding asynchronous query and clear results.
    /// @param hard_reset True for a hard reset, false for a soft reset.
    virtual void onReset(bool hard_reset) = 0;

  protected:
    QueryDetail *imp_;
  };
}  // namespace ohm

#endif  // OCCUPANCYQUERY_H
