// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_COPYUTIL_H
#define OHM_COPYUTIL_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

namespace ohm
{
class OccupancyMap;
struct MapChunk;

/// Filter used to determine if a chunk should be copied. Note that an empty filter always indicates an implied pass.
/// This is relevant to logical operations applied to filter functions.
using CopyChunkFilter = std::function<bool(const MapChunk &chunk)>;

/// Filter used to determine if a layer should be included when copying a map.
using CopyLayerFilter = std::function<bool(const std::string &layer_name)>;

/// Create a copy filter function which passes any @c MapChunk which overlaps the given axis aligned extents.
/// @param min_ext The minimum extents value (not validated).
/// @param max_ext The maximum extents value (not validated).
/// @return The filtering function object.
CopyChunkFilter ohm_API copyFilterExtents(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext);

/// Create a copy filter function which passes any @c MapChunk with a @c MapChunk::dirty_stamp greater than
/// @p after_stamp .
/// @param after_stamp The stamp value at/after which to pass chunks.
/// @return The filtering function object.
CopyChunkFilter ohm_API copyFilterStamp(uint64_t after_stamp);

/// Returns a copy filter function which passes if both @c filter_a and @c filter_b pass.
///
/// Note that if either @c filter_a or @c filter_b are empty, then the other object is required. It emerges that an
/// empty filter object is returned when both are empty.
///
/// @param filter_a The first filter function object.
/// @param filter_a The second filter function object.
/// @return A filter function which passes if both given filters pass. May be empty if both arguments are empty.
inline CopyChunkFilter ohm_API copyFilterAnd(const CopyChunkFilter &filter_a, const CopyChunkFilter &filter_b)
{
  if (!filter_a || !filter_b)
  {
    return (filter_a) ? filter_a : filter_b;
  }
  return [filter_a, filter_b](const MapChunk &chunk) { return filter_a(chunk) && filter_b(chunk); };
}

/// Returns a copy filter function which passes if either @c filter_a or @c filter_b pass.
///
/// Note that if either @c filter_a or @c filter_b are empty, then an empty filter object is returned which implies
/// always passing.
///
/// @param filter_a The first filter function object.
/// @param filter_a The second filter function object.
/// @return A filter function which passes if either given filters pass. May be empty if both either argument is empty.
inline CopyChunkFilter ohm_API copyFilterOr(const CopyChunkFilter &filter_a, const CopyChunkFilter &filter_b)
{
  if (!filter_a || !filter_b)
  {
    // In the case one filter is empty, that's an implied pass. We simply return an empty filter.
    return {};
  }
  return [filter_a, filter_b](const MapChunk &chunk) { return filter_a(chunk) || filter_b(chunk); };
}

/// Returns a copy filter function which negates the results of @p filter . Remember that an empty filter is an implied
/// pass, so the negation of an empty filter always returns @c false .
/// @param filter The filter to negate the results of.
/// @return A filter function which negates the results of @p filter .
inline CopyChunkFilter ohm_API copyFilterNegate(const CopyChunkFilter &filter)
{
  if (filter)
  {
    return [filter](const MapChunk &chunk) { return !filter(chunk); };
  }
  return [](const MapChunk &chunk) {
    (void)chunk;
    return false;
  };
}

/// A layer filter function for @c copyMap() which passes the selected @p layers .
/// @param layers The set of allowed layers.
/// @return A filter function for the given layers.
inline CopyLayerFilter ohm_API copyLayersFilter(const std::vector<std::string> &layers)
{
  return [layers](const std::string &layer_name) {
    return std::find(layers.begin(), layers.end(), layer_name) != layers.end();
  };
}

/// Returns a copy filter function which negates the results of @p filter . Remember that an empty filter is an implied
/// pass, so the negation of an empty filter always returns @c false .
/// @param filter The filter to negate the results of.
/// @return A filter function which negates the results of @p filter .
inline CopyLayerFilter ohm_API copyFilterNegate(const CopyLayerFilter &filter)
{
  if (filter)
  {
    return [filter](const std::string &layer_name) { return !filter(layer_name); };
  }
  return [](const std::string &layer_name) {
    (void)layer_name;
    return false;
  };
}

/// Validate that it is possible to copy data from this map to @p other .
///
/// The requirements are:
///
/// - Cannot copy to self.
/// - Both maps must:
///   - have the same resolution
///   - have the same region size
///   - have the same origin
/// - The @p dst map is not being modifies by another thread (not enforced).
///
/// @param src The map to copy from.
/// @param dst The map to copy do.
bool ohm_API canCopy(const OccupancyMap &dst, const OccupancyMap &src);

/// Copy data from this map to @p other, copying only the regions in the range `[min_ext, max_ext]`. Only layers
/// which are present in both maps are copied.
///
/// There are some specific conditions which must be met in order for this to work:
/// - @c canCopy() must pass.
/// - The maps must have common map layers matched by name and voxel layout.
///
/// @note This is not currently threadsafe.
///
/// @param dst The map to copy into.
/// @param src The map to copy from.
/// @param copy_chunk_filter Optional @c MapChunk filter to apply restricting what is copied.
/// @param copy_layer_filter Optional filter function for determining which layers are copied from the source map,
/// provided they exist in the destination map. Only source layers which pass the filter function are copied.
bool ohm_API copyMap(OccupancyMap &dst, const OccupancyMap &src, const CopyChunkFilter &copy_chunk_filter = {},
                     const CopyLayerFilter &copy_layer_filter = {});
}  // namespace ohm

#endif  // OHM_COPYUTIL_H
