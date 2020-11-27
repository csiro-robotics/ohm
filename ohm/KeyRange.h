// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_KEYRANGE_H
#define OHM_KEYRANGE_H

#include "OhmConfig.h"

#include "Key.h"

#include <glm/vec3.hpp>

namespace ohm
{
class OccupancyMap;

class KeyRangeIterator;

/// A utility class which represents a key range in a map. The range supports iteration and volume operations.
///
/// Note that a @c KeyRange always behaves as a closed interval. That is, the range defines a rectantular prism from
/// the @c minKey() to the @c maxKey() including the maximal key. This is illustrated in 2D below.
///
/// @code{.unparsed}
///
///   +-----------------------------M
///   |                             |
///   |                             |
///   |                             |
///   |                             |
///   |                             |
///   |                             |
///   m-----------------------------+
///
/// - m is the minimum key in the range.
/// - M is the maximum key in the range.
/// - The far left and the top border are included when iterating the key range as well as the max key (M).
///
/// @endcode{.unparsed}
///
/// Also note, a key range is bound to the map for which it was created. This is because of the way that the region
/// voxel dimensions affect key iteration.
class ohm_API KeyRange
{
public:
  using iterator = KeyRangeIterator;  // NOLINT(readability-identifier-naming)

  /// Default constructor. The created range consists of null min/max keys is not valid.
  KeyRange() = default;

  /// Create a range between the given keys. The resulting range is only valid if the given @p min_key and @p max_key
  /// values are correctly configured to define a zero or increasing range along each axis. Otherwise @c isValid() will
  /// be false.
  /// @param min_key The minimul bounds for the range to define.
  /// @param max_key The maximul bounds for the range to define.
  /// @param region_voxel_dim Defines the number of voxels in each region of the owning map.
  inline KeyRange(const Key &min_key, const Key &max_key, const glm::u8vec3 &region_voxel_dim)
    : min_key_(min_key)
    , max_key_(max_key)
    , region_dimensions_(region_voxel_dim)
  {
    updateEndKey();
  }

  /// Create a range between the given keys for the specified map. The resulting range is only valid if the given @p
  /// min_key and @p max_key values are correctly configured to define a zero or increasing range along each axis.
  /// Otherwise @c isValid() will be false.
  /// @param min_key The minimul bounds for the range to define.
  /// @param max_key The maximul bounds for the range to define.
  /// @param map Defines the map to which the range belongs. This sets the @c regionDimensions() but no reference or
  /// pointer to the map is retained.
  KeyRange(const Key &min_key, const Key &max_key, const ohm::OccupancyMap &map);

  /// Copy constructor
  /// @param other Range to copy.
  inline KeyRange(const KeyRange &other) = default;

  /// Assignment opreator.
  /// @param other Range to copy.
  inline KeyRange &operator=(const KeyRange &other) = default;

  /// Create an iterator to start iterating the range.
  ///
  /// @note Iteration of an invalid range - where @c isValid() is false - creates undefined behaviour.
  ///
  /// @return An iterator object referencing the @c minKey() .
  iterator begin() const;

  /// Create an interation which references the voxel one step after the @c maxKey() .
  ///
  /// @note Iteration of an invalid range - where @c isValid() is false - creates undefined behaviour.
  ///
  /// @return An iterator object referencing the key one step beyond @c maxKey() .
  iterator end() const;

  /// Query the minimum key for the range.
  /// @return The minimum key value.
  inline const Key &minKey() const { return min_key_; }
  /// Set the minimum key value for the range.
  /// @note No validation is performed by this call.
  /// @param key The new value for @c minKey().
  inline void setMinKey(const Key &key)
  {
    min_key_ = key;
    updateEndKey();
  }
  /// Query the maximum key for the range.
  /// @return The maximum key value.
  inline const Key &maxKey() const { return max_key_; }
  /// Set the maximum key value for the range.
  /// @note No validation is performed by this call.
  /// @note This may invalidate any current iterator objects referencing this range.
  /// @param key The new value for @c maxKey().
  inline void setMaxKey(const Key &key)
  {
    max_key_ = key;
    updateEndKey();
  }

  /// Expand the key range to include the given @p key.
  ///
  /// No change is made if @p key is null. For each of min/max keys which are null, they are set to match @p key.
  ///
  /// @param key The key to include in the range.
  void expand(const Key &key);

  /// Expand the key range to include the given @p range.
  ///
  /// No change is made by each null source key from @p range. For each of min/max keys in this range which are null,
  /// they are set to match the input range keys.
  ///
  /// @param range The other @c KeyRange to include in this one.
  inline void expand(const KeyRange &range)
  {
    expand(range.minKey());
    expand(range.maxKey());
  }

  /// Query the key value used to represent the end of iteration.
  ///
  /// The resulting key references a single step beyond the @c maxKey() value. The resulting key has it's X and Y axes
  /// equal to those of the @c minKey() while the Z axis is one step greater than the @c maxKey() range. This matches
  /// the way key ranges are iterated.
  ///
  /// @note The result is undefined for an invalid range.
  ///
  /// @return The key value representing the end of iteration.
  inline const Key &endKey() const { return end_key_; }

  /// Query the region dimensions which this range operates on. See @c OccupancyMap::regionVoxelDimensions().
  /// @return A vector defining the number of voxels along each axis for a @c MapRegion.
  inline const glm::u8vec3 &regionDimensions() const { return region_dimensions_; }
  /// Set the @c regionDimensions() value. For expert use.
  /// @param dim The new @c regionDimensions() value.
  inline void setRegionDimensions(const glm::u8vec3 &dim) { region_dimensions_ = dim; }

  /// Query if the selected min/max keys create a valid range.
  ///
  /// A range is invalid if:
  /// - @c minKey().isNull() is true
  /// - @c maxKey().isNull() is true
  /// - The range defined by @c minKey() to @c maxKey() is not positive.
  ///
  /// @return True if this object defines a valid range.
  inline bool isValid() const
  {
    const auto r = range();
    return !min_key_.isNull() && !max_key_.isNull() && r.x > 0 && r.y > 0 && r.z > 0;
  }

  /// Query the number of voxels along each axis in the range.
  ///
  /// A range defines a closed interval, so a range where @c minkey() is equal to @c maxKey() will have a @c range()
  /// value of `(1, 1, 1)`.
  ///
  /// @note Behaviour is undefined for an invalid @c KeyRange.
  ///
  /// @return The number of voxels along each axis which this range encloses.
  glm::ivec3 range() const;

  /// Query the number of voxels in the enclosed region.
  /// @note Behaviour is undefined for an invalid @c KeyRange.
  /// @return The number of voxels contained in the range.
  inline int volume() const
  {
    const auto r = range();
    return r.x * r.y * r.z;
  }

  /// Calculate a 1D index for the given @p key in this range.
  /// Indexing runs along each X coordinate first, then Y then Z.
  /// @param key The key to query the index of. Assumed to be in the range.
  /// @return The 1D index of @p key in this range or zero if this range is invalid.
  size_t indexOf(const Key &key) const;

  /// Performs the reverse operation of @c indexOf(), yielding a key from a 1D offset into this range.
  /// @param index The index of this range. Assumed to be in the range.
  /// @return The key coresponding to @p index or a null key if this range is invalid.
  Key fromIndex(size_t index) const;

  /// Walk the given @p key to the next key in this range. Note: this is not bounds checked along Z and will continue
  /// to iterate the XY region beyond the range in Z. This function is intended for internal use to support
  /// @c KeyRangeIterator.
  /// @param[in,out] key The key to walk to the next voxel.
  inline void walkNext(Key &key) const { return walkNext(key, min_key_, max_key_, region_dimensions_); }

  /// Static utility function to walk the given @p key to the next key in this range. Note: this is not bounds checked
  /// along Z and will continue to iterate the XY region beyond the range in Z. This function is intended for internal
  /// use to support @c KeyRangeIterator.
  /// @param[in,out] key The key to walk to the next voxel.
  /// @param min_key The minimal key of the range to walk.
  /// @param max_key The maximal (inclusive) key of the range to walk.
  /// @param region_dimensions Defines the extents of a region in the map to which the defined range belongs.
  static void walkNext(Key &key, const Key &min_key, const Key &max_key, const glm::ivec3 &region_dimensions);

private:
  /// Update the value of @c end_key_. To be called whenever the min or max keys change.
  void updateEndKey();

  Key min_key_ = Key::kNull;       ///< The minimal/first key in the range.
  Key max_key_ = Key::kNull;       ///< The maximal/last key in the range.
  Key end_key_ = Key::kNull;       ///< A cached version of the end key - essentially the `max_key_ + 1`
  glm::u8vec3 region_dimensions_;  ///< Defines the number of voxels in each region for the map to which this belongs.
};

/// Defines an object used to iterate a @c KeyRange. It is assumed that the range does not change during iteration.
class ohm_API KeyRangeIterator
{
public:
  /// Default constructor. The resulting iterator is not valid.
  inline KeyRangeIterator() = default;

  /// Create an iterator object targetting the given @p range starting at @p initial_key.
  ///
  /// The @c initial_key is assumed to either be contained by the range, or to be the range object's @c endKey() value.
  ///
  /// @param range The range object to iterate.
  /// @param initial_key The key to start iterating at or the @c range.endKey() value.
  inline KeyRangeIterator(const KeyRange &range, const Key &initial_key)
    : key_(initial_key)
    , range_(range)
  {}

  /// Create an iterator object targetting the given @p range starting at the range object's @c minKey() value.
  /// @param range The range to begin iterating.
  explicit inline KeyRangeIterator(const KeyRange &range)
    : KeyRangeIterator(range, range.minKey())
  {}

  /// Copy constructor.
  /// @param other The iterator object to copy.
  inline KeyRangeIterator(const KeyRangeIterator &other) = default;
  /// Assignment operator.
  /// @param other The iterator object value to assign.
  inline KeyRangeIterator &operator=(const KeyRangeIterator &other) = default;

  /// Equality operator.
  /// @return True if the @c key() exactly matches @p other.key(). The associated ranges are not checked.
  inline bool operator==(const KeyRangeIterator &other) const { return other.key() == key(); }
  /// Inequality operator.
  /// @return True if the @c key() does not exactly matche @p other.key(). The associated ranges are not checked.
  inline bool operator!=(const KeyRangeIterator &other) const { return other.key() != key(); }

  /// Query the current key.
  /// @return The current key value.
  inline Key key() const { return key_; }

  /// Dereference the iterator as a key.
  /// @return The current voxel key.
  inline const Key &operator*() const { return key_; }

  /// Dereference the iterator as a key.
  /// @return The current voxel key.
  inline const Key *operator->() const { return &key_; }

  /// Prefix increment for the iterator. Iterator becomes invalid when incrementing an iterator
  /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
  /// @return This iterator after the increment.
  inline KeyRangeIterator &operator++()
  {
    range_.walkNext(key_);
    return *this;
  }

  /// Postfix increment for the iterator. Iterator becomes invalid when incrementing an iterator
  /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
  /// @return @c This iterator before the increment.
  inline KeyRangeIterator operator++(int)
  {
    const auto iter = *this;
    range_.walkNext(key_);
    return iter;
  }

private:
  Key key_ = Key::kNull;
  KeyRange range_{};
};  // namespace ohm


inline KeyRange::iterator KeyRange::begin() const
{
  return KeyRange::iterator(*this);
}


inline KeyRange::iterator KeyRange::end() const
{
  return KeyRange::iterator(*this, endKey());
}
}  // namespace ohm

#endif  // OHM_KEYRANGE_H
