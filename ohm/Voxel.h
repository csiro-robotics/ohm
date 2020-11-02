// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHMVOXEL_H
#define OHMVOXEL_H

#include "OhmConfig.h"

#include "Key.h"
#include "MapChunk.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"
#include "VoxelBlock.h"

#include <cinttypes>
#include <type_traits>

#include <glm/vec3.hpp>

namespace ohm
{
namespace detail
{
/// Internal helper to manage updating const/mutable chunks with the base as the mutable version.
template <typename T>
struct VoxelChunkAccess
{
  /// Resolve a mutable chunk for @p key from @p map , creating the chunk if required.
  /// @param map The map of interest.
  /// @param key The key to resolve the chunk for.
  /// @return The chunk for @p key .
  static MapChunk *chunk(OccupancyMap *map, const Key &key) { return map->region(key.regionKey(), true); }

  /// Update the first valid index for @p chunk using @p voxel_index .
  /// @param chunk The map chunk being touched: must be valid.
  /// @param voxel_index The linear index of the modified voxel.
  static void touch(MapChunk *chunk, unsigned voxel_index) { chunk->updateFirstValid(voxel_index); }

  /// Mark @p chunk as having been updated within @p layer_index .
  /// This will @c OccupancyMap::touch() the @p map and update the stamps in @p chunk relevant to @p layer_index .
  /// @param map The map of interest.
  /// @param chunk The map chunk being touched: must be valid.
  /// @param layer_index The voxel memory index in chunk which has been modified.
  static void touch(OccupancyMap *map, MapChunk *chunk, int layer_index)
  {
    chunk->dirty_stamp = map->touch();
    chunk->touched_stamps[layer_index].store(chunk->dirty_stamp, std::memory_order_relaxed);
  }

  /// Write the @p value to the voxel at @p voxel_index within @p voxel_memory .
  /// @param voxel_memory Start of the voxel memory.
  /// @param voxel_index Index of the voxel within @p voxel_memory - strided by @c T .
  /// @param value The value to write.
  /// @param flags_change Flags to set in @p flags .
  /// @param flags Flags to modify by setting @p flags_change .
  static void writeVoxel(uint8_t *voxel_memory, unsigned voxel_index, const T &value, unsigned flags_change,
                         uint16_t *flags)
  {
    memcpy(voxel_memory + sizeof(T) * voxel_index, &value, sizeof(T));
    *flags |= flags_change;
  }
};

/// Internal helper to manage const chunks. Supports fetching existing chunks, but no modification.
template <typename T>
struct VoxelChunkAccess<const T>
{
  /// Query the @c MapChunk pointer for @p key.
  /// @param map The Occupancy map of interest
  /// @param key The key to get a chunk for.
  /// @return The @c MapChunk for key, or null if the chunk does not exist.
  static const MapChunk *chunk(const OccupancyMap *map, const Key &key) { return map->region(key.regionKey()); }

  /// Noop.
  /// @param chunk Ignored.
  /// @param voxel_index Ignored.
  static void touch(const MapChunk *chunk, unsigned voxel_index)
  {
    (void)chunk;
    (void)voxel_index;
  }

  /// Noop.
  /// @param map Ignored.
  /// @param chunk Ignored.
  /// @param layer_index Ignored.
  static void touch(const OccupancyMap *map, const MapChunk *chunk, int layer_index)
  {
    (void)map;
    (void)chunk;
    (void)layer_index;
  }

  static void writeVoxel(uint8_t voxel_memory, unsigned voxel_index, const T &value, unsigned flags_change,
                         uint16_t *flags) = delete;
};
}  // namespace detail

/// The @c Voxel interface provides a semi optimal abstraction and book keeping for accessing voxel data.
///
/// The @c Voxel interface deals directly with the @c MapLayer abstraction of the @c OccupancyMap , providing access
/// to the data within a single @p MapLayer . The template type is used to resolve the data within the layer as
/// the template type, supporting mutable and const access (see below). The template type is validated against
/// the data in the proposed layer index by checking the size of @c T against the size of the data stored in the
/// layer. The layer index is invalidated when the sizes do not match.
///
/// A mutable @c Voxel is one where the template type @c T is non-const, while a const @c Voxel has a const template
/// type @c T . Only a mutable @c Voxel can create new @c MapChunks within the @c OccupancyMap . This occurs
/// immediately on setting a key, generally via @c setKey() . Additional book keeping is managed by the mutable
/// @c Voxel to ensure correct update of @c OccupancyMap::stamp() (via @c OccupancyMap::touch() ) as well as
/// updating appropriate @c MapChunk stamps. The @c MapChunk::first_valid_index is also udpated for the occupancy
/// layer.
///
/// A @c Voxel is initialised to reference a specific @c MapLayer within a specific @c OccupancyMap . At this point
/// the @c Voxel should pass @c isValidLayer() to ensure that the map is valid, the layer reference is valid and
/// that the size of @c T matches the @c MayLayer voxel size. Specific voxels can then be referenced via @c setKey()
/// to identify which voxel to reference. This resolves the @c MapChunk , creating the chunk in a mutable @c Voxel
/// if required. A const @c Voxel will never create a new @c MapChunk and may have a valid @c Key , with a null
/// @c MapChunk .
///
/// From this we see that a @c Voxel may be in one of several states. Below we list the states and various methods
/// which can be used to check these states.
///
/// State description               | `isLayerValid()`  | `isValidReference()`  | `isValid()` | `errorFlags()`
/// ------------------------------- | ----------------  | --------------------- | ----------- | -------------
/// Null map                        | false             | false                 | false       | `NullMap`
/// `layerIndex()` out of range     | false             | false                 | false       | `InvalidLayerIndex`
/// `sizeof(T)!=voxelByteSize()` *  | false             | false                 | false       | `VoxelSizeMismatch`
/// Initialised                     | true              | false                 | false       | 0
/// Key set, chunk null (const only)| true              | true                  | false       | 0
/// Key set, chunk resovled         | true              | true                  | true        | 0
///
/// * see @c MapLayer::voxelByteSize()
///
/// There are various ways to set a @c Voxel to reference a specific voxel using @c setKey() or some constructors.
/// The @c setKey() function supports various overloads. The first accepts a @c Key which references a voxel. This
/// key is immediately used to resolve the @c MapChunk then the specific voxel @c data() on request. Setting to a key
/// in the same @c MapChunk as the current maintains the current @c MapChunk pointer. @c setKey() also accepts a
/// @c Key with a @c MapChunk for cases where the caller already has a reference to the correct @c MapChunk . This
/// chunk pointer is assumed to be correct and is not validated.
///
/// The key may also be set from another @c Voxel even one for a different layer. This copies the @c Key and
/// @c MapChunk from the other @c Voxel saving on a looking into the map to resolve the chunk. This call is validated
/// only to ensure that the @c OccupancyMap pointers match, setting @c Error::kMapMismatch on failure.
///
/// Finally a voxel reference may be set from an @c OccupancyMap::iterator for a mutable voxel or an
/// @c OccupancyMap::const_iterator for a const voxel. This call also assumes the @c MapChunk in the iterator is valid
/// and will save a looking into the map. Note that the iterator is assumed to be valid by the @c Voxel and is not
/// validated. That is, a @c Voxel must not be initialised from an @c end() or otherwise invalid iterator.
///
/// A similar set of parameterisation of the @c Voxel constructors also exist.
///
/// For convenience the free function @c setVoxelKey() may be used to initialise a number of @c Voxel references,
/// presumably to different layers, from a single key reference. This is a variadic template function accepting
/// at least two arguments. The first is the key reference object - a @c Key , iterator or valid @c Voxel reference -
/// followed by any number of @c Voxel objects. Take care to ensure that the first argument is always a valid key
/// reference.
///
/// For optimal memory access a @c MapChunk should be accessed as linearly as possible. This is mostly only applicable
/// to reading only access. For write access some gains are to be made by referencing voxels in a spatially coherent
/// fashion, ones which are likely to fall in the same @c MapChunk . This helps keep @c Voxel overheads to a
/// minimum, but direct access to and linear traversal of voxel data may be more optimal with manual book keeping
/// to manage the various stamps and @c MapChunk::first_valid_index .
///
/// Typical usage is illustrated in the example below, populating a line in the map and reading a line of voxels.
///
/// @code
/// void populateLine(ohm::OccupancyMap &map)
///{
///  // Create a voxel reference for accessing voxel occupancy.
///  ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
///  // Create a voxel reference for accessing VoxelMean.
///  ohm::Voxel<ohm::VoxelMean> mean(&map, map.layout().meanLayer());
///
///  // Ensure the occupancy layer is ok.
///  if (!occupancy.isLayerValid())
///  {
///    return;
///  }
///
///  glm::dvec3 sample(0, 0, 0);
///  for (int i = 0; i < 10; ++i)
///  {
///    sample.x = i * map.resolution();
///    const Key key = map.voxelKey(sample);
///
///    // Below we look at several methods for setting the keys for the Voxel objects. Note that they have equivalent
///    // results and only one need be used. The process of setting the Key instantiates the MapChunk in map.
///
///    // 1. Set each voxel individually. This has some slight additional overhead as each Voxel resolved the chunk.
///    occupancy.setKey(key);
///    mean.setKey(key);
///
///    // 2. Set one Voxel key directly, the subsequent items are chained and the chunk is resolved once.
///    mean.setKey(occupancy.setKey(key));
///
///    // 3. Equivalent, but slightly more readable version of 2. Note this version is open to mistakenly passing a
///    // Voxel for the first argument rather than a Key and not actually setting a key.
///    ohm::setVoxelKey(key, occupancy, mean);
///
///    // Next we look at some different ways of setting occupancy. Again, only 1 need be used.
///    // A) Set occupancy directly:
///    occupancy.data() = map.hitValue();
///
///    // B) Use helper function. Assumes occupancy.isValid() is true
///    ohm::integrateHit(occupancy);  // in VoxelOccupancy.h
///
///    // Finally update the voxel mean. Using both techniques below results in adding the sample twice, so the
///    // mean count will be 2.
///    // i. Safe version, with mean.isValid() may be false.
///    ohm::updatePositionSafe(mean, sample);  // in VoxelMean.h
///    // ii. Unchecked version, where mean.isValid() is assumed to be true. We check explicitly because we have not
///    // checked mean.isLayerValid() above.
///    if (mean.isValid())
///    {
///      ohm::updatePositionUnsafe(mean, sample);  // in VoxelMean.h
///    }
///  }
///}
///
/// void walkLine(const ohm::OccupancyMap &map)
///{
///  // Create a voxel reference for accessing voxel occupancy.
///  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
///  // Create a voxel reference for accessing VoxelMean.
///  ohm::Voxel<const ohm::VoxelMean> mean(&map, map.layout().meanLayer());
///
///  glm::dvec3 sample(0, 0, 0);
///  for (int i = 0; i < 12; ++i)  // iterate further than the original line
///  {
///    sample.x = i * map.resolution();
///    const Key key = map.voxelKey(sample);
///
///    // Use option 3 from above to set the key.
///    // This will not create the chunk if it does not exist because we are using const template types for
///    // Voxel.
///    ohm::setVoxelKey(key, occupancy, mean);
///
///    std::cout << "check map at (" << sample.x << "," << sample.y << "," << sample.z << ")" << std::endl;
///    std::cout << "occupancy: ";
///    if (!ohm::isUnobservedOrNull(occupancy))  // in VoxelOccupancy.h
///    {
///      std::cout << occupancy.data() << std::endl;
///    }
///    else
///    {
///      std::cout << "unobserved" << std::endl;
///    }
///
///    // The the voxel position, safely. This will be the voxel centre if VoxelMean is not supported.
///    const glm::dvec3 voxel_pos = ohm::positionSafe(mean);  // in VoxelMean.h
///    std::cout << "position: (" << voxel_pos.x << "," << voxel_pos.y << "," << voxel_pos.z << ")" << std::endl;
///  }
///}
/// @endcode
///
/// @note The map must outlive the voxel. Generally this will be true when using @c Voxel objects within a limited
/// scope. However care must be taken when an @c OccupancyMap and a @c Voxel reference to that map exist in the same
/// scope or when performing operations which can invalidate the @c MapLayout or clear the map. In these cases,
/// all @c Voxel objects should be explicitly released via @c Voxel::release() .
///
/// @tparam T The data type expected to be contained in the @c MapLayer to operate on. Checked for size match with
/// layer content. Note that compiler alignment of structures may cause mismatches between the data added to a
/// @c MapLayer and the size of @c T as @c MapLayer content will not be padded while @c T may be.
///
/// @todo Add validation debug compile flag which validate the various initalisations and data access calls.
template <typename T>
class ohm_API Voxel
{
public:
  /// Non-const data type for @c T
  using DataType = typename std::remove_const<T>::type;
  /// Const or non-const @c OccupancyMap iterator to support construction from an iterator. Saves on resolving
  /// the @c MapChunk again.
  using MapIteratorType =
    typename std::conditional<std::is_const<T>::value, OccupancyMap::const_iterator, OccupancyMap::iterator>::type;
  /// Const or non-const @c OccupancyMap pointer based on @c T constness
  using MapTypePtr = typename std::conditional<std::is_const<T>::value, const OccupancyMap *, OccupancyMap *>::type;
  /// Const or non-const @c MapChunk pointer based on @c T constness
  using MapChunkPtr = typename std::conditional<std::is_const<T>::value, const MapChunk *, MapChunk *>::type;
  /// Voxel data pointer - always a @c uint8_t type with @c const added if @c T is @c const .
  using VoxelDataPtr = typename std::conditional<std::is_const<T>::value, const uint8_t *, uint8_t *>::type;

  /// Error flag values indicating why initialisation may have failed.
  enum class Error : uint16_t
  {
    kNone = 0,                      ///< No error
    kNullMap = (1 << 0),            ///< @c OccupancyMap is null
    kInvalidLayerIndex = (1 << 1),  ///< The given layer index is invalid.
    kVoxelSizeMismatch = (1 << 2),  ///< The @c MapLayer voxel size does not match the size of @c T
    kMapMismatch = (1 << 3)         ///< When two @c Voxel object maps do not match such as in @c setKey() chaining.
  };

  /// Book keeping flags.
  enum class Flag : uint16_t
  {
    kNone = 0,                      ///< Nothing of note
    kIsOccupancyLayer = (1u << 0),  ///< Marks that this @c Voxel points to the occupancy layer of @c OccupancyMap.
    kCompressionLock = (1u << 1),   ///< Indiates the layer's @c VoxelBlock has been retained in @c chunk_.
    kTouchedChunk = (1u << 2),      ///< Marks that the current @c MapChunk data has been accessed for mutation.
    kTouchedVoxel = (1u << 3),      ///< Marks that the current voxel data has been accessed for mutation.

    /// Flag values which are not propagated in copy assigment.
    kNonPropagatingFlags = kTouchedChunk | kTouchedVoxel | kCompressionLock
  };

  /// Empty constructor generating an invalid voxel with no map.
  Voxel() = default;
  /// Copy constructor - same type.
  /// @param other The voxel to copy.
  Voxel(const Voxel<T> &other);
  /// RValue constructor - same type.
  /// @param other The voxel to copy.
  Voxel(Voxel<T> &&other);
  /// Copy constructor from a different type. Copies the map, chunk and key, but references a different layer/type.
  /// @param other The base voxel to copy.
  /// @param layer_index The @c MapLayer to access for the type @c T .
  template <typename U>
  Voxel(const Voxel<U> &other, int layer_index);
  /// Create a @c Voxel reference for @c map and the specified @c layer_index . The map and layer are validated
  /// (see @c Error flags) and @c isLayerValid() will be true on success. The key is not set so @c isValid() remains
  /// false.
  /// @param map The map to access and mutate for non-const @c Voxel types.
  /// @param layer_index The @c MapLayer to access for the type @c T .
  Voxel(MapTypePtr map, int layer_index);
  /// Create a @c Voxel reference for @c map and the specified @c layer_index and reference the voxel at @c key. The
  /// map and layer are validated (see @c Error flags) and @c isLayerValid() will be true on success. The key is also
  /// set and @c isValid() will be true for a mutable @c Voxel , creating the @c MapChunk if required. A non-mutable
  /// @c Voxel will still not be valid in the case where the @p key references a @c MapChunk which does not exist.
  /// @param map The map to access and mutate for non-const @c Voxel types.
  /// @param layer_index The @c MapLayer to access for the type @c T .
  /// @param key The key for the voxel to initialy reference. May be changed layer with @c setKey() .
  Voxel(MapTypePtr map, int layer_index, const Key &key);

  /// Set the @c Voxel to reference the start of the voxel buffer within the @c MapChunk associated with the given
  /// @p region_key . This is equivalent to using @c Key(region_key,0,0,0) .
  /// @param map The map to access and mutate for non-const @c Voxel types.
  /// @param layer_index The @c MapLayer to access for the type @c T .
  /// @param region_key The region coordinate key for the chunk to reference.
  Voxel(MapTypePtr map, int layer_index, const glm::i16vec3 &region_key);

  /// Create a @c Voxel reference from a @c OccupancyMap::iterator (mutable @c Voxel ) or a
  /// @c OccupancyMap::const_iterator (const @c Voxel ). This is similar to using the
  /// @c Voxel(MapTypePtr,layer_index,key) constructor, with the map, chunk and key always coming from the iterator.
  /// This will never create a @c MapChunk as it is assumed to be valid in the iterator.
  Voxel(const MapIteratorType &iter, int layer_index);

  /// Destrutor, ensures book keeping operations are completed on the @c MapChunk .
  inline ~Voxel() { updateTouch(); }

  /// Check if the map and layer references are valid and error flags are clear.
  /// @return True if the @c map() is not null, the @c layerIndex() is valid and @c errorFlags() are zero.
  inline bool isLayerValid() const { return map_ && layer_index_ >= 0 && error_flags_ == 0; }
  /// Check if the voxel reference is valid for @c data() calls.
  /// @return True if @c isLayerValid() and the @c chunk() and @c key() values are  non-null.
  inline bool isValid() const { return isLayerValid() && chunk_ && key_ != Key::kNull; }
  /// Check if the voxel reference is invalid.
  /// @return The logical negation of @c isValid()
  inline bool isNull() const { return isLayerValid() && (!chunk_ || key_ == Key::kNull); }
  /// Check if the @c Voxel is a valid voxel reference. This does not check the @c chunk() .
  /// @return True if @c isValidLayer() and the key is non-null.
  inline bool isValidReference() const { return isLayerValid() && key_ != Key::kNull; }

  /// Nullify the @c Voxel . This clears the map, layer, chunk and key values. Book keeping is performed as required.
  inline void reset() { *this = Voxel<T>(); }

  /// Query the pointer to the @c OccupancyMap .
  /// @return The map pointer.
  inline MapTypePtr map() const { return map_; }
  /// Query the pointer to the @c MapChunk .
  /// @return The chunk pointer.
  inline MapChunkPtr chunk() const { return chunk_; }
  /// Query the current @c Key reference.
  /// @return The current key value.
  inline Key key() const { return key_; }
  /// Query the index of the target @c MapLayer in the @c OccupancyMap .
  /// @return The map layer to target.
  inline int layerIndex() const { return layer_index_; }
  /// Query the cached @c MapLayer::dimensions() .
  /// @return The map layer voxel dimensions.
  inline glm::u8vec3 layerDim() const { return layer_dim_; }
  /// Query the status @c Flag values for the voxel. These are generally book keeping flags.
  /// @return The current status flags.
  inline unsigned flags() const { return flags_; }
  /// Query the status @c Error flag values for the voxel.
  /// @return The current error flags.
  inline unsigned errorFlags() const { return error_flags_; }

  /// Set the voxel @c Key . This may create a @c MapChunk for a mutable @c Voxel .
  /// @param key The key for the voxel to reference. Must be non-null and in range.
  /// @return `*this`
  Voxel<T> &setKey(const Key &key);
  /// Set the voxel @c Key with a pre-resolved @c MapChunk .
  /// @param key The key for the voxel to reference. Must be non-null and in range.
  /// @param chunk A pointer to the correct chunk for the @c Key . This must be the correct chunk.
  /// @return `*this`
  Voxel<T> &setKey(const Key &key, MapChunkPtr chunk);
  template <typename U>
  /// Set the voxel key from another @c Voxel reference. This copies the @c MapChunk from @p other .
  ///
  /// Will set the @c Error::kMapMismatch flag if the map pointers do not match between @c this and @p other.
  /// @param other The voxel to initialise from.
  /// @return `*this`
  Voxel<T> &setKey(const Voxel<U> &other);
  /// Set the voxel reference from an @c OccupancyMap::iterator (mutable) or @c OccupancyMap::const_iterator (const).
  /// @param iter The iterator to set the voxel reference from. Must be a valid voxel iterator.
  /// @return `*this`
  Voxel<T> &setKey(const MapIteratorType &iter);

  /// Resolve the linearised voxel index into the @c MapChunk layer. @c isValidReference() must be true before
  /// calling.
  /// @return The linear voxel index resolved from the key.
  inline unsigned voxelIndex() const { return ohm::voxelIndex(key_, layer_dim_); }

  /// Access the data for the current voxel. This is a convenience wrapper for the @c read() function which returns
  /// the template data type. Only call if @c isValid() is true.
  /// @return The data for the current voxel.
  DataType data() const
  {
    DataType d;
    read(&d);
    return d;
  }

  /// Read the current voxel data value.
  ///
  /// This reads the data for the current @c voxelIndex() into @p value . This call does no error checking and
  /// assumes that all pointers and index values have been validated.
  ///
  /// @param[out] value Pointer where to write the voxel data of type @c T for the currently referenced voxel.
  /// @return The read value - i.e., `*value`.
  inline const DataType &read(DataType *value) const
  {
    memcpy(value, voxel_memory_ + sizeof(T) * voxelIndex(), sizeof(T));
    return *value;
  }

  /// Write the current voxel data @p value .
  ///
  /// This writes @p value to the current @c voxelIndex() location. This call does no error checking and
  /// assumes that all pointers and index values have been validated. This can only be called if @c T is a non const
  /// type.
  ///
  /// @param[in] value Value to write for the current voxel.
  inline void write(const DataType &value)
  {
    detail::VoxelChunkAccess<T>::writeVoxel(voxel_memory_, voxelIndex(), value,
                                            unsigned(Flag::kTouchedChunk) | unsigned(Flag::kTouchedVoxel), &flags_);
  }

  /// Return a pointer to the start of the voxel memory for the current chunk.
  /// @c isValid() must be true before calling.
  /// @return A pointer to the voxel memory for the currently referenced chunk.
  inline VoxelDataPtr voxelMemory() { return voxel_memory_; }

  /// Attempt to step the voxel reference to the next voxel in the current @c MapChunk .
  ///
  /// This first validates @c isValidReference() before attempting to modify the @c key() . On success, the
  /// key will reference the next @c voxelIndex() in the @c MapChunk .
  ///
  /// @return True on success.
  bool nextInRegion();

  /// Swap this voxel content with @p other .
  /// @param other The voxel to exchange data with.
  void swap(Voxel &other);

  /// Assignment operator. Performs book keeping before assignment.
  ///
  /// Note that some @c Flag values will not be copied by the assignment. See @c Flag::kNonPropagatingFlags .
  /// @param other The voxel reference to assign from.
  /// @return `*this`
  Voxel &operator=(const Voxel<T> &other);
  /// Move assignent operator.
  /// @param other The rvalue reference to move data from.
  /// @return `*this`
  Voxel &operator=(Voxel<T> &&other);

protected:
  /// Internal key set function. Performs book keeping for @c Flag::kIsOccupancyLayer and @c Flag::kTouchedVoxel .
  /// @param key The key value to set.
  inline void setKeyInternal(const Key &key)
  {
    updateVoxelTouch();
    key_ = key;
  }

  /// Internal chunk set function. Performs book keeping for @c Flag::kTouchedChunk .
  /// @param chunk The chunk to set the voxel to reference.
  inline void setChunk(MapChunkPtr chunk)
  {
    if (chunk_ != chunk)
    {
      updateChunkTouchAndCompression();
      chunk_ = chunk;
      if (chunk_ && layer_index_ != -1)
      {
        chunk_->voxel_blocks[layer_index_]->retain();
        flags_ |= unsigned(Flag::kCompressionLock);
        voxel_memory_ = chunk_->voxel_blocks[layer_index_]->voxelBytes();
      }
      else
      {
        voxel_memory_ = nullptr;
      }
    }
  }

  /// Validate the @c layerIndex() . May invalidate the layer index and set @c Error flag values.
  void validateLayer();
  /// Perform book keeping for the currently referenced voxel. Handles @c Flag::kIsOccupancyLayer and
  /// @c Flag::kTouchedVoxel . This only needs to do work when @c Flag::kIsOccupancyLayer is true by updating
  /// @c MapChunk::first_valid_index the @c Flag::kTouchedVoxel has been set. @c Flag::kTouchedVoxel is cleared.
  ///
  /// Safe to call when no book keeping needs to be done or the voxel reference is invalid.
  void updateVoxelTouch();
  /// Perform book keeping for the current chunk. Handles @c Flag::kTouchedChunk by ensuring @c OccupancyMap::touch()
  /// is called then updating the @c MapChunk::dirty_stamp and the @c MapChunk::touched_stamps for the referenced
  /// layer. @c Flag::kTouchedChunk is cleared.
  ///
  /// Safe to call when no book keeping needs to be done or the voxel reference is invalid.
  void updateChunkTouchAndCompression();
  /// Full book keeping calling @c updateVoxelTouch() and @c updateChunkTouchAndCompression() .
  ///
  /// Safe to call when no book keeping needs to be done or the voxel reference is invalid.
  inline void updateTouch()
  {
    updateVoxelTouch();
    updateChunkTouchAndCompression();
  }

private:
  VoxelDataPtr voxel_memory_ = nullptr;  ///< A pointer to the start of voxel memory within the current chunk.
  MapTypePtr map_ = nullptr;             ///< @c OccupancyMap pointer
  MapChunkPtr chunk_ = nullptr;          ///< Current @c MapChunk pointer - may be null even with a valid key reference.
  Key key_ = Key::kNull;                 ///< Current voxel @c Key reference.
  int layer_index_ = -1;                 ///< The target map layer. Validated on construction.
  glm::u8vec3 layer_dim_{ 0, 0, 0 };     ///< The voxel dimensions of the layer.
  uint16_t flags_ = 0;                   ///< Current status/book keeping flags
  uint16_t error_flags_ = 0;             ///< Current error flags.
};


/// @overload
template <typename KeyType, typename T>
void setVoxelKey2(const KeyType &key, Voxel<T> &voxel)
{
  voxel.setKey(key);
}

/// A less safe version of @c setVoxelKey(). This version isn't as safe as it accepts any @c KeyType , which opens up
/// up bugs where the first argument is an invalid @c Voxel reference, rather than the expected usage where the first
/// argument is a @c Key or @c OccupancyMap::iterator .
/// @param key The object from which to set the key value. Expected to be a @c Key , @c OccupancyMap::iterator or
/// @c OccupancyMap::const_iterator
/// @param voxel The first @c Voxel to set the key for.
/// @param args Additional @c Voxel references.
template <typename KeyType, typename T, typename... Args>
void setVoxelKey2(const KeyType &key, Voxel<T> &voxel, Args &... args)
{
  setVoxelKey2(voxel.setKey(key), args...);
}

/// @overload
template <typename T>
void setVoxelKey(const Key &key, Voxel<T> &voxel)
{
  voxel.setKey(key);
}

/// @overload
template <typename T>
void setVoxelKey(const OccupancyMap::iterator &iter, Voxel<T> &voxel)
{
  voxel.setKey(iter);
}

/// @overload
template <typename T>
void setVoxelKey(const OccupancyMap::const_iterator &iter, Voxel<T> &voxel)
{
  voxel.setKey(iter);
}

/// Set the key value for multiple @c Voxel objects with reduced overhead by resolving the @c MapChunk once.
/// @param key The object from which to set the key value.
/// @param voxel The first @c Voxel to set the key for.
/// @param args Additional @c Voxel references.
template <typename T, typename... Args>
void setVoxelKey(const Key &key, Voxel<T> &voxel, Args &... args)
{
  setVoxelKey2(key, voxel, args...);
}

/// Set the key value for multiple @c Voxel from a map iterator.
/// @param iter The occupancy map iterator to set from.
/// @param voxel The first @c Voxel to set the key for. Must be a mutable voxel.
/// @param args Additional @c Voxel references.
template <typename T, typename... Args>
void setVoxelKey(const OccupancyMap::iterator &iter, Voxel<T> &voxel, Args &... args)
{
  setVoxelKey2(iter, voxel, args...);
}

/// Set the key value for multiple @c Voxel from a map iterator.
/// @param iter The occupancy map iterator to set from.
/// @param voxel The first @c Voxel to set the key for. Must be a const voxel.
/// @param args Additional @c Voxel references.
template <typename T, typename... Args>
void setVoxelKey(const OccupancyMap::const_iterator &iter, Voxel<T> &voxel, Args &... args)
{
  setVoxelKey2(iter, voxel, args...);
}


template <typename T>
Voxel<T>::Voxel(const Voxel<T> &other)
  : map_(other.map_)
  , chunk_(nullptr)
  , key_(other.key_)
  , layer_index_(other.layer_index_)
  , layer_dim_(other.layer_dim_)
  , flags_(other.flags_ & ~unsigned(Flag::kNonPropagatingFlags))
  , error_flags_(other.error_flags_)
{
  // Do not set chunk or voxel_memory_ pointers directly. Use the method call to ensure flags are correctly
  // maintained.
  setChunk(other.chunk_);
}


template <typename T>
Voxel<T>::Voxel(Voxel<T> &&other)
  : voxel_memory_(std::exchange(other.voxel_memory_, nullptr))
  , map_(std::exchange(other.map_, nullptr))
  , chunk_(std::exchange(other.chunk_, nullptr))
  , key_(std::exchange(other.key_, Key::kNull))
  , layer_index_(std::exchange(other.layer_index_, -1))
  , layer_dim_(std::exchange(other.layer_dim_, glm::u8vec3(0, 0, 0)))
  , flags_(std::exchange(other.flags_, 0u))
  , error_flags_(std::exchange(other.error_flags_, 0u))
{}


template <typename T>
template <typename U>
Voxel<T>::Voxel(const Voxel<U> &other, int layer_index)
  : map_(other.map())
  , chunk_(nullptr)
  , key_(other.key())
  , layer_index_(layer_index)
  , flags_(other.flags() & ~unsigned(Flag::kNonPropagatingFlags))
  , error_flags_(other.errorFlags())
{
  validateLayer();
  // Do not set chunk or voxel_memory_ pointers directly. Use the method call to ensure flags are correctly
  // maintained.
  setChunk(other.chunk());
}


template <typename T>
Voxel<T>::Voxel(MapTypePtr map, int layer_index)
  : map_(map)
  , layer_index_(layer_index)
{
  validateLayer();
}


template <typename T>
Voxel<T>::Voxel(MapTypePtr map, int layer_index, const Key &key)
  : Voxel<T>(map, layer_index)
{
  if (map)
  {
    setKey(key);
  }
}


template <typename T>
Voxel<T>::Voxel(MapTypePtr map, int layer_index, const glm::i16vec3 &region_key)
  : Voxel<T>(map, layer_index, Key(region_key, 0, 0, 0))
{}


template <typename T>
Voxel<T>::Voxel(const MapIteratorType &iter, int layer_index)
  : Voxel<T>(iter.map(), layer_index, iter.key())
{
  setChunk(iter.chunk());
}


template <typename T>
Voxel<T> &Voxel<T>::setKey(const Key &key)
{
  setKeyInternal(key);
  if (!chunk_ || chunk_->region.coord != key.regionKey())
  {
    // Create chunk if not read only access.
    setChunk(detail::VoxelChunkAccess<T>::chunk(map_, key));
  }
  return *this;
}


template <typename T>
Voxel<T> &Voxel<T>::setKey(const Key &key, MapChunkPtr chunk)
{
  setKeyInternal(key);
  setChunk(chunk);
  return *this;
}


template <typename T>
Voxel<T> &Voxel<T>::setKey(const MapIteratorType &iter)
{
  setKeyInternal(*iter);
  setChunk(iter.chunk());
  return *this;
}


template <typename T>
template <typename U>
Voxel<T> &Voxel<T>::setKey(const Voxel<U> &other)
{
  setKeyInternal(other.key());
  setChunk(other.chunk());
  error_flags_ |= !(map_ == other.map()) * unsigned(Error::kMapMismatch);
  return *this;
}


template <typename T>
bool Voxel<T>::nextInRegion()
{
  if (isValidReference())
  {
    if (key_.localKey().x + 1 == layer_dim_.x)
    {
      if (key_.localKey().y + 1 == layer_dim_.y)
      {
        if (key_.localKey().z + 1 == layer_dim_.z)
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

  return false;
}


template <typename T>
void Voxel<T>::swap(Voxel &other)
{
  std::swap(voxel_memory_, other.voxel_memory_);
  std::swap(map_, other.map_);
  std::swap(chunk_, other.chunk_);
  std::swap(key_, other.key_);
  std::swap(layer_index_, other.layer_index_);
  std::swap(layer_dim_, other.layer_dim_);
  std::swap(flags_, other.flags_);
  std::swap(error_flags_, other.error_flags_);
}


template <typename T>
Voxel<T> &Voxel<T>::operator=(const Voxel<T> &other)
{
  updateTouch();
  // Set chunk to null to run flush logic
  setChunk(nullptr);
  map_ = other.map_;
  setKeyInternal(other.key_);
  layer_index_ = other.layer_index_;
  layer_dim_ = other.layer_dim_;
  flags_ = other.flags_ & ~unsigned(Flag::kNonPropagatingFlags);
  error_flags_ = other.error_flags_;
  // Do not set chunk or voxel_memory_ pointers directly. Use the method call to ensure flags are correctly
  // maintained.
  voxel_memory_ = nullptr;  // About to be resolved in setChunk()
  setChunk(other.chunk_);
  return *this;
}


template <typename T>
Voxel<T> &Voxel<T>::operator=(Voxel<T> &&other)
{
  Voxel<T> copy(std::move(other));
  swap(copy);
  return *this;
}


template <typename T>
void Voxel<T>::validateLayer()
{
  if (layer_index_ >= 0)
  {
    const MapLayer *layer = map_ ? map_->layout().layerPtr(layer_index_) : nullptr;
    if (!map_)
    {
      error_flags_ |= unsigned(Error::kNullMap);
    }
    else if (!layer)
    {
      // Invalid layer.
      error_flags_ |= unsigned(Error::kInvalidLayerIndex);
    }
    else if (layer->voxelByteSize() != sizeof(T))
    {
      // Incorrect layer size.
      error_flags_ |= unsigned(Error::kVoxelSizeMismatch);
    }
    else
    {
      layer_dim_ = layer->dimensions(map_->regionVoxelDimensions());
    }

    flags_ &= ~unsigned(Flag::kIsOccupancyLayer);
    flags_ |= (layer && map_ && layer_index_ == map_->layout().occupancyLayer()) * unsigned(Flag::kIsOccupancyLayer);
  }
}


template <typename T>
void Voxel<T>::updateVoxelTouch()
{
  if ((flags_ & unsigned(Flag::kIsOccupancyLayer) | unsigned(Flag::kTouchedVoxel)) && chunk_)
  {
    detail::VoxelChunkAccess<T>::touch(chunk_, voxelIndex());
  }
  flags_ &= ~unsigned(Flag::kTouchedVoxel);
}


template <typename T>
void Voxel<T>::updateChunkTouchAndCompression()
{
  if (map_ && chunk_)
  {
    if (flags_ & unsigned(Flag::kTouchedChunk))
    {
      detail::VoxelChunkAccess<T>::touch(map_, chunk_, layer_index_);
    }
    if (flags_ & unsigned(Flag::kCompressionLock))
    {
      chunk_->voxel_blocks[layer_index_]->release();
    }
  }
  flags_ &= ~(unsigned(Flag::kTouchedChunk) | unsigned(Flag::kCompressionLock));
}
}  // namespace ohm

#endif  // OHMVOXEL_H
