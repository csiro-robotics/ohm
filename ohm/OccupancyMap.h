// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYMAP_H
#define OCCUPANCYMAP_H

#include "OhmConfig.h"

#include "Key.h"
#include "MapFlag.h"
#include "MapProbability.h"
#include "OccupancyType.h"
#include "RayFilter.h"
#include "RayFlag.h"

#include <glm/glm.hpp>

#include <array>
#include <functional>
#include <vector>

#define OHM_DEFAULT_CHUNK_DIM_X 32
#define OHM_DEFAULT_CHUNK_DIM_Y OHM_DEFAULT_CHUNK_DIM_X
#define OHM_DEFAULT_CHUNK_DIM_Z OHM_DEFAULT_CHUNK_DIM_X

namespace ohm
{
class Aabb;
class KeyList;
class KeyRange;
struct MapChunk;
class MapInfo;
class MapLayout;
struct OccupancyMapDetail;
class RayFilter;

/// A spatial container using a voxel representation of 3D space.
///
/// The map is divided into rectangular regions of voxels. The map uses an initial spatial hash to identify a larger
/// @c MapRegion which contains a contiguous array of voxels. This is used rather than an octree to support:
/// - Fast region allocation/de-allocation
/// - Constant lookup
/// - Dropping regions
/// - Eventually, out of core serialisation.
///
/// The size of the regions is determined by the @c regionVoxelDimensions argument given on construction. Larger
/// regions consume more memory each, but are more suited to GPU based operations. The default size of 32*32*32, or an
/// equivalent volume is recommended.
///
/// The contents of an @c OccupancyMap are determined by the @c MapLayout with voxel data split into separate layers.
/// An @c OccupancyMap is always expected to support a occupancy layer of @c float values. A map may optionally
/// support additional, standard layers and user defined layers. The standard layers are:
///
/// Layer Name    | Type          | Purpose
/// ------------- | ------------- | --------------
/// occupancy     | @c float      | Stores occupancy values
/// mean          | @c VoxelMean  | Quantised voxel mean coordinates and sample count
/// covariance    | @c CovarianceVoxel  | Tracks the covariance within the voxel for @c NdtMap
/// clearance     | @c float      | Distance to the nearest occupied voxel (experimental - see @c ClearanceProcess )
///
/// Voxels may be accessed directly from @c MapChunk entries accessed via @c region() calls. This provides direct
/// access to the voxel layer memory and requires manual decoding of voxel layer content and indexing. Direct access
/// may yield the best performance, but requires advanced use code. The @c Voxel<T> template class may be used to
/// access voxel data via a convenient abstraction and provides some validation of the voxel content against the
/// requested data type.
///
/// For example, the occupancy layer and the @c VoxelMean may be accessed with the following code:
///
/// @code
/// ohm::OccupancyMap map(0.1, ohm::MapFlag::kVoxelMean);
///
/// ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
/// ohm::Voxel<ohm::VoxelMean> mean(&map, map.layout().occupancyLayer());
///
/// const glm::dvec3 sample{1, 2, 3};
/// occupancy.setKey(map.voxelKey(sample);
/// mean.setKey(occupancy.key());
///
/// ohm::integrateHit(occupancy);
/// ohm::updatePosition(mean);
/// @endcode
///
/// The map is typically updated by collecting point samples with corresponding sensor origin coordinates via a
/// @c RayMapper class such as @c RayMapperOccupancy . More manual updates may also be made by passing each
/// origin/sample pair to @c calculateSegmentKeys() . This identifies the voxels intersected by the line segment
/// connecting the two points. These voxels, excluding the voxel containing the sample, should be updated in the map
/// by calling @c integrateMiss() reinforcing such voxels as free. For sample voxels, @c integrateHit() should be
/// called.
///
/// There are two forms of addressing for the @c OccupancyMap - region and local - encapsulated by the @c Key class.
/// The region key is a coarse indexer which identifies a @c MapRegion and its associated memory in @c MapChunk .
/// The local key is a fine indexer which resolves to a specific voxel within a @c MapRegion . Region indexing is
/// limited by the size of an @c int16_t supporting a spatial range of
/// ~`2^15 * regionVoxelDimensions() * resolution()` while the local key is limited by @c regionVoxelDimensions() .
///
/// @par Compression
/// The dense memory architecture used by this class creates an inherent memory overhead. Regions are readily
/// sparsely populated, but the memory allocation for a region is fixed. @c MapFlag::kCompressed may be used alleviate
/// the memory usage. In this mode an @c OccupancyMap uses a background compression thread to compress voxel data. The
/// background compression thread compresses regions which have not been touched for some time.
///
/// The background compression does impose a some CPU overhead and latency especially when iterating the map as a
/// whole to ensure voxel data are uncompressed when needed. The overhead is minimal when not using compression.
///
/// @todo Consider ways to modify the ohm API to better support `std::shared_ptr<OccupancyMap>`. Current occupancy map
/// usage encourages stack allocation or @c std::unique_ptr usage of the map, but then ends up passing ray pointer and
/// marks borrowed pointers to other objects - such as a @c GpuMap . This obfuscates ownership. The plan would be to
/// use @c std::shared_ptr<OccupancyMap> in these borrowed pointer cases. The ideal is to continue to allow stack
/// allocation of a map be ensuring the map lifetime exceeds any borrowed ownership. This needs testing and
/// verification, but the goal is to use smart pointers rather than raw pointers. At worst this may require
/// deriving @c std::enable_shared_from_this<OccupancyMap>
class ohm_API OccupancyMap
{
public:
  /// Base iterator class.
  class ohm_API base_iterator  // NOLINT
  {
  public:
    /// Invalid constructor.
    base_iterator();
    /// Base iterator into @p map starting at @p key. Map must remain unchanged during iteration.
    /// @param map The map to iterate in.
    /// @param key The key to start iterating at.
    base_iterator(OccupancyMap *map, const Key &key);
    /// Copy constructor.
    /// @param other Object to shallow copy.
    base_iterator(const base_iterator &other);
    /// Destructor.
    ~base_iterator();

    /// Key for the current voxel.
    /// @return Current voxel key.
    inline const Key &key() const { return key_; }

    /// Access the @c MapChunk the iterator is currently referencing.
    /// @return The current @c MapChunk .
    const MapChunk *chunk() const;

    /// Access the @c OccupancyMap the iterator is currently referencing.
    /// @return The current @c OccupancyMap .
    inline const OccupancyMap *map() const { return map_; }

    /// Copy assignment.
    /// @param other Object to shallow copy.
    base_iterator &operator=(const base_iterator &other);

    /// Comparison: equality. Comparison of invalid iterators is ill defined unless referencing the same map.
    /// @param other Iterator to compare against.
    /// @return True if @c this is logically equivalent to @p other.
    bool operator==(const base_iterator &other) const;

    /// Comparison: inequality.
    /// @param other Iterator to compare against.
    /// @return True if @c this is not logically equivalent to @p other.
    bool operator!=(const base_iterator &other) const;

    /// Is this iterator still valid?
    /// @return True if the iterator may be safely dereferenced.
    bool isValid() const;

    /// Dereference the iterator as a key.
    /// @return The current voxel key.
    inline const Key &operator*() const { return key_; }

    /// Dereference the iterator as a key.
    /// @return The current voxel key.
    inline const Key *operator->() const { return &key_; }

  protected:
    /// Move to the next voxel. Iterator becomes invalid if already referencing the last voxel.
    /// Iterator is unchanged if already invalid.
    void walkNext();

    OccupancyMap *map_ = nullptr;  ///< The referenced map.
    Key key_;                      ///< The current voxel key.
    /// Memory used to track an iterator into a hidden container type.
    /// We use an anonymous, fixed size memory chunk and placement new to prevent exposing STL
    /// types as part of the ABI.
    std::array<uint8_t, 32> chunk_mem_;  // NOLINT(readability-magic-numbers)
  };

  /// Non-constant iterator into an @c OccupancyMap.
  class ohm_API iterator : public base_iterator  // NOLINT
  {
  public:
    /// Constructor of an invalid iterator.
    iterator() = default;
    /// Iterator into @p map starting at @p key . Map must remain unchanged during iteration.
    /// @param map The map to iterate in.
    /// @param key The key to start iterating at.
    inline iterator(OccupancyMap *map, const Key &key)
      : base_iterator(map, key)
    {}
    /// Copy constructor.
    /// @param other Object to shallow copy.
    inline iterator(const iterator &other) = default;

    /// Empty destructor.
    ~iterator() = default;

    using base_iterator::chunk;
    /// @overload
    MapChunk *chunk() const;

    using base_iterator::map;
    /// @overload
    inline OccupancyMap *map() { return map_; }

    /// Prefix increment for the iterator. Iterator becomes invalid when incrementing an iterator
    /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
    /// @return This iterator after the increment.
    inline iterator &operator++()
    {
      walkNext();
      return *this;
    }

    /// Postfix increment for the iterator. Iterator becomes invalid when incrementing an iterator
    /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
    /// @return @c This iterator before the increment.
    inline iterator operator++(int)
    {
      iterator iter(*this);
      walkNext();
      return iter;
    }
  };

  /// Constant (read only) iterator for an @c OccupancyMap.
  class ohm_API const_iterator : public base_iterator  // NOLINT
  {
  public:
    /// Constructor of an invalid iterator.
    const_iterator() = default;
    /// Iterator into @p map starting at @p key. Map must remain unchanged during iteration.
    /// @param map The map to iterate in.
    /// @param key The key to start iterating at.
    inline const_iterator(OccupancyMap *map, const Key &key)
      : base_iterator(map, key)
    {}
    /// Copy constructor.
    /// @param other Object to shallow copy.
    inline const_iterator(const const_iterator &other) = default;
    /// Copy constructor.
    /// @param other Object to shallow copy.
    inline const_iterator(const iterator &other)  // NOLINT(google-explicit-constructor) want implicit conversion here
      : base_iterator(other)
    {}

    /// Prefix increment for the iterator. Iterator becomes invalid when incrementing an iterator
    /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
    /// @return This iterator after the increment.
    inline const_iterator &operator++()
    {
      walkNext();
      return *this;
    }
    /// Postfix increment for the iterator. Iterator becomes invalid when incrementing an iterator
    /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
    /// @return @c This iterator before the increment.
    inline const_iterator operator++(int)
    {
      const_iterator iter(*this);
      walkNext();
      return iter;
    }
  };

  /// Construct an @c OccupancyMap at the given voxels resolution.
  ///
  /// The @p regionVoxelDimensions controls the size of the map regions. The number of voxels
  /// along each axis is set by each axis of @p regionVoxelDimensions, thus the total volume of
  /// each region is set by the product of the axes. The memory usage of the region voxels is
  /// determined by multiplying the region volume by the size of:
  /// - float for the occupancy value
  /// - float for the clearance value
  /// There is also a sub-sampled clearance array in each chunk (size TBC).
  ///
  /// @param resolution The resolution for a single voxel in the map. Any zero value
  ///   dimension is replaced with its default value; e.g., @c OHM_DEFAULT_CHUNK_DIM_X.
  /// @param region_voxel_dimensions Sets the number of voxels in each map region.
  /// @param flags Map initialisation flags. See @c MapFlag .
  /// @param seed_layout The @p MapLayout to create the map with. The constructed map clones the @c seed_layout
  ///   object. Omit to use the default layout.
  OccupancyMap(double resolution, const glm::u8vec3 &region_voxel_dimensions, MapFlag flags,
               const MapLayout &seed_layout);

  /// @overload
  explicit OccupancyMap(double resolution = 1.0, const glm::u8vec3 &region_voxel_dimensions = glm::u8vec3(0, 0, 0),
                        MapFlag flags = MapFlag::kDefault);

  /// @overload
  OccupancyMap(double resolution, MapFlag flags, const MapLayout &seed_layout);

  /// @overload
  OccupancyMap(double resolution, MapFlag flags);

  /// Destructor.
  ~OccupancyMap();

  // Iterator.
  /// Create an iterator to the first voxel in the map. The map should not have voxels added or removed
  /// during iterator.
  /// @return An @c iterator to the first voxel in the map, or an invalid iterator when empty.
  iterator begin();
  /// Create a read only iterator to the first voxel in the map. The map should not have voxels added or removed
  /// during iterator.
  /// @return An @c const_iterator to the first voxel in the map, or an invalid iterator when empty.
  const_iterator begin() const;

  /// Create an iterator representing the end of iteration. See standard iteration patterns.
  /// @return An invalid iterator for this map.
  iterator end();
  /// Create a read only iterator representing the end of iteration. See standard iteration patterns.
  /// @return An invalid iterator for this map.
  const_iterator end() const;

  /// Calculate the approximate memory usage of this map in bytes.
  /// @return The approximate memory usage (bytes).
  size_t calculateApproximateMemory() const;

  /// Get the voxel resolution of the occupancy map. Voxels are cubes.
  /// @return The leaf voxel resolution.
  double resolution() const;

  /// A cyclic stamp value which changes whenever the map is touched. May be used to note when the map has
  /// been changed from a previous stamp value.
  /// @return The current cyclic stamp value.
  uint64_t stamp() const;

  /// Touches the map, progressing the @c stamp() value.
  /// @return The @c stamp() value after the touch.
  uint64_t touch();

  /// Query the timestmap for the first ray in this map. This time is used to as a timebase reference for rays with
  /// timestamps allowing time values to be store relative to this time. This time is normally updated once they
  /// left unchanged. Changing this value will invalidate the touch time layer.
  /// @note Timestamps may be unavailable.
  /// @return The first ray timestamp or a negative value if not set.
  double firstRayTime() const;
  /// Set the @c firstRayTime() regardless of the current value.
  /// Not threadsafe.
  /// @param time The first ray timestamp.
  void setFirstRayTime(double time);
  /// Update the @c firstRayTime() setting it only if it has yet to be set.
  /// Not threadsafe.
  /// @param time The first ray timestamp.
  /// @return The @c firstRayTime()
  double updateFirstRayTime(double time);

  /// Query the spatial resolution of each @c MapRegion. This represents the spatial extents of each region.
  /// @return The spatial resolution of a @c MapRegion.
  glm::dvec3 regionSpatialResolution() const;

  /// Query the number of voxels in each region, split by axis.
  /// @return A vector identifying the number of voxels in each region along each respective axis.
  glm::u8vec3 regionVoxelDimensions() const;

  /// Query the total number of voxels per region.
  /// This is simply the product of the @p regionVoxelDimensions().
  /// @return The number of voxels in each region.
  size_t regionVoxelVolume() const;

  /// Calculate the minimum spatial coordinate for the region identified by @p region_key.
  ///
  /// The region need not be present in the map for this calculation.
  /// @param region_key The region of interest.
  /// @return The minimum spatial extent coordinate for the region.
  glm::dvec3 regionSpatialMin(const glm::i16vec3 &region_key) const;

  /// Calculate the maximum spatial coordinate for the region identified by @p region_key.
  ///
  /// The region need not be present in the map for this calculation.
  /// @param region_key The region of interest.
  /// @return The maximum spatial extent coordinate for the region.
  glm::dvec3 regionSpatialMax(const glm::i16vec3 &region_key) const;

  /// Calculate the spatial centre for the region identified by @p region_key.
  ///
  /// The region need not be present in the map for this calculation.
  /// @param region_key The region of interest.
  /// @return The coordinates of the spatial centre of the region.
  glm::dvec3 regionSpatialCentre(const glm::i16vec3 &region_key) const;

  /// Sets the map origin. All point references are converted to be relative to this origin.
  /// Changing the origin will effectively shift all existing voxels.
  /// @param origin The map origin.
  void setOrigin(const glm::dvec3 &origin);

  /// Gets the map origin.
  /// @return The map origin.
  const glm::dvec3 &origin() const;

  /// Calculate the extents of the map based on existing regions containing known data.
  /// @param[out] min_ext Set to the minimum corner of the axis aligned extents. May be nullptr.
  /// @param[out] max_ext Set to the maximum corner of the axis aligned extents. May be nullptr.
  /// @param[out] key_range The key range enclosing the key extents.
  /// @return True if the map is non-empty and resulting extents are valid. False for an empty map in which case the
  ///   out values are undefined.
  bool calculateExtents(glm::dvec3 *min_ext, glm::dvec3 *max_ext, KeyRange *key_range = nullptr) const;

  /// Calculate the extents of the map based on existing regions containing known data.
  /// @param[out] min_ext Set to the minimum corner of the axis aligned extents. May be nullptr.
  /// @param[out] max_ext Set to the maximum corner of the axis aligned extents. May be nullptr.
  /// @param[out] min_key Set to the voxel key of the minimum corner of the axis aligned extents. May be nullptr.
  /// @param[out] max_key Set to the voxel key of the maximum corner of the axis aligned extents. May be nullptr.
  /// @return True if the map is non-empty and resulting extents are valid. False for an empty map in which case the
  ///   out values are undefined.
  bool calculateExtents(glm::dvec3 *min_ext, glm::dvec3 *max_ext, Key *min_key, Key *max_key = nullptr) const;

  /// Access to the map info structure for storing general meta data.
  ///
  /// This structure is serialised with the map.
  ///
  /// Some values are reserved for heightmap. See @c Heightmap.
  ///
  /// @return The map info structure.
  MapInfo &mapInfo();

  /// @overload
  const MapInfo &mapInfo() const;

  /// Get the initialisation flags for the map.
  /// @return Initialisation flags.
  MapFlag flags() const;

  //-------------------------------------------------------
  // Region management.
  //-------------------------------------------------------

  /// May layout controlling layers.
  /// @return Layout details.
  const MapLayout &layout() const;

  /// May layout controlling layers. Note: define a custom layout the existing layout may need to be cleared, or
  // reset to only preserve the occupancy layer.
  /// @return Layout details.
  MapLayout &layout();

  /// Adds the voxel layer required to track voxel mean position (@c VoxelMean). This invalidates any existing
  /// @c Voxel or direct data references.
  ///
  /// Note note that changing the requires all map chunks have additional voxel layers allocated.
  ///
  /// Does nothing if already present.
  void addVoxelMeanLayer();

  /// Is voxel mean positioning enabled on this map?
  /// @return True if the @c VoxelMean layer is enabled.
  bool voxelMeanEnabled() const;

  /// Add the "traversal" layer to this map. This adds @c default_layer::traversalLayerName() containing one float
  /// per voxel. This value accumulates the distance travelled through the voxels by all previously intersected rays.
  ///
  /// Note note that changing the requires all map chunks have additional voxel layers allocated.
  ///
  /// Does nothing if the layer is already present.
  void addTraversalLayer();

  /// Is traversal calculation enabled on this map?
  /// @return True if the "traversal" layer is enabled.
  bool traversalEnabled() const;

  /// Add the "touch_time" layer to the map. See @c addTouchTime() . Does nothing if the layer is already present.
  void addTouchTimeLayer();

  /// Check if the "touch_time" layer exists.
  /// @return True if the "touch_time" layer is enabled.
  bool touchTimeEnabled() const;

  /// Add the "incident_normal" layer to the map. See @c addIncidentNormal() . Does nothing if the layer is already
  /// present.
  void addIncidentNormalLayer();

  /// Check if the "incident_normal" layer exists.
  /// @return True if the "incident_normal" layer is enabled.
  bool incidentNormalEnabled() const;

  /// Ensure a voxel layer called @p layer_name is present, invoking @p add_layer_function to add it if necessary.
  ///
  /// If the layer is not present, then a copy of the @c MapLayout is first made and modified by calling
  /// @p add_layer_function . The layer changes are committed using @c updateLayout() . It is assumed that
  /// @p add_layer_function only modifies the @c MapLayout by adding an appropriate layer matching @p layer_name .
  ///
  /// No action is performed if a voxel layer named @p layer_name is already present, under the assumption that there
  /// is only one valid function to use to add a layer matching @p layer_name .
  ///
  /// @param layer_name Name of the layer to be added.
  /// @param add_layer_function Function to invoke to add the layer if required.
  void addLayer(const char *layer_name, const std::function<void(MapLayout &)> &add_layer_function);

  /// Update the memory layout to match that in this map's @c MapLayout. Must be called after updating
  /// the @p layout() after construction.
  ///
  /// By default this will attempt to preserve all voxel layers which are equivalent between the two layouts (see
  /// @c MapLayer::checkEquivalent() ). Voxel layers not present in the new layer are destroyed, while new voxel
  /// layers are allocated. This may take some time to process.
  ///
  /// This behaviour may be modified, by setting @c preserve_map to @c false , in which case this call destroys the
  /// current map content.
  ///
  /// In both cases the GPU cache is invalidated.
  ///
  /// @param new_layout The map layout to update to.
  /// @param preserve_map Try to preserve the map content for equivalent layers?
  void updateLayout(const MapLayout &new_layout, bool preserve_map = true);

  /// Query the number of regions in the map which have been touched.
  /// @return The number of regions in the map.
  size_t regionCount() const;

  /// Expire @c MapRegion sections which have not been touched after @p timestamp.
  /// Such regions are removed from the map.
  ///
  /// This compares each @c MapRegion::touched_time against @p timestamp, removing regions
  /// with a @c touched_time before @p timestamp. The region touch time is updated via
  /// @c Voxel::touch(), @c touchRegionTimestamp() or @c touchRegionTimestampByKey().
  ///
  /// @param timestamp The reference time.
  /// @return The number of removed regions.
  unsigned expireRegions(double timestamp);

  /// Remove @c MapRegion chunks which are sufficiently far from @p relativeTo.
  ///
  /// Regions are removed if their centre is further than @p distance from @p relativeTo.
  ///
  /// @param relative_to The spatial reference point to measure distance relative to.
  /// @param distance The distance threshold.
  /// @return The number of removed regions.
  unsigned removeDistanceRegions(const glm::dvec3 &relative_to, float distance);

  /// Remove @c MapRegion chunks which do not overalp the given axis aligned box.
  ///
  /// @param min_extents The AABB minimum extents.
  /// @param max_extents The AABB minimum extents.
  /// @return The number of removed regions.
  unsigned cullRegionsOutside(const glm::dvec3 &min_extents, const glm::dvec3 &max_extents);

  /// Touch the @c MapRegion which contains @p point .
  /// @param point A spatial point from which to resolve a containing region. There may be border case issues.
  /// @param timestamp The timestamp to update the region touch time to.
  /// @param allow_create Create the region (all uncertain) if it doesn't exist?
  /// @see @c touchRegionTimestampByKey()
  inline void touchRegionTimestamp(const glm::vec3 &point, double timestamp, bool allow_create = false)
  {
    touchRegionTimestamp(voxelKey(point), timestamp, allow_create);
  }

  /// Touch the @c MapRegion which contains @p voxel_key .
  /// @param voxel_key A voxel key from which to resolve a containing region.
  /// @param timestamp The timestamp to update the region touch time to.
  /// @param allow_create Create the region (all uncertain) if it doesn't exist?
  /// @see @c touchRegionTimestampByKey()
  inline void touchRegionTimestamp(const Key &voxel_key, double timestamp, bool allow_create = false)
  {
    touchRegionTimestampByKey(voxel_key.regionKey(), timestamp, allow_create);
  }

  /// Touch the @c MapRegion identified by @p regionKey.
  ///
  /// If @p regionKey resolves to a valid @c MapRegion, then its @c MapRegion::touched_time is set
  /// to @p timestamp. Otherwise the call is ignored.
  ///
  /// @param region_key The key for the region.
  /// @param timestamp The timestamp to update the region touch time to.
  /// @param allow_create Create the region (all uncertain) if it doesn't exist?
  void touchRegionTimestampByKey(const glm::i16vec3 &region_key, double timestamp, bool allow_create = false);

  /// Returns the centre of the region identified by @p regionKey.
  ///
  /// The region may or may not be present in the map. This function simply calculates where this region would lie.
  ///
  /// @param region_key The region of interest.
  /// @return The centre of the region @p regionKey in global coordinates.
  glm::dvec3 regionCentreGlobal(const glm::i16vec3 &region_key) const;

  /// Returns the centre of the region identified by @p regionKey in map local coordinates.
  ///
  /// The region may or may not be present in the map. This function simply calculates where this region would lie.
  ///
  /// @param region_key The region of interest.
  /// @return The centre of the region @p regionKey in map local coordinates.
  glm::dvec3 regionCentreLocal(const glm::i16vec3 &region_key) const;

  /// Calculates the region key for the key containing @p point (global coordinates).
  /// @param point The global coordinate point of interest.
  /// @return The key of the region containing @p point.
  glm::i16vec3 regionKey(const glm::dvec3 &point) const;

  //-------------------------------------------------------
  // Probabilistic map functions.
  //-------------------------------------------------------

  /// Defines the value adjustment made to a voxel when integrating a hit into the map.
  /// This is equivalent to <tt>valueToProbability(hitProbability())</tt>.
  ///
  /// The value must be positive to represent an increase in probability.
  ///
  /// @return The value adjustment made to a voxel on recording a 'hit'.
  /// @see integrateHit()
  float hitValue() const;
  /// The probability adjustment made to a voxel when integrating a hit into the map.
  /// @return The probability adjustment represented by a 'hit' in the map.
  /// @see integrateHit()
  float hitProbability() const;
  /// Sets the voxel occupancy probability used to represent a single hit in the map.
  ///
  /// The probability of voxel occupancy and the accumulation of such is equivalent to the
  /// logic used by Octomap, which serves as good reference material.
  ///
  /// Note the @p probabilty must be at least 0.5 for well define behaviour. Otherwise each hit
  /// will actually decrease the occupancy probability of a voxel.
  ///
  /// Probability do not affect already populated voxels in the map, only future updates.
  ///
  /// @param probability The new probability associated with a hit. Should be in the range (0.5, 1.0].
  void setHitProbability(float probability);
  /// Set the voxel occupancy hit value.
  /// @param value New hit value; must be > 0 for well defined behaviour.
  /// @see setHitProbability()
  void setHitValue(float value);

  /// Defines the value adjustment made to a voxel when integrating a miss or free space into the map.
  /// This is equivalent to <tt>valueToProbability(missProbability())</tt>.
  ///
  /// The value must be negative to represent an decrease in probability.
  ///
  /// @return The value adjustment made to a voxel on recording a 'hit'.
  /// @see integrateMiss()
  float missValue() const;
  /// The probability adjustment made to a voxel when integrating a miss into the map.
  /// @return The probability adjustment represented by a 'miss' in the map.
  /// @see integrateMiss()
  float missProbability() const;
  /// Sets the voxel occupancy probability used to represent a single miss in the map.
  ///
  /// The probability of voxel occupancy and the accumulation of such is equivalent to the
  /// logic used by Octomap, which serves as good reference material.
  ///
  /// Note the @p probabilty must be less than 0.5 for well define behaviour. Otherwise each miss
  /// will actually increase the occupancy probability of a voxel.
  ///
  /// Probability do not affect already populated voxels in the map, only future updates.
  ///
  /// @param probability The new probability associated with a miss. Should be in the range [0, 0.5).
  void setMissProbability(float probability);
  /// Set the voxel occupancy miss value.
  /// @param value New miss value; must be < 0 for well defined behaviour.
  /// @see setMissProbability()
  void setMissValue(float value);

  /// Get threshold value at which a voxel is considered occupied.
  /// This is equivalent to <tt>valueToProbability(occupancyThresholdProbability())</tt>.
  /// Use of this value is not generally recommended as @c occupancyThresholdProbability() provides
  /// a more meaningful value.
  /// @return The value at which a voxel is considered occupied.
  float occupancyThresholdValue() const;
  /// Get threshold probability at which a voxel is considered occupied.
  /// @return The probabilty [0, 1] at which a voxel is considered occupied.
  float occupancyThresholdProbability() const;
  /// Set the threshold at which to considere a voxel occupied.
  ///
  /// Setting a value less than 0.5 is not recommended as this can include "miss" results integrated
  /// into the map.
  ///
  /// @param probability The new occupancy threshold [0, 1].
  void setOccupancyThresholdProbability(float probability);

  /// The minimum value a voxel can have. Value adjustments are clamped to this minimum.
  /// @return The minimum voxel value.
  float minVoxelValue() const;

  /// Set the minimum value a voxel can have. Value adjustments are clamped to this minimum.
  /// Changing the minimum value does not affect any existing voxels with smaller values.
  /// @param value The new minimum value.
  void setMinVoxelValue(float value);

  /// Set the minimum value for a voxel expressed as a probability.
  /// @see @c setMinVoxelValue
  /// @param probability The minimum probability value allowed.
  void setMinVoxelProbability(float probability);

  /// Get the minimum value for a voxel expressed as a probability.
  /// @return The minimum voxel occupancy probability.
  float minVoxelProbability() const;

  /// Do voxels saturate at the minimum value, preventing further adjustment?
  /// @return True if voxels saturate and maintain state at the minimum value.
  bool saturateAtMinValue() const;

  /// Set the voxel minimum value saturation value. Saturation behaviour can be circumvented
  /// using some value adjustment functions.
  /// @param saturate True to have voxels prevent further value adjustments at the minimum value.
  void setSaturateAtMinValue(bool saturate);

  /// The maximum value a voxel can have. Value adjustments are clamped to this maximum.
  /// @return The maximum voxel value.
  float maxVoxelValue() const;

  /// Set the maximum value a voxel can have. Value adjustments are clamped to this maximum.
  /// Changing the maximum value does not affect any existing voxels with larger values.
  /// @param value The new maximum value.
  void setMaxVoxelValue(float value);

  /// Set the maximum value for a voxel expressed as a probability.
  /// @see @c setMinVoxelValue
  /// @param probability The maximum probability value allowed.
  void setMaxVoxelProbability(float probability);

  /// Get the maximum value for a voxel expressed as a probability.
  /// @return The maximum voxel occupancy probability.
  float maxVoxelProbability() const;

  /// Do voxels saturate at the maximum value, preventing further adjustment?
  /// @return True if voxels saturate and maintain state at the maximum value.
  bool saturateAtMaxValue() const;

  /// Set the voxel maximum value saturation value. Saturation behaviour can be circumvented
  /// using some value adjustment functions.
  /// @param saturate True to have voxels prevent further value adjustments at the minimum value.
  void setSaturateAtMaxValue(bool saturate);

  //-------------------------------------------------------
  // General map manipulation.
  //-------------------------------------------------------

  /// Retrieve the coordinates for the centre of the voxel identified by @p key local to the map origin.
  ///
  /// @param key The voxel of interest.
  /// @return The voxel coordinates, relative to the map @c origin().
  glm::dvec3 voxelCentreLocal(const Key &key) const;

  /// Calculate a voxel centre position.
  ///
  /// This is used by @c voxelCentreLocal() as `voxelCentre(key, map.voxelResolution())` and @c voxelCentreGlobal() as
  /// follows:
  ///
  /// @code
  /// void voxelCentres(const ohm::OccupancyMap &map, const ohm::Key &key,
  ///                   glm::dvec3 *local_centre, glm::dvec3 *global_centre)
  /// {
  ///   // voxelCentreLocal() equivalent call.
  ///   *local_centre = ohm::OccupancyMap::voxelCentre(key, map.resolution(), map.regionSpatialResolution());
  ///   // voxelCentreGlobal() equivalent call.
  ///   *global_centre = ohm::OccupancyMap::voxelCentre(key, map.resolution(), map.regionSpatialResolution(),
  ///                                                   map.origin());
  /// }
  /// @endcode
  ///
  ///
  /// @param key Voxel key for which to resolve the voxel centre.
  /// @param voxel_resolution The size of the voxels. From @c resolution() .
  /// @param region_spatial_dimensions The spatial dimensions of each @c MapRegion . From @c regionSpatialResolution() .
  /// @param map_origin The origin of the occupancy map. From @c origin() .
  static inline glm::dvec3 voxelCentre(const Key &key, double voxel_resolution,
                                       const glm::dvec3 &region_spatial_dimensions,
                                       const glm::dvec3 &map_origin = glm::dvec3(0.0))
  {
    glm::dvec3 centre;
    // Region centre
    centre = glm::vec3(key.regionKey());
    // Note: converting imp_->region_spatial_dimensions to glm::vec3 then multiplying to vec3 values resulted in
    // additional floating point error. The following component wise multiplication of float/int generates better
    // values.
    centre.x *= region_spatial_dimensions.x;
    centre.y *= region_spatial_dimensions.y;
    centre.z *= region_spatial_dimensions.z;
    // Offset to the lower extents of the region.
    centre -= 0.5 * region_spatial_dimensions;
    // Map offset.
    centre += map_origin;
    // Local offset.
    centre += glm::dvec3(key.localKey()) * voxel_resolution;
    centre += glm::dvec3(0.5 * voxel_resolution);
    return centre;
  }

  /// Retrieve the global coordinates for the centre of the voxel identified by @p key. This includes
  /// the map @c origin().
  ///
  /// @param key The voxel of interest.
  /// @return The global voxel coordinates.
  glm::dvec3 voxelCentreGlobal(const Key &key) const;

  /// Convert a global coordinate to the key value for the containing voxel.
  /// The voxel may not currently exist in the map.
  /// @param point A global coordinate to convert.
  /// @return The @c Key for the voxel containing @p point.
  Key voxelKey(const glm::dvec3 &point) const;

  /// @overload
  Key voxelKey(const glm::vec3 &point) const;

  /// Convert a local coordinate to the key value for the containing voxel.
  /// @param local_point A map local coordinate (relative to @c origin()) to convert.
  /// @return The @c Key for the voxel containing @p localPoint.
  Key voxelKeyLocal(const glm::vec3 &local_point) const;

  /// Move an @c Key along a selected axis.
  ///
  /// This function moves @p key along the selected @p axis. The direction and
  /// magnitude of the movement is determined by the sign and magnitude of @p step.
  /// Valid @p axis values are [0, 1, 2] corresponding to the X, Y, Z axes respectively.
  ///
  /// For example, to move @p key to it's previous neighbour along the X axis, call:
  /// <tt>moveKeyAlongAxis(key, 0, -1)</tt>. To move a key 16 voxels along the Y axis,
  /// call <tt>moveKeyAlongAxis(key, 1, 16)</tt>.
  ///
  /// Use of this method ensures the @p key region is updated as required when at the limits
  /// of the local indexing.
  ///
  /// @param key The key to move.
  /// @param axis The axis to move along: [0, 1, 2] mapping to X, Y, Z respectively.
  /// @param step Defines the step direction and magnitude along the selected @p axis.
  void moveKeyAlongAxis(Key &key, int axis, int step) const;

  /// Step the @p key along the @p axis in the given @p dir.
  ///
  /// This is a faster version of @c moveKeyAlongAxis() for when stepping just one voxel at a time.
  ///
  /// @param key The key to modify.
  /// @param axis The axis to modify. Must be [0, 2] mapping to XYZ respectively, or behaviour is undefined.
  /// @param dir Direction to step. Must be 1 or -1 or behaviour is undefined.
  /// @param region_voxel_dimensions The number of voxels in each region given per axis. See @c regionVoxelDimensions().
  static inline void stepKey(Key &key, int axis, int dir, const glm::ivec3 &region_voxel_dimensions)
  {
    int local_key = key.localKey()[axis] + dir;
    int region_key = key.regionKey()[axis];

    if (local_key < 0)
    {
      --region_key;
      local_key = region_voxel_dimensions[axis] - 1;
    }
    else if (local_key >= region_voxel_dimensions[axis])
    {
      ++region_key;
      local_key = 0;
    }

    key.setLocalAxis(axis, uint8_t(local_key));
    key.setRegionAxis(axis, uint16_t(region_key));
  }

  /// @overload
  void stepKey(Key &key, int axis, int dir) const;

  /// Move an @c Key by a given offset.
  ///
  /// This function moves @p key by the given @p x, @p y, @p z voxel offsets.
  /// Each of the offsets determine the direction and magnitude of movement along the
  /// X, Y and Z axes respectively.
  ///
  /// For example, to move @p key to it's previous neighbour along the X axis, call:
  /// <tt>moveKey(key, -1, 0, 0)</tt>. To move a key 16 voxels along the Y axis and -2
  /// voxels along Z, call <tt>moveKey(key, 0, 16, -2)</tt>.
  ///
  /// Use of this method ensures the @p key region is updated as required when at the limits
  /// of the local indexing.
  ///
  /// @param key The key to move.
  /// @param x The voxel offset to apply to @p key on the X axis.
  /// @param y The voxel offset to apply to @p key on the Y axis.
  /// @param z The voxel offset to apply to @p key on the X axis.
  void moveKey(Key &key, int x, int y, int z) const;

  /// @overload
  template <typename VecType>
  inline void moveKey(Key &key, const VecType &v) const
  {
    moveKey(key, v.x, v.y, v.z);
  }

  /// Calculate the number of voxels along each axis between two keys. Semantically, <tt>b - a</tt>.
  /// @param from The first key.
  /// @param to The second key.
  /// @return The voxel offset from @p from to @p to along each axis.
  glm::ivec3 rangeBetween(const Key &from, const Key &to) const;

  /// A static overload which requires the region dimensions in voxels for a region.
  /// @param from The first key.
  /// @param to The second key.
  /// @param region_voxel_dimensions The number of voxels in each region given per axis. See @c regionVoxelDimensions().
  /// @return The voxel offset from @p from to @p to along each axis.
  static inline glm::ivec3 rangeBetween(const Key &from, const Key &to, const glm::ivec3 &region_voxel_dimensions)
  {
    // First diff the regions.
    const glm::ivec3 region_diff = to.regionKey() - from.regionKey();
    glm::ivec3 voxel_diff;

    // Voxel difference is the sum of the local difference plus the region step difference.
    for (int i = 0; i < 3; ++i)
    {
      voxel_diff[i] =
        int(to.localKey()[i]) - int(from.localKey()[i]) + region_diff[i] * int(region_voxel_dimensions[i]);
    }

    return voxel_diff;
  }

  /// Set the range filter applied to all rays to be integrated into the map. @c RayMapper implementations must
  /// respect this filter in @c RayMapper::integrateRays() .
  /// @param ray_filter The range filter to install and filter rays with. Accepts an empty, which clears the filter.
  void setRayFilter(const RayFilterFunction &ray_filter);

  /// Get the range filter applied to all rays given to @c integrateRays().
  /// The map starts with a filter configured to remove any infinite or NaN points by rejecting rays longer than
  /// @c 1e10.
  /// @return The current ray filter.
  const RayFilterFunction &rayFilter() const;

  /// Clears the @c rayFilter().
  void clearRayFilter();

  /// Integrate the given @p rays into the map. The @p rays form a list of origin/sample pairs for which
  /// we generally consider the sample voxel as a hit when (increasing occupancy) and all other voxels as misses
  /// (free space). The sample may be treated as a miss when @p endPointsAsOccupied is false.
  ///
  /// @param rays Array of origin/sample point pairs.
  /// @param element_count The number of points in @p rays. The ray count is half this value.
  /// @param intensities An array of intensity values matching the @p rays items. There is one intensity value per ray
  ///   so there are @c element_count/2 items. May be null to omit intensity values.
  /// @param timestamps An array of timestamp values matching the @p rays items. There is one timestamp value per ray
  ///   so there are @c element_count/2 items. May be null to omit timestamp values in which case the touch time layer
  ///   will not be updated.
  /// @param ray_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
  void integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities = nullptr,
                     const double *timestamps = nullptr, unsigned ray_update_flags = kRfDefault);

  //-------------------------------------------------------
  // Data copy/cloning
  //-------------------------------------------------------

  /// Clone the entire map.
  /// @return A deep clone of this map. Caller takes ownership.
  OccupancyMap *clone() const;

  /// Clone the map within the given extents.
  ///
  /// This creates a deep clone of this may, copying only regions which overlap the given extents.
  /// Note that any region which partially overmaps the extents is copied in its entirety.
  ///
  /// @param min_ext The minimum spatial extents to over.
  /// @param max_ext The maximum spatial extents to over.
  /// @return A deep clone of this map. Caller takes ownership.
  OccupancyMap *clone(const glm::dvec3 &min_ext, const glm::dvec3 &max_ext) const;

  //-------------------------------------------------------
  // Internal
  //-------------------------------------------------------

  /// Internal occupancy map detail access.
  /// @return Internal map details.
  inline OccupancyMapDetail *detail() { return imp_; }
  /// Internal occupancy map detail access.
  /// @return Internal map details.
  inline const OccupancyMapDetail *detail() const { return imp_; }

  /// Enumerate the regions within this map.
  /// @param[out] chunks The enumerated chunks are added to this container.
  void enumerateRegions(std::vector<const MapChunk *> &chunks) const;

  /// Fetch a region, potentially creating it. For internal use.
  /// @param region_key The key of the region to fetch.
  /// @param allow_create Create the region if it doesn't exist?
  /// @return A pointer to the requested region. Null if it doesn't exist and @p allowCreate is @c false.
  MapChunk *region(const glm::i16vec3 &region_key, bool allow_create = false);

  /// @overload
  const MapChunk *region(const glm::i16vec3 &region_key) const;

  /// Populate @c regions with a list of regions who's touch stamp is greater than the given value.
  ///
  /// Adds to @p regions without clearing it, thus there may be redundancy.
  ///
  /// @param from_stamp The map stamp value from which to fetch regions.
  /// @param regions The list to add to.
  unsigned collectDirtyRegions(uint64_t from_stamp, std::vector<std::pair<uint64_t, glm::i16vec3>> &regions) const;

  /// Experimental: calculate the extents of regions which have been changed since @c from_stamp .
  /// @param from_stamp The base stamp used to determine dirty regions.
  /// @param min_ext The region key which identifies the minimum extents of the dirty regions.
  /// @param max_ext The region key which identifies the maximum extents of the dirty regions.
  /// @return The most up to date stamp value for the dirty regions.
  uint64_t calculateDirtyExtents(uint64_t from_stamp, glm::i16vec3 *min_ext, glm::i16vec3 *max_ext) const;

  /// Experimental: calculate the dirty region extents for the clearance layer.
  /// @param min_ext The region key which identifies the minimum extents of the dirty regions.
  /// @param max_ext The region key which identifies the maximum extents of the dirty regions.
  /// @param region_padding Number of regions to pad the returned extents.
  void calculateDirtyClearanceExtents(glm::i16vec3 *min_ext, glm::i16vec3 *max_ext, unsigned region_padding = 0) const;

  /// Clear the map content and release map memory.
  void clear();

private:
  Key firstIterationKey() const;
  MapChunk *newChunk(const Key &for_key);
  static void releaseChunk(const MapChunk *chunk);

  /// Culling function for @c cullRegions().
  using RegionCullFunc = std::function<bool(const MapChunk &)>;

  /// Remove regions/chunks for which @c cull_func returns true.
  /// @param cull_func The culling criteria.
  /// @return The number of regions removed.
  unsigned cullRegions(const RegionCullFunc &cull_func);

  OccupancyMapDetail *imp_;
};

inline void OccupancyMap::setMinVoxelProbability(float probability)
{
  setMinVoxelValue(probabilityToValue(probability));
}

inline float OccupancyMap::minVoxelProbability() const
{
  return valueToProbability(minVoxelValue());
}

inline void OccupancyMap::setMaxVoxelProbability(float probability)
{
  setMaxVoxelValue(probabilityToValue(probability));
}

inline float OccupancyMap::maxVoxelProbability() const
{
  return valueToProbability(maxVoxelValue());
}
}  // namespace ohm

#endif  // OCCUPANCYMAP_H
