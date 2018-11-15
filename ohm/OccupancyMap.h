// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYMAP_H
#define OCCUPANCYMAP_H

#include "OhmConfig.h"

#include "Key.h"
#include "Voxel.h"

#include <glm/glm.hpp>

#include <functional>
#include <vector>

#define OHM_DEFAULT_CHUNK_DIM_X 32
#define OHM_DEFAULT_CHUNK_DIM_Y OHM_DEFAULT_CHUNK_DIM_X
#define OHM_DEFAULT_CHUNK_DIM_Z OHM_DEFAULT_CHUNK_DIM_X

namespace ohm
{
  class MapCache;
  struct MapChunk;
  class MapLayout;
  struct OccupancyMapDetail;
  class KeyList;

  /// A spatial container using a voxel representation of 3D space.
  ///
  /// The map is divided into rectangular regions of voxels. The map uses an initial spatial
  /// hash to identify a larger @c MapRegion which contains a contiguous array of voxels.
  /// This used rather than an octree to support:
  /// - Fast region allocation/de-allocation
  /// - Constant lookup
  /// - Region dropping regions
  /// - Eventually, out of core serialisation.
  ///
  /// The size of the regions is determined by the @c regionVoxelDimensions argument
  /// given on construction. Larger regions consume more memory each, but are more suited
  /// to GPU based operations. The default size of 32*32*32, or an equivalent volume is
  /// recommended.
  ///
  /// Nodes in the tree have two associated values: an occupancy value and a user value.
  /// The occupancy value may be updated probabilistically, but this only serves in when
  /// it is possible to mark free voxels in the tree as well as unoccupied voxels, or
  /// voxel occupancy may decay over time.
  ///
  /// Voxels are never accessed directly. Rather, various access methods return
  /// an @c Voxel or @c VoxelCost wrapper objects. Voxels may have three
  /// states as defined by @c OccupancyType [uncertain, free, occupied], accessible via
  /// @c occupancyType(). The state is entirely dependent on a voxel's @c value()
  /// with values greater than or equal to the @c occupancyThresholdValue()
  /// being occupied, values less than the threshold are free. Uncertain voxels have a value
  /// equal to @c voxel::invalidMarkerValue().
  ///
  /// The map is typically updated by collecting point samples with corresponding sensor origin
  /// coordinates. For each origin/sample pair, @c calculateSegmentKeys() is used to identify
  /// the voxels intersected by the line segment connecting the two. These voxels, excluding
  /// the voxel containing the same, should be updated in the map by calling @c integrateMiss()
  /// reinforcing such voxels as free. For sample voxels, @c integrateHit() should be called.
  ///
  /// Some methods, such as @c integrateHit() and @c integrateMiss() support a @c MapCache argument.
  /// This may provide a small performance benefit when repeatedly accessing the same region. This
  /// will tend to be the case when processing results from @c calculateSegmentKeys().
  class ohm_API OccupancyMap
  {
  public:
    /// Base iterator class.
    class ohm_API base_iterator // NOLINT
    {
    public:
      /// Invalid constructor.
      base_iterator();
      /// Base iterator into @p map starting at @p key. Map must remain unchanged during iteration.
      /// @param map The map to iterate in.
      /// @param key The key to start iterating at.
      base_iterator(OccupancyMapDetail *map, const Key &key);
      /// Copy constructor.
      /// @param other Object to shallow copy.
      base_iterator(const base_iterator &other);
      /// Destructor.
      ~base_iterator();

      /// Key for the current voxel.
      /// @return Current voxel key.
      inline const Key &key() const { return key_; }

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

      /// Access the voxel this iterator refers to. Iterator must be valid before calling.
      /// @return An constant reference wrapper for the iterator's voxel.
      VoxelConst voxel() const;

    protected:
      /// Move to the next voxel. Iterator becomes invalid if already referencing the last voxel.
      /// Iterator is unchanged if already invalid.
      void walkNext();

      OccupancyMapDetail *map_; ///< The referenced map.
      Key key_;        ///< The current voxel key.
      /// Memory used to track an iterator into a hidden container type.
      /// We use an anonymous, fixed size memory chunk and placement new to prevent exposing STL
      /// types as part of the ABI.
      uint8_t chunk_mem_[32];
    };

    /// Non-constant iterator into an @c OccupancyMap.
    class iterator : public base_iterator // NOLINT
    {
    public:
      /// Constructor of an invalid iterator.
      iterator() = default;
      /// Iterator into @p map starting at @p key. Map must remain unchanged during iteration.
      /// @param map The map to iterate in.
      /// @param key The key to start iterating at.
      inline iterator(OccupancyMapDetail *map, const Key &key) : base_iterator(map, key) { }
      /// Copy constructor.
      /// @param other Object to shallow copy.
      inline iterator(const iterator &other) : base_iterator(other) {}

      /// Empty destructor.
      ~iterator() = default;

      /// Prefix increment for the iterator. Iterator becomes invalid when incrementing an iterator
      /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
      /// @return This iterator after the increment.
      inline iterator &operator++() { walkNext(); return *this; }

      /// Postfix increment for the iterator. Iterator becomes invalid when incrementing an iterator
      /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
      /// @return @c This iterator before the increment.
      inline iterator operator++(int) { iterator iter(*this);  walkNext(); return iter; }

      /// Access the voxel this iterator refers to. Iterator must be valid before calling.
      /// @return A non-constant reference wrapper for the iterator's voxel.
      Voxel voxel();
      using base_iterator::voxel;

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline Voxel &operator *() { resolveNode(); return voxel_; }

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline const Voxel &operator *() const { resolveNode(); return voxel_; }

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline Voxel *operator ->() { resolveNode(); return &voxel_; }

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline const Voxel *operator ->() const { resolveNode(); return &voxel_; }

    private:
      /// Resolve the @c base_iterator data into @c _voxel.
      inline void resolveNode() const { voxel_ = const_cast<iterator *>(this)->voxel(); }

      /// A cached @c Voxel version of the underlying @c base_iterator data, here to support
      /// the @c * and @c -> operators.
      mutable Voxel voxel_;
    };

    /// Constant (read only) iterator for an @c OccupancyMap.
    class const_iterator : public base_iterator // NOLINT
    {
    public:
      /// Constructor of an invalid iterator.
      const_iterator() = default;
      /// Iterator into @p map starting at @p key. Map must remain unchanged during iteration.
      /// @param map The map to iterate in.
      /// @param key The key to start iterating at.
      inline const_iterator(OccupancyMapDetail *map, const Key &key) : base_iterator(map, key) {}
      /// Copy constructor.
      /// @param other Object to shallow copy.
      inline const_iterator(const base_iterator &other) : base_iterator(other) {}

      /// Prefix increment for the iterator. Iterator becomes invalid when incrementing an iterator
      /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
      /// @return This iterator after the increment.
      inline const_iterator &operator++() { walkNext(); return *this; }
      /// Postfix increment for the iterator. Iterator becomes invalid when incrementing an iterator
      /// referencing the last voxel in the map. Safe to call on an invalid iterator (no change).
      /// @return @c This iterator before the increment.
      inline const_iterator operator++(int) { const_iterator iter(*this);  walkNext(); return iter; }

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline const VoxelConst &operator *() const { resolveNode(); return voxel_; }

      /// Alias for @c voxel().
      /// @return Same as @c voxel().
      inline const VoxelConst *operator ->() const { resolveNode(); return &voxel_; }

    private:
      /// Resolve the @c base_iterator data into @c _voxel.
      inline void resolveNode() const { voxel_ = voxel(); }

      /// A cached @c VoxelConst version of the underlying @c base_iterator data, here to support
      /// the @c * and @c -> operators.
      mutable VoxelConst voxel_;
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
    /// See also @c voxelMemoryPerRegion().
    ///
    /// @param resolution The resolution for a single voxel in the map. Any zero value
    ///   dimension is replaced with its default value; e.g., @c OHM_DEFAULT_CHUNK_DIM_X.
    /// @param region_voxel_dimensions Sets the number of voxels in each map region.
    OccupancyMap(double resolution = 1.0, const glm::u8vec3 &region_voxel_dimensions = glm::u8vec3(0, 0, 0));
    /// Destructor.
    ~OccupancyMap();

    /// Calculates the memory allocated for each region's voxels set based on the given @p regionVoxelDimensions.
    /// @param region_voxel_dimensions Voxel dimensions of each chunk. Any zero value
    ///   dimension is replaced with its default value; e.g., @c OHM_DEFAULT_CHUNK_DIM_X.
    /// @return The memory required for the voxels in each region in bytes.
    static size_t voxelMemoryPerRegion(glm::u8vec3 region_voxel_dimensions = glm::u8vec3(0, 0, 0));

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

    /// Access the voxel at @p key, optionally creating the voxel if it does not exist.
    ///
    /// The key for any spatial coordinate can be generated by @c voxelKey().
    ///
    /// @param key The key for the voxel to access.
    /// @param allow_create True to create the voxel if it does not exist.
    /// @param cache Optional cache used to expidite region search.
    /// @return An @c Voxel wrapper object for the keyed @p MayNode. The voxel is
    ///   null if does not exist and creation is not allowed.
    Voxel voxel(const Key &key, bool allow_create, MapCache *cache = nullptr);
    /// @overload
    VoxelConst voxel(const Key &key, MapCache *cache = nullptr) const;

    /// Determine the @c OccupancyType of @p voxel
    /// @param voxel The voxel to check. May be null.
    /// @return The @c OccupancyType of @p voxel.
    int occupancyType(const VoxelConst &voxel) const;

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
    void touch();

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

    glm::dvec3 regionSpatialMin(const glm::i16vec3 &region_key) const;
    glm::dvec3 regionSpatialMax(const glm::i16vec3 &region_key) const;
    glm::dvec3 regionSpatialCentre(const glm::i16vec3 &region_key) const;

    /// Sets the map origin. All point references are converted to be relative to this origin.
    /// Changing the origin will effectively shift all existing voxels.
    /// @param origin The map origin.
    void setOrigin(const glm::dvec3 &origin);

    /// Gets the map origin.
    /// @return The map origin.
    const glm::dvec3 &origin() const;

    /// Calculate the extents of the map based on existing regions containing known data.
    /// @param[out] min_ext Set to the minimum corner of the axis aligned extents.
    /// @param[out] max_ext Set to the maximum corner of the axis aligned extents.
    void calculateExtents(glm::dvec3 &min_ext, glm::dvec3 &max_ext) const;

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

    /// Query the number of regions in the map.
    size_t regionCount() const;

    /// Expire @c MapRegion sections which have not been touched after @p timestamp.
    /// Such regions are removed from the map.
    ///
    /// This compares each @c MapRegion::touched_time against @p timestamp, removing regions
    /// with a @c touched_time before @p timestamp. The region touch time is updated via
    /// @c Voxel::touch(), @c touchRegion() or @c touchRegionByKey().
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

    /// Touch the @c MapRegion which contains @p point.
    /// @param point A spatial point from which to resolve a containing region. There may be border case issues.
    /// @param timestamp The timestamp to update the region touch time to.
    /// @param allow_create Create the region (all uncertain) if it doesn't exist?
    /// @see @c touchRegionByKey()
    inline void touchRegion(const glm::vec3 &point, double timestamp, bool allow_create = false) { touchRegion(voxelKey(point), timestamp, allow_create); }

    /// Touch the @c MapRegion which contains @p voxelKey.
    /// @param voxel_key A voxel key from which to resolve a containing region.
    /// @param timestamp The timestamp to update the region touch time to.
    /// @param allow_create Create the region (all uncertain) if it doesn't exist?
    /// @see @c touchRegionByKey()
    inline void touchRegion(const Key &voxel_key, double timestamp, bool allow_create = false) { touchRegionByKey(voxel_key.regionKey(), timestamp, allow_create); }

    /// Touch the @c MapRegion identified by @p regionKey.
    ///
    /// If @p regionKey resolves to a valid @c MapRegion, then its @c MapRegion::touched_time is set
    /// to @p timestamp. Otherwise the call is ignored.
    ///
    /// @param region_key The key for the region.
    /// @param timestamp The timestamp to update the region touch time to.
    /// @param allow_create Create the region (all uncertain) if it doesn't exist?
    void touchRegionByKey(const glm::i16vec3 &region_key, double timestamp, bool allow_create = false);

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

    /// Adjust @p voxel by increasing its occupancy probability. The value of the voxel is adjusted by
    /// adding @c hitValue(), which logically increases its occupancy probability by @c hitProbability().
    /// @param voxel The voxel to increase the occupancy probabilty for. Must be a valid, non-null voxel.
    void integrateHit(Voxel &voxel) const;

    /// Integrate a hit into the map, creating the occupancy voxel as required.
    /// @param point The global coordinate to integrate a hit at.
    /// @param cache Optional cache used to expidite region search.
    /// @return A reference to the voxel containing @p point after integrating the hit.
    Voxel integrateHit(const glm::dvec3 &point, MapCache *cache = nullptr);

    /// @overload
    Voxel integrateHit(const Key &key, MapCache *cache = nullptr);

    /// Adjust @p voxel by decreasing its occupancy probability. The value of the voxel is adjusted by
    /// adding @c missValue() which should be negative. This logically decreases its occupancy probability
    /// by @c missProbability().
    /// @param voxel The voxel to decrease the occupancy probabilty for. Must be a valid, non-null voxel.
    void integrateMiss(Voxel &voxel) const;

    /// Integrate a miss into the map, creating the occupancy voxel as required.
    /// @param point The global coordinate to integrate a hit at.
    /// @param cache Optional cache used to expidite region search.
    /// @return A reference to the voxel containing @p point after integrating the hit.
    Voxel integrateMiss(const glm::dvec3 &point, MapCache *cache = nullptr);

    /// @overload
    Voxel integrateMiss(const Key &key, MapCache *cache = nullptr);

    /// Adjust the value of @p voxel by forcibly setting its occupancy probabilty to @c hitProbability().
    /// @param voxel The voxel to increase the occupancy probabilty for. Must be a valid, non-null voxel.
    inline void setHit(Voxel &voxel) const { if (voxel.isValid()) voxel.setValue(hitValue()); }
    /// Adjust the value of @p voxel by forcibly setting its occupancy probabilty to @c missProbability().
    /// @param voxel The voxel to increase the occupancy probabilty for. Must be a valid, non-null voxel.
    inline void setMiss(Voxel &voxel) const { if (voxel.isValid()) voxel.setValue(missValue()); }

    /// The minimum value a voxel can have. Value adjustments are clamped to this minimum.
    /// @return The minimum voxel value.
    float minNodeValue() const;

    /// Set the minimum value a voxel can have. Value adjustments are clamped to this minimum.
    /// Changing the minimum value does not affect any existing voxels with smaller values.
    /// @param value The new minimum value.
    void setMinNodeValue(float value);

    /// Set the minimum value for a voxel expressed as a probability.
    /// @see @c setMinNodeValue
    /// @param probability The minimum probability value allowed.
    void setMinNodeProbability(float probability);

    /// Get the minimum value for a voxel expressed as a probability.
    /// @return The minimum voxel occupancy probability.
    float minNodeProbability() const;

    /// Do voxels saturate at the minimum value, preventing further adjustment?
    /// @return True if voxels saturate and maintain state at the minimum value.
    bool saturateAtMinValue() const;

    /// Set the voxel minimum value saturation value. Saturation behaviour can be circumvented
    /// using some value adjustment functions.
    /// @param saturate True to have voxels prevent further value adjustments at the minimum value.
    void setSaturateAtMinValue(bool saturate);

    /// The maximum value a voxel can have. Value adjustments are clamped to this maximum.
    /// @return The maximum voxel value.
    float maxNodeValue() const;

    /// Set the maximum value a voxel can have. Value adjustments are clamped to this maximum.
    /// Changing the maximum value does not affect any existing voxels with larger values.
    /// @param value The new maximum value.
    void setMaxNodeValue(float value);

    /// Set the maximum value for a voxel expressed as a probability.
    /// @see @c setMinNodeValue
    /// @param probability The maximum probability value allowed.
    void setMaxNodeProbability(float probability);

    /// Get the maximum value for a voxel expressed as a probability.
    /// @return The maximum voxel occupancy probability.
    float maxNodeProbability() const;

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

    /// Add a to to the map at @p key, settings its occupancy @p value.
    /// This creates map regions as required. Should the voxel already exist, then the existing
    /// voxel value is replaced by @p value.
    ///
    /// @param key Identifies the voxel to add (or replace).
    /// @param value The value to assign the voxel.
    /// @return A mutable reference to the voxel. This object may be short lived and should only be
    ///   briefly retained.
    /// @see voxelKey(), voxelKeyLocal()
    Voxel addNode(const Key &key, float value);

    /// Retrieve the coordinates for the centre of the voxel identified by @p key local to the map origin.
    ///
    /// @param key The voxel of interest.
    /// @return The voxel coordinates, relative to the map @c origin().
    glm::dvec3 voxelCentreLocal(const Key &key) const;

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

    /// Builds the list of voxel keys intersected by the line segment connecting @p startPoint and @p endPoint.
    ///
    /// The @p keys list is populated with all voxels intersected by the specified line segment.
    /// The voxel containing @p endPoint may or may not be included, depending on the value of @p includeEndPoint.
    /// Keys are added in order of traversal from @p startPoint to @p endPoint.
    ///
    /// Note this method clears @p keys on entry.
    ///
    /// @param keys The list to populate with intersected voxel keys.
    /// @param start_point The global coordinate marking the start of the line segment.
    /// @param end_point The global coordinate marking the end of the line segment.
    /// @param include_end_point @c true to incldue the voxel containing @p endPoint, @c false to exclude this
    ///   voxel from @p keys.
    /// @return The number of voxels added to @p keys.
    size_t calculateSegmentKeys(KeyList &keys, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                                bool include_end_point = true) const;

    /// Integrate the given @p rays into the map. The @p rays form a list of origin/sample pairs for which
    /// we generally consider the sample voxel as a hit when (increasing occupancy) and all other voxels as misses
    /// (free space). The sample may be treated as a miss when @p endPointsAsOccupied is false.
    ///
    /// @param rays Array of origin/sample point pairs.
    /// @param point_count The number of points in @p rays. The ray count is half this value.
    /// @param end_points_as_occupied When @c true, the end points of the rays increase the occupancy probability.
    ///   Otherwise they decrease the probability just as the rest of the ray.
    void integrateRays(const glm::dvec3 *rays, size_t point_count, bool end_points_as_occupied = true);

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
    /// @internal
    inline OccupancyMapDetail *detail() { return imp_; }
    /// @internal
    inline const OccupancyMapDetail *detail() const { return imp_; }

    /// Enumerate the regions within this map.
    /// @param[out] chunks The enumerated chunks are added to this container.
    void enumerateRegions(std::vector<const MapChunk *> &chunks) const;

    /// @internal
    /// Fetch a region, potentially creating it.
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

    void calculateDirtyExtents(uint64_t *from_stamp, glm::i16vec3 *min_ext, glm::i16vec3 *max_ext) const;
    void calculateDirtyClearanceExtents(glm::i16vec3 *min_ext, glm::i16vec3 *max_ext, unsigned region_padding = 0) const;

    /// Clear the map content and release map memory.
    void clear();

  private:
    Key firstIterationKey() const;
    MapChunk *newChunk(const Key &for_key);
    static void releaseChunk(const MapChunk *chunk);

    /// Culling function for @c cullRegions().
    using RegionCullFunc = std::function<bool (const MapChunk &)>;

    /// Remove regions/chunks for which @c cull_func returns true.
    /// @param cull_func The culling criteria.
    /// @return The number of regions removed.
    unsigned cullRegions(const RegionCullFunc &cull_func);

    OccupancyMapDetail *imp_;
  };


  inline void OccupancyMap::integrateHit(Voxel &voxel) const
  {
    if (voxel.isValid())
    {
      if (!voxel.isUncertain())
      {
        voxel.setValue(voxel.value() + hitValue());
      }
      else
      {
        voxel.setValue(hitValue());
      }
    }
  }


  inline Voxel OccupancyMap::integrateHit(const glm::dvec3 &point, MapCache *cache)
  {
    Key key = voxelKey(point);
    return integrateHit(key, cache);
  }


  inline Voxel OccupancyMap::integrateHit(const Key &key, MapCache *cache)
  {
    Voxel voxel = this->voxel(key, true, cache);
    integrateHit(voxel);
    return voxel;
  }


  inline void OccupancyMap::integrateMiss(Voxel &voxel) const
  {
    if (voxel.isValid())
    {
      if (!voxel.isUncertain())
      {
        voxel.setValue(voxel.value() + missValue());
      }
      else
      {
        voxel.setValue(missValue());
      }
    }
  }


  inline Voxel OccupancyMap::integrateMiss(const glm::dvec3 &point, MapCache *cache)
  {
    Key key = voxelKey(point);
    return integrateMiss(key, cache);
  }


  inline Voxel OccupancyMap::integrateMiss(const Key &key, MapCache *cache)
  {
    Voxel voxel = this->voxel(key, true, cache);
    integrateMiss(voxel);
    return voxel;
  }


  inline void OccupancyMap::setMinNodeProbability(float probability)
  {
    setMinNodeValue(probabilityToValue(probability));
  }


  inline float OccupancyMap::minNodeProbability() const
  {
    return valueToProbability(minNodeValue());
  }


  inline void OccupancyMap::setMaxNodeProbability(float probability)
  {
    setMaxNodeValue(probabilityToValue(probability));
  }


  inline float OccupancyMap::maxNodeProbability() const
  {
    return valueToProbability(maxNodeValue());
  }
}

#endif // OCCUPANCYMAP_H
