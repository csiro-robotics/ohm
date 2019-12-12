// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAP_H
#define HEIGHTMAP_H

#include "OhmConfig.h"

#include "Aabb.h"
#include "UpAxis.h"

#include <memory>

#include <glm/fwd.hpp>

#include <vector>

namespace ohm
{
  struct HeightmapDetail;
  class Key;
  class MapInfo;
  class OccupancyMap;
  class VoxelConst;

  /// A 2D voxel map variant which calculate a heightmap surface from another @c OccupancyMap.
  ///
  /// The heightmap is built from an @c OccupancyMap and forms an axis aligned collapse of that map. The up axis may be
  /// specified on construction of the heightmap, but must be aligned to a primary axis. The heightmap is build in
  /// its own @c OccupancyMap, which consists of a single layer of voxels. The @c MapLayout for the heightmap is
  /// two layers:
  /// - **occupancy** layer
  ///   - float occupancy
  /// - *heightmap* layer (named from @c HeightmapVoxel::kHeightmapLayer)
  ///   - @c HeightmapVoxel
  ///
  /// The height specifies the absolute height of the surface, while clearance denotes how much room there is above
  /// the surface voxel before the next obstruction. Note that the height values always increase going up, so the
  /// height value will be inverted when using any @c UpAxis::kNegN @c UpAxis value. Similarly, the clearance is always
  /// positive unless there are no further voxels above the surface, in which case the clearance is zero
  /// (no information).
  ///
  /// The height for each heightmap voxel is simplistically calculated by finding the its corresponding voxel column in
  /// the source map. The algorithm then walks up this column. By default, there is only one column per heightmap voxel.
  /// This may create incorrect results when the two maps use different voxel resolutions.
  ///
  /// A @c blurLevel() may be set to expand the column. Each blur factor expands the search space into a square around
  /// the original column. A blur level of 1 tests 9 voxels per source map layer to populate a heightmap voxel, blur 2
  /// uses 25, 3 uses 49, etc. Note that setting a blur level prevents the heightmap voxels from storing laterally
  /// offset positions (i.e., prevents lateral sub-voxel positioning in the heightmap).
  ///
  /// The resulting heightmap may be accessed via @c heightmap().
  ///
  /// The @c OccupancyMap used to represent the heightmap has additional meta data stored in it's @c MapInfo:
  /// - <b>heightmap</b> - Present and true if this is a heightmap.
  /// - <b>heightmap-axis</b> - The up axis ID for a heightmap.
  /// - <b>heightmap-axis-x</b> - The up axis X value for a heightmap.
  /// - <b>heightmap-axis-y</b> - The up axis Y value for a heightmap.
  /// - <b>heightmap-axis-z</b> - The up axis Z value for a heightmap.
  /// - <b>heightmap-blur</b> - The blur value used to generate the heightamp.
  /// - <b>heightmap-clearance</b> - The clearance value used to generate the heightamp.
  ///
  /// @todo Is this a @c Query?
  class Heightmap
  {
  public:
    /// Size of regions in the heightmap. This is a 2D voxel extent. The region height is always one voxel.
    static const unsigned kDefaultRegionSize = 128;
    /// Voxel value assigned to heightmap cells which represent a real surface extracted from the source map.
    static constexpr float kHeightmapSurfaceValue = 1.0f;
    /// Voxel value assigned to heightmap cells which represent a virtual surface extracted from the source map.
    /// Virtual surfaces may be formed by the interface between a free voxel supported by an uncertain/null voxel.
    static constexpr float kHeightmapVirtualSurfaceValue = -1.0f;
    /// Voxel value assigned to heightmap cells which have no valid voxel in the entire column from the source map.
    static constexpr float kHeightmapVacantValue = 0.0f;

    /// Construct a default initialised heightmap.
    Heightmap();

    /// Construct a new heightmap optionally tied to a specific @p map.
    /// @param grid_resolution The grid resolution for the heightmap. Should match the source map for best results.
    /// @param min_clearance The minimum clearance value expected above each surface voxel.
    /// @param up_axis Identifies the up axis for the map.
    /// @param region_size Grid size of each region in the heightmap.
    Heightmap(double grid_resolution, double min_clearance, UpAxis up_axis = UpAxis::kZ, unsigned region_size = 0);

    /// Destructor.
    ~Heightmap();

    /// Set number of threads to use in heightmap generation, enabling multi-threaded code path as required.
    ///
    /// Setting the @p thread_count to zero enabled multi-threading using the maximum number of threads. Setting the
    /// @p thread_count to 1 disables threads (default).
    ///
    /// Using multiple threads may not yield significant gains.
    ///
    /// @param thread_count The number of threads to set.
    /// @return True if mult-threading is available. False when no mult-threading is available and @p thread_count is
    /// ignored.
    bool setThreadCount(unsigned thread_count);

    /// Get the number of threads to use.
    ///
    /// - 0: use all available
    /// - 1: force single threaded, or no multi-threading is available.
    /// - n: Use n threads.
    /// @return The number of threads to use.
    unsigned threadCount() const;

    /// Set the occupancy map on which to base the heightmap. The heightmap does not takes no ownership of the pointer
    /// so the @p map must persist until @c update() is called.
    void setOccupancyMap(OccupancyMap *map);

    /// Access the current source occupancy map.
    OccupancyMap *occupancyMap() const;

    /// Access the currently generated heightmap.
    OccupancyMap &heightmap() const;

    /// Set the floor level. Points below this distance below the base height in the source map are ignored.
    /// @param floor The new floor value. Positive to enable.
    void setFloor(double floor);

    /// Get the floor level. Points above this distance above the base height in the source map are ignored.
    /// @return The floor value.
    double floor() const;

    /// Set the ceiling level. Points above this distance above the base height in the source map are ignored.
    /// @param ceiling The new ceiling value. Positive to enable.
    void setCeiling(double ceiling);

    /// Get the ceiling level. Points above this distance above the base height in the source map are ignored.
    /// @return The ceiling value.
    double ceiling() const;

    /// Set the minimum clearance required above a voxel in order to consider it a heightmap voxel.
    /// @param clearance The new clearance value.
    void setMinClearance(double clearance);

    /// Get the minimum clearance required above a voxel in order to consider it a heightmap voxel.
    /// @return The height clearance value.
    double minClearance() const;

    /// Sets whether sub-voxel positions are ignored (true) forcing the use of voxel centres.
    /// @param ignore True to force voxel centres even when sub-voxel positions are present.
    void setIgnoreSubVoxelPositioning(bool ignore);

    /// Force voxel centres even when sub-voxel positions are present?
    /// @return True to ignore sub-voxel positioning.
    /// @seealso @ref subvoxel
    bool ignoreSubVoxelPositioning() const;

    /// Set the generation of a heightmap floor around the transition from unknown to free voxels?
    ///
    /// This option allows a heightmap floor to be generated in columns where there is no clear occupied floor voxel.
    /// When enabled, the heightmap generates a floor level at the lowest transition point from unknown to free voxel.
    ///
    /// @param enable Enable this option?
    void setGenerateVirtualFloor(bool enable);

    /// Allow the generation of a heightmap floor around the transition from unknown to free voxels?
    ///
    /// @see @c setGenerateVirtualFloor()
    ///
    /// @retrun True if this option is enabled.
    bool generateVirtualFloor() const;

    /// The layer number which contains @c HeightmapVoxel structures.
    /// @return The heightmap layer index or -1 on error (not present).
    /// @seealso @ref subvoxel
    int heightmapVoxelLayer() const;

    /// The layer number which contains @c HeightmapVoxel structures during heightmap construction.
    /// @return The heightmap build layer index or -1 on error (not present).
    int heightmapVoxelBuildLayer() const;

    /// Get the up axis identifier used to generate the heightmap.
    UpAxis upAxis() const;

    /// Get the up axis index [0, 2] marking XYZ respectively. Ignores direction.
    int upAxisIndex() const;

    /// Get the normal vector for the up axis used to last @c update().
    const glm::dvec3 &upAxisNormal() const;

    /// Component index of the first surface axis normal [0, 2].
    int surfaceAxisIndexA() const;

    /// Get a unit vector which lies along the surface of the heightmap, perpendicular to @c surfaceAxisB() and
    /// upAxisNormal().
    const glm::dvec3 &surfaceAxisA() const;

    /// Component of the second surface axis normal [0, 2].
    int surfaceAxisIndexB() const;

    /// Get a unit vector which lies along the surface of the heightmap, perpendicular to @c surfaceAxisA() and
    /// upAxisNormal().
    const glm::dvec3 &surfaceAxisB() const;

    /// Static resolution of @c Axis to a normal.
    /// @param id The @c Axis ID.
    static const glm::dvec3 &upAxisNormal(UpAxis axis_id);

    /// Get a unit vector which lies along the surface of the heightmap, perpendicular to @c surfaceAxisB() and
    /// upAxisNormal().
    static const glm::dvec3 &surfaceAxisA(UpAxis axis_id);

    /// Get a unit vector which lies along the surface of the heightmap, perpendicular to @c surfaceAxisA() and
    /// upAxisNormal().
    static const glm::dvec3 &surfaceAxisB(UpAxis axis_id);

    /// The last base height value given to @p update().
    double baseHeight() const;

    void seedLocalCache(const glm::dvec3 &reference_pos);

    /// Update the heightmap around a reference position. This sets the @c base_height as in the overload, but also
    /// changes the behaviour to flood fill out from the reference position.
    ///
    /// @param reference_pos The staring position to build a heightmap around. Nominally a vehicle position.
    /// @return true on success.
    bool update(const glm::dvec3 &reference_pos, const ohm::Aabb &cull_to = ohm::Aabb(0.0));

    bool heightmapVoxelPosition(const VoxelConst &heightmap_voxel, glm::dvec3 *pos, float *clearance = nullptr) const;

  public:
    //-------------------------------------------------------
    // Internal
    //-------------------------------------------------------
    /// @internal
    inline HeightmapDetail *detail() { return imp_.get(); }
    /// @internal
    inline const HeightmapDetail *detail() const { return imp_.get(); }

    /// Update @c info to reflect the details of how the heightmap is generated. See class comments.
    /// @param info The info object to update.
    void updateMapInfo(MapInfo &info) const;  // NOLINT(google-runtime-references)

    /// Ensure that @p key is referencing a voxel within the heightmap plane.'
    /// @param key[in,out] The key to project. May be modified by this call. Must not be null.
    /// @return A reference to @p key.
    Key &project(Key *key);

  private:
    void updateLocalCache(const glm::dvec3 &reference_pos);

    bool lookupLocalCache(const glm::dvec3 &lookup_pos, glm::dvec3 *cache_pos, float *cache_value, double *clearance);

    std::unique_ptr<HeightmapDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAP_H
